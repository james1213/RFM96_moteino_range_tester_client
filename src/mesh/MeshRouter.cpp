#include "MeshRouter.h"

MeshRouter *MeshRouter::instance = nullptr;

MeshRouter::MeshRouter(RadioManager *manager) {
    this->manager = manager;
    instance = this;
}

void MeshRouter::onDataReceived(void (*callback)(String &payload, uint8_t origin)) {
    dataReceivedCallback = callback;
}

void MeshRouter::setFrozen(bool value) {
    frozen = value;
}

void MeshRouter::loop() {
    if (frozen) return; // OTA: zadnych beaconow ani forwardingu, RAM dla transferu
    ageTables();
    if (millis() - lastBeaconMillis >= beaconDueInMs) {
        lastBeaconMillis = millis();
        if (sendBeacon()) {
            // Jitter: beacony roznych wezlow nie moga sie zsynchronizowac, bo
            // broadcast nie ma ACK - zderzenie beaconow jest niewykrywalne.
            beaconDueInMs = MESH_BEACON_INTERVAL_MS + (micros() & 0x1FF);
        } else {
            // Radio zajete albo brak RAM: ponow szybko, zamiast czekac cala kadencje -
            // przy gestym ruchu gubilismy w ten sposob wiekszosc slotow beaconowych.
            beaconDueInMs = 300 + (micros() & 0xFF);
        }
    }
}

// Kazda poprawna ramka od sasiada (dane, ACK, beacon) to dowod zycia lacza.
// Bez tego o zyciu decydowalyby same beacony - a te gina w kolizjach i zywe
// trasy padaly z komunikatem "zamilkl", mimo ze dane wciaz plynely.
void MeshRouter::noteFrameFrom(uint8_t senderId) {
    if (senderId == 0 || senderId == manager->nodeId) return;
    Neighbor *n = findNeighbor(senderId, false);
    if (n == nullptr) return; // sasiadow tworza tylko beacony - tylko one niosa moc TX
    n->lastHeardMillis = millis();
    // Wskrzeszenie trasy bezposredniej po timeoutach ACK: skoro ramka doszla,
    // lacze wrocilo - nie czekamy do nastepnego beaconu.
    Route *r = findRoute(senderId, true);
    if (r != nullptr && r->metric >= MESH_METRIC_INFINITY) {
        r->nextHop = senderId;
        r->metric = linkCost(*n);
    }
}

// ==================== JAKOSC LACZA I TRASY ====================

// Koszt lacza z wygladzonego tlumienia sciezki. Bazowy koszt kazdego skoku (4)
// premiuje trasy o mniejszej liczbie skokow - kazdy skok to airtime i ryzyko.
uint8_t MeshRouter::linkCost(const Neighbor &n) {
    uint16_t cost = MESH_LINK_COST_BASE;
    if (n.pathLossDb > MESH_LINK_GOOD_PATHLOSS) {
        cost += (n.pathLossDb - MESH_LINK_GOOD_PATHLOSS) / 8; // +1 na kazde 8 dB tlumienia
    }
    return cost > 60 ? 60 : (uint8_t) cost;
}

MeshRouter::Neighbor *MeshRouter::findNeighbor(uint8_t id, bool create) {
    Neighbor *freeSlot = nullptr;
    for (auto &n : neighbors) {
        if (n.id == id) return &n;
        if (n.id == 0 && freeSlot == nullptr) freeSlot = &n;
    }
    if (create && freeSlot != nullptr) {
        freeSlot->id = id;
        freeSlot->pathLossDb = 0;
        return freeSlot;
    }
    return nullptr;
}

MeshRouter::Route *MeshRouter::findRoute(uint8_t dest, bool create) {
    Route *freeSlot = nullptr;
    for (auto &r : routes) {
        if (r.dest == dest) return &r;
        if (r.dest == 0 && freeSlot == nullptr) freeSlot = &r;
    }
    if (create && freeSlot != nullptr) {
        freeSlot->dest = dest;
        freeSlot->metric = MESH_METRIC_INFINITY;
        return freeSlot;
    }
    return nullptr;
}

uint8_t MeshRouter::getNextHop(uint8_t dest) {
    Route *r = findRoute(dest, false);
    return (r != nullptr && r->metric < MESH_METRIC_INFINITY) ? r->nextHop : 0;
}

uint8_t MeshRouter::getRouteMetric(uint8_t dest) {
    Route *r = findRoute(dest, false);
    return r != nullptr ? r->metric : MESH_METRIC_INFINITY;
}

// Wezly sa w ruchu: sasiad, ktory zamilkl, i trasy przez niego musza zniknac
// szybko, a nie dopiero gdy inne wezly to ogłosza.
void MeshRouter::ageTables() {
    for (auto &n : neighbors) {
        if (n.id != 0 && millis() - n.lastHeardMillis > MESH_NEIGHBOR_TIMEOUT_MS) {
            Serial.print(F("MESH | sasiad "));
            Serial.print(n.id);
            Serial.println(F(" zamilkl - usuwam trasy przez niego"));
            invalidateRoutesVia(n.id);
            n.id = 0;
        }
    }
}

void MeshRouter::invalidateRoutesVia(uint8_t neighborId) {
    for (auto &r : routes) {
        if (r.dest != 0 && r.nextHop == neighborId) {
            r.metric = MESH_METRIC_INFINITY; // wpis zostaje: seq chroni przed starymi echami
        }
    }
}

bool MeshRouter::isDuplicate(uint8_t origin, uint8_t flowId) {
    for (uint8_t i = 0; i < MESH_DEDUP_SIZE; i++) {
        if (dedupOrigin[i] == origin && dedupId[i] == flowId) return true;
    }
    dedupOrigin[dedupNext] = origin;
    dedupId[dedupNext] = flowId;
    dedupNext = (dedupNext + 1) % MESH_DEDUP_SIZE;
    return false;
}

// ==================== BEACONY ====================

bool MeshRouter::sendBeacon() {
    uint8_t seqToSend = ownSeq + 1;
    if (seqToSend == 0) seqToSend = 1;
    String beacon;
    if (!beacon.reserve(24 + MESH_MAX_ROUTES * 12)) {
        Serial.println(F("MESH | brak RAM na beacon - pomijam"));
        return false;
    }
    beacon = F("<MSH>B?");
    beacon += (int) manager->apcMaxDbm; // moc, z jaka beacon FAKTYCZNIE poleci (sufit)
    beacon += '?';
    beacon += seqToSend;
    beacon += '?';
    bool first = true;
    for (auto &r : routes) {
        if (r.dest == 0) continue;
        if (!first) beacon += ',';
        first = false;
        beacon += r.dest;
        beacon += ':';
        beacon += r.metric;
        beacon += ':';
        beacon += r.seq;
    }
    // Broadcast bez ACK, zawsze na suficie mocy - patrz naglowek pliku.
    bool queued = manager->sendTaggedAtPower(beacon, RADIO_BROADCAST_ID, manager->apcMaxDbm);
    if (queued) ownSeq = seqToSend; // seq rosnie tylko dla beaconow, ktore poszly
    return queued;
}

// body wskazuje za "B?": "<mocTx>?<seq>?<cel>:<koszt>:<seq>,..."
void MeshRouter::handleBeacon(const char *body, uint8_t radioSender) {
    char *cursor;
    long beaconTxPower = strtol(body, &cursor, 10);
    if (*cursor != '?') return;
    long senderSeq = strtol(cursor + 1, &cursor, 10);
    if (*cursor != '?') return;
    cursor++;

    // Tlumienie lacza do nadawcy: znamy moc nadania (z beaconu) i RSSI odbioru.
    // EMA 3/4 starej + 1/4 nowej probki - RSSI pojedynczej ramki skacze o kilka dB.
    int pathLoss = (int) beaconTxPower - manager->getLastRssi();
    if (pathLoss < 0) pathLoss = 0;
    Neighbor *n = findNeighbor(radioSender, true);
    if (n == nullptr) return; // tablica pelna - sasiad poczeka na wolny slot
    if (n->pathLossDb == 0) n->pathLossDb = pathLoss;
    else n->pathLossDb = (n->pathLossDb * 3 + pathLoss) / 4;
    n->lastHeardMillis = millis();

    // Sam nadawca beaconu: trasa 1-skokowa, seq z beaconu.
    uint8_t directCost = linkCost(*n);
    Route *r = findRoute(radioSender, true);
    if (r != nullptr) {
        bool newer = (int8_t) ((uint8_t) senderSeq - r->seq) > 0;
        if (newer || r->metric >= MESH_METRIC_INFINITY
            || r->nextHop == radioSender
            || directCost + MESH_ROUTE_SWITCH_MARGIN < r->metric) {
            if (r->nextHop != radioSender && r->metric < MESH_METRIC_INFINITY) {
                Serial.print(F("MESH | trasa do "));
                Serial.print(radioSender);
                Serial.println(F(": teraz bezposrednio"));
            }
            r->nextHop = radioSender;
            r->metric = directCost;
            r->seq = (uint8_t) senderSeq;
        }
    }

    // Trasy ogloszone przez nadawce: DSDV - nowszy seq wygrywa zawsze, w ramach
    // tego samego seq obowiazuje histereza (ruch = wahania RSSI = ryzyko trzepotania).
    while (*cursor != '\0') {
        long dest = strtol(cursor, &cursor, 10);
        if (*cursor != ':') break;
        long metric = strtol(cursor + 1, &cursor, 10);
        if (*cursor != ':') break;
        long seq = strtol(cursor + 1, &cursor, 10);
        if (*cursor == ',') cursor++;

        if (dest == manager->nodeId || dest == radioSender || dest <= 0 || dest > 250) continue;
        uint16_t total = (uint16_t) directCost + (metric >= MESH_METRIC_INFINITY
                                                  ? MESH_METRIC_INFINITY : (uint16_t) metric);
        uint8_t candidate = total >= MESH_METRIC_INFINITY ? MESH_METRIC_INFINITY : (uint8_t) total;
        Route *route = findRoute((uint8_t) dest, candidate < MESH_METRIC_INFINITY);
        if (route == nullptr) continue;
        bool newer = (int8_t) ((uint8_t) seq - route->seq) > 0;
        bool sameHop = route->nextHop == radioSender;
        if (newer || sameHop || route->metric >= MESH_METRIC_INFINITY
            || candidate + MESH_ROUTE_SWITCH_MARGIN < route->metric) {
            if (!sameHop && route->metric < MESH_METRIC_INFINITY
                && candidate < MESH_METRIC_INFINITY) {
                Serial.print(F("MESH | trasa do "));
                Serial.print((int) dest);
                Serial.print(F(": via "));
                Serial.print(radioSender);
                Serial.print(F(" (koszt "));
                Serial.print(candidate);
                Serial.println(')');
            }
            route->nextHop = radioSender;
            route->metric = candidate;
            route->seq = (uint8_t) seq;
        }
    }
}

// ==================== DANE ====================

bool MeshRouter::send(uint8_t finalDest, const String &payload,
                      void (*okCallback)(), void (*failCallback)(String &)) {
    // RadioManager ma JEDEN slot callbackow ACK - drugi skok z ACK w locie
    // nadpisalby callbacki (i kontekst ponowien) tego pierwszego.
    if (manager->waitingForAck) return false;
    uint8_t nextHop = getNextHop(finalDest);
    if (nextHop == 0) {
        Serial.print(F("MESH | brak trasy do "));
        Serial.println(finalDest);
        return false;
    }
    String frame;
    if (!frame.reserve(payload.length() + 20)) {
        Serial.println(F("MESH | brak RAM na ramke danych - nie wyslano"));
        return false;
    }
    frame = F("<MSH>D?");
    frame += manager->nodeId;
    frame += '?';
    frame += finalDest;
    frame += '?';
    frame += MESH_MAX_TTL;
    frame += '?';
    frame += nextFlowId;
    frame += '?';
    frame += payload;
    isDuplicate(manager->nodeId, nextFlowId); // wlasna ramka do dedupu: echo ma zginac
    nextFlowId++;
    if (nextFlowId == 0) nextFlowId = 1;
    hopRetriesLeft = MESH_HOP_RETRIES;
    hopDest = nextHop;
    // ACK skok po skoku: "OK" u aplikacji = dotarlo do PIERWSZEGO posrednika.
    appOkCallback = okCallback;
    appFailCallback = failCallback;
    return manager->sendTagged(frame, nextHop, sHopAckOk, sHopAckFail);
}

void MeshRouter::sHopAckOk() {
    if (instance == nullptr) return;
    instance->hopRetriesLeft = 0;
    if (instance->appOkCallback) instance->appOkCallback();
}

void MeshRouter::sHopAckFail(String &taggedPayload) {
    if (instance != nullptr) instance->hopAckFail(taggedPayload);
}

// ACK skoku nie doszedl. Ponawiamy ograniczona liczbe razy, a potem uniewazniamy
// wszystkie trasy przez tego sasiada - wezly sa w ruchu, wiec brak ACK to zwykle
// "odjechal", a nastepne beacony i tak przyniosa swieza topologie.
void MeshRouter::hopAckFail(String &taggedPayload) {
    if (hopRetriesLeft > 0) {
        hopRetriesLeft--;
        Serial.println(F("MESH | skok bez ACK - ponawiam"));
        if (manager->sendTagged(taggedPayload, hopDest, sHopAckOk, sHopAckFail)) return;
    }
    Serial.print(F("MESH | sasiad "));
    Serial.print(hopDest);
    Serial.println(F(" nie potwierdza - uniewazniam trasy przez niego"));
    invalidateRoutesVia(hopDest);
    if (appFailCallback) appFailCallback(taggedPayload);
}

// str = payload po zdjeciu "<MSH>" (radio juz zdjelo naglowek ramki radiowej).
void MeshRouter::radioMeshDataReceived(String &str, uint8_t radioSender) {
    const char *body = str.c_str();
    if (body == nullptr || body[0] == '\0') return;
    if (body[0] == 'B' && body[1] == '?') {
        handleBeacon(body + 2, radioSender);
    } else if (body[0] == 'D' && body[1] == '?') {
        handleData(str, body + 2, radioSender);
    }
}

// body wskazuje za "D?": "<zrodlo>?<cel>?<ttl>?<id>?<tresc>"
void MeshRouter::handleData(String &str, const char *body, uint8_t radioSender) {
    (void) radioSender;
    char *cursor;
    long origin = strtol(body, &cursor, 10);
    if (*cursor != '?') return;
    long finalDest = strtol(cursor + 1, &cursor, 10);
    if (*cursor != '?') return;
    long ttl = strtol(cursor + 1, &cursor, 10);
    if (*cursor != '?') return;
    long flowId = strtol(cursor + 1, &cursor, 10);
    if (*cursor != '?') return;
    const char *payload = cursor + 1;

    if (origin <= 0 || origin > 250 || finalDest <= 0 || finalDest > 250) return;
    if (isDuplicate((uint8_t) origin, (uint8_t) flowId)) return;

    if ((uint8_t) finalDest == manager->nodeId) {
        // Dostarczenie: tresc wycinamy z ORYGINALNEGO Stringa (bez drugiej kopii).
        String delivered = str.substring(payload - str.c_str());
        if (dataReceivedCallback) dataReceivedCallback(delivered, (uint8_t) origin);
        return;
    }
    if (frozen) return;      // OTA: nie forwardujemy cudzych ramek
    if (ttl <= 1) {
        Serial.println(F("MESH | TTL wyczerpany - porzucam ramke"));
        return;
    }
    forwardData((uint8_t) origin, (uint8_t) finalDest, (uint8_t) (ttl - 1),
                (uint8_t) flowId, payload);
}

bool MeshRouter::forwardData(uint8_t origin, uint8_t finalDest, uint8_t ttl,
                             uint8_t flowId, const char *payload) {
    // Jeden skok z ACK naraz - patrz komentarz w send(). Porzucona ramka nie jest
    // tragedia: nadawca ma wlasne ponowienia, a dedup i tak przepusci powtorke.
    if (manager->waitingForAck) {
        Serial.println(F("MESH | forward pominiety - poprzedni skok czeka na ACK"));
        return false;
    }
    uint8_t nextHop = getNextHop(finalDest);
    if (nextHop == 0) {
        Serial.print(F("MESH | forward: brak trasy do "));
        Serial.println(finalDest);
        return false;
    }
    String frame;
    if (!frame.reserve(strlen(payload) + 20)) {
        Serial.println(F("MESH | brak RAM na forward - porzucam"));
        return false;
    }
    frame = F("<MSH>D?");
    frame += origin;
    frame += '?';
    frame += finalDest;
    frame += '?';
    frame += ttl;
    frame += '?';
    frame += flowId;
    frame += '?';
    frame += payload;
    Serial.print(F("MESH | forward "));
    Serial.print(origin);
    Serial.print(F("->"));
    Serial.print(finalDest);
    Serial.print(F(" via "));
    Serial.println(nextHop);
    hopRetriesLeft = MESH_HOP_RETRIES;
    hopDest = nextHop;
    appOkCallback = nullptr;   // forward nie jest nasza aplikacyjna wysylka
    appFailCallback = nullptr;
    return manager->sendTagged(frame, nextHop, sHopAckOk, sHopAckFail);
}

void MeshRouter::printTopology() {
    Serial.println(F("MESH | sasiedzi (id, tlumienie dB):"));
    for (auto &n : neighbors) {
        if (n.id == 0) continue;
        Serial.print(F("  "));
        Serial.print(n.id);
        Serial.print(F(" pl="));
        Serial.println(n.pathLossDb);
    }
    Serial.println(F("MESH | trasy (cel via nastepny, koszt, seq):"));
    for (auto &r : routes) {
        if (r.dest == 0) continue;
        Serial.print(F("  "));
        Serial.print(r.dest);
        Serial.print(F(" via "));
        Serial.print(r.nextHop);
        Serial.print(F(" koszt="));
        Serial.print(r.metric);
        Serial.print(F(" seq="));
        Serial.println(r.seq);
    }
}
