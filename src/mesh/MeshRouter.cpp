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
    if (value && !frozen) {
        // Zamrozenie obejmuje takze skok w locie: bez tego timeout ACK transakcji
        // sprzed zamrozenia ponawialby ramki mesh w srodku transferu OTA.
        hopRetriesLeft = 0;
        hopRetryPending = false;
        pendingForwardFrame = "";
        appOkCallback = nullptr;
        appFailCallback = nullptr;
    }
    frozen = value;
}

void MeshRouter::loop() {
    if (frozen) return; // OTA: zadnych beaconow ani forwardingu, RAM dla transferu
    ageTables();

    // Odlozone ponowienie skoku (timeout ACK trafil w zajete radio): ma
    // pierwszenstwo - trzyma slot transakcji, payload przetrwal w ackCallback_paylod.
    if (hopRetryPending && millis() >= hopRetryAtMillis && !manager->waitingForAck) {
        if (millis() > hopRetryDeadlineMillis
            || manager->ackCallback_paylod.length() == 0) {
            hopRetryPending = false;
            giveUpHop();
        } else if (manager->sendTagged(manager->ackCallback_paylod, hopDest,
                                       sHopAckOk, sHopAckFail)) {
            hopRetryPending = false;
        } else {
            hopRetryAtMillis = millis() + 150 + (micros() & 0x7F);
        }
    }

    // Odlozony forward: poprzedni skok juz potwierdzil te ramke, wiec nikt jej
    // nie ponowi - odsylamy ja, gdy tylko slot transakcji sie zwolni.
    if (pendingForwardFrame.length() > 0 && !manager->waitingForAck && !hopRetryPending) {
        if (millis() > pendingForwardDeadline) {
            Serial.println(F("MESH | odlozony forward przeterminowany - porzucam"));
            pendingForwardFrame = "";
        } else {
            hopRetriesLeft = MESH_HOP_RETRIES;
            hopDest = pendingForwardHop;
            appOkCallback = nullptr;
            appFailCallback = nullptr;
            if (manager->sendTagged(pendingForwardFrame, pendingForwardHop,
                                    sHopAckOk, sHopAckFail)) {
                pendingForwardFrame = "";
            }
        }
    }

    if (millis() - lastBeaconMillis >= beaconDueInMs) {
        lastBeaconMillis = millis();
        // Beacon nie nadaje, gdy nasluchujemy ACK skoku: radio jest poldupleksowe,
        // nadanie beaconu zagluszyloby wlasnie nadchodzace potwierdzenie.
        if (!manager->waitingForAck && !hopRetryPending && sendBeacon()) {
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
    Route *deadSlot = nullptr;
    for (auto &r : routes) {
        if (r.dest == dest) return &r;
        if (r.dest == 0 && freeSlot == nullptr) freeSlot = &r;
        if (r.dest != 0 && r.metric >= MESH_METRIC_INFINITY && deadSlot == nullptr) deadSlot = &r;
    }
    if (create) {
        // Wpisy INF nie znikaja same (pelnia role trucizny DSDV), wiec przy pelnej
        // tablicy oddajemy najpierw martwy wpis - inaczej po 6 roznych celach w
        // historii nowe wezly bylyby nieosiagalne az do restartu.
        if (freeSlot == nullptr) freeSlot = deadSlot;
        if (freeSlot != nullptr) {
            freeSlot->dest = dest;
            freeSlot->metric = MESH_METRIC_INFINITY;
            freeSlot->seq = 0; // slot moze byc z odzysku - stary seq nie ma tu prawa zyc
            return freeSlot;
        }
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

// Regula DSDV dla zerwanej trasy: uniewaznienie PODBIJA numer sekwencyjny celu.
// Bez tego wpis INF przegrywal z krazacym jeszcze ogloszeniem o tym samym seq
// i skonczonej metryce - dwa wezly potrafily sobie nawzajem "przywracac" trase
// do martwego celu az do nasycenia metryki (count-to-infinity). Zywy cel i tak
// wygra: jego wlasne beacony podnosza seq co ~3 s.
void MeshRouter::invalidateRoutesVia(uint8_t neighborId) {
    for (auto &r : routes) {
        if (r.dest != 0 && r.nextHop == neighborId && r.metric < MESH_METRIC_INFINITY) {
            r.metric = MESH_METRIC_INFINITY;
            r.seq++;
            if (r.seq == 0) r.seq = 1;
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
// Parsowanie strtoul-em, nie strtol-em: pola sa nieujemne, a strtoul i tak musi
// byc zlinkowany (CRC32 w OTA przekracza zakres long) - dzieki temu generyczny
// strtol (~560 B) w ogole nie trafia do binarki. Smieci ujemne/ogromne odrzuca
// walidacja zakresow ponizej.
void MeshRouter::handleBeacon(const char *body, uint8_t radioSender) {
    char *cursor;
    long beaconTxPower = (long) strtoul(body, &cursor, 10);
    if (*cursor != '?') return;
    long senderSeq = (long) strtoul(cursor + 1, &cursor, 10);
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

    // Sam nadawca beaconu: trasa 1-skokowa, seq z beaconu. Ogloszenie STARSZE od
    // naszego seq (np. po naszym podbiciu przy uniewaznieniu) nie ma prawa nic
    // zmienic - to wlasnie stare echa napedzaly count-to-infinity.
    uint8_t directCost = linkCost(*n);
    Route *r = findRoute(radioSender, true);
    if (r != nullptr) {
        bool newer = (int8_t) ((uint8_t) senderSeq - r->seq) > 0;
        bool older = (int8_t) ((uint8_t) senderSeq - r->seq) < 0;
        if (newer || (!older && (r->metric >= MESH_METRIC_INFINITY
            || r->nextHop == radioSender
            || directCost + MESH_ROUTE_SWITCH_MARGIN < r->metric))) {
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
        long dest = (long) strtoul(cursor, &cursor, 10);
        if (*cursor != ':') break;
        long metric = (long) strtoul(cursor + 1, &cursor, 10);
        if (*cursor != ':') break;
        long seq = (long) strtoul(cursor + 1, &cursor, 10);
        if (*cursor == ',') cursor++;

        if (dest == manager->nodeId) {
            // Odzysk ciaglosci seq po restarcie (DSDV): jesli siec pamieta nas z
            // wyzszym numerem, przeskakujemy go - inaczej nasze swieze beacony
            // bylyby "starsze" od widma sprzed restartu nawet przez kilka minut.
            if ((int8_t) ((uint8_t) seq - ownSeq) > 0) ownSeq = (uint8_t) seq;
            continue;
        }
        if (dest == radioSender || dest <= 0 || dest > 250) continue;
        uint16_t total = (uint16_t) directCost + (metric >= MESH_METRIC_INFINITY
                                                  ? MESH_METRIC_INFINITY : (uint16_t) metric);
        uint8_t candidate = total >= MESH_METRIC_INFINITY ? MESH_METRIC_INFINITY : (uint8_t) total;
        Route *route = findRoute((uint8_t) dest, candidate < MESH_METRIC_INFINITY);
        if (route == nullptr) continue;
        bool newer = (int8_t) ((uint8_t) seq - route->seq) > 0;
        bool older = (int8_t) ((uint8_t) seq - route->seq) < 0;
        bool sameHop = route->nextHop == radioSender;
        if (newer || (!older && (sameHop || route->metric >= MESH_METRIC_INFINITY
            || candidate + MESH_ROUTE_SWITCH_MARGIN < route->metric))) {
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
    // nadpisalby callbacki (i kontekst ponowien) tego pierwszego. Odlozone
    // ponowienie tez trzyma slot.
    if (manager->waitingForAck || hopRetryPending) return false;
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
    hopRetriesLeft = MESH_HOP_RETRIES;
    hopDest = nextHop;
    // ACK skok po skoku: "OK" u aplikacji = dotarlo do PIERWSZEGO posrednika.
    appOkCallback = okCallback;
    appFailCallback = failCallback;
    if (!manager->sendTagged(frame, nextHop, sHopAckOk, sHopAckFail)) return false;
    // Dedup i zuzycie id dopiero po udanym zakolejkowaniu - nieudana proba nic
    // nie nadala, wiec to samo id moze legalnie sprobowac ponownie.
    isDuplicate(manager->nodeId, nextFlowId); // wlasna ramka do dedupu: echo ma zginac
    nextFlowId++;
    if (nextFlowId == 0) nextFlowId = 1;
    return true;
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
    if (frozen) return; // OTA: transakcja sprzed zamrozenia wygasa bez ponowien i kar
    if (taggedPayload.length() == 0) {
        // Payload przepadl (cichy OOM Stringa) - nie ma czego ponawiac, a pusta
        // ramka i tak zostalaby odrzucona przez sendDirectly.
        giveUpHop();
        return;
    }
    if (hopRetriesLeft > 0) {
        Serial.println(F("MESH | skok bez ACK - ponawiam"));
        if (manager->sendTagged(taggedPayload, hopDest, sHopAckOk, sHopAckFail)) {
            hopRetriesLeft--; // proba zuzyta dopiero, gdy naprawde poszla w eter
            return;
        }
        // Radio zajete w chwili timeoutu to NIE strata w eterze: nie zuzywaj proby
        // i nie karz sasiada uniewaznieniem tras - odloz ponowienie (obsluga w loop()).
        if (!hopRetryPending) hopRetryDeadlineMillis = millis() + 2000;
        hopRetryPending = true;
        hopRetryAtMillis = millis() + 150 + (micros() & 0x7F);
        return;
    }
    giveUpHop();
}

void MeshRouter::giveUpHop() {
    Serial.print(F("MESH | sasiad "));
    Serial.print(hopDest);
    Serial.println(F(" nie potwierdza - uniewazniam trasy przez niego"));
    invalidateRoutesVia(hopDest);
    if (appFailCallback) {
        appFailCallback(manager->ackCallback_paylod);
        appFailCallback = nullptr;
    }
    appOkCallback = nullptr;
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
    long origin = (long) strtoul(body, &cursor, 10);
    if (*cursor != '?') return;
    long finalDest = (long) strtoul(cursor + 1, &cursor, 10);
    if (*cursor != '?') return;
    long ttl = (long) strtoul(cursor + 1, &cursor, 10);
    if (*cursor != '?') return;
    long flowId = (long) strtoul(cursor + 1, &cursor, 10);
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
    if (!manager->waitingForAck && !hopRetryPending) {
        hopRetriesLeft = MESH_HOP_RETRIES;
        hopDest = nextHop;
        appOkCallback = nullptr;   // forward nie jest nasza aplikacyjna wysylka
        appFailCallback = nullptr;
        if (manager->sendTagged(frame, nextHop, sHopAckOk, sHopAckFail)) {
            Serial.print(F("MESH | forward "));
            Serial.print(origin);
            Serial.print(F("->"));
            Serial.print(finalDest);
            Serial.print(F(" via "));
            Serial.println(nextHop);
            return true;
        }
    }
    // Slot transakcji zajety. Ramki NIE wolno porzucic: poprzedni skok juz dostal
    // jej radiowe ACK, wiec zadne ponowienie z tamtej strony nie nadejdzie.
    // Odkladamy ja do jednego gniazda i wysylamy z loop(), gdy slot sie zwolni.
    if (pendingForwardFrame.length() == 0) {
        pendingForwardFrame = frame;
        if (pendingForwardFrame.length() == frame.length()) { // kopia mogla pasc na OOM
            pendingForwardHop = nextHop;
            pendingForwardDeadline = millis() + 2500;
            Serial.println(F("MESH | forward odlozony (slot transakcji zajety)"));
            return true;
        }
        pendingForwardFrame = "";
    }
    Serial.println(F("MESH | forward porzucony - gniazdo odlozen zajete"));
    return false;
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
