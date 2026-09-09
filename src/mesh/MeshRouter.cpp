#include "MeshRouter.h"

MeshRouter *MeshRouter::instance = nullptr;

MeshRouter::MeshRouter(RadioManager *manager) {
    this->manager = manager;
    instance = this;
}

void MeshRouter::onDataReceived(MeshDataCallback callback) {
    dataReceivedCallback = callback;
}

void MeshRouter::setFrozen(bool value) {
    if (value && !frozen) {
        // Zamrozenie obejmuje takze skok w locie: bez tego timeout ACK transakcji
        // sprzed zamrozenia ponawialby ramki mesh w srodku transferu OTA.
        hopRetriesLeft = 0;
        hopRetryPending = false;
        pendingForwardLen = 0;
        appOkCallback = nullptr;
        appFailCallback = nullptr;
    }
    frozen = value;
}

void MeshRouter::loop() {
    if (frozen) return; // OTA: zadnych beaconow ani forwardingu, eter dla transferu
    ageTables();

    // Odlozone ponowienie skoku (timeout ACK trafil w zajete radio): ma
    // pierwszenstwo - trzyma slot transakcji, a payload wciaz lezy w buforze
    // nadawczym radia (resendRetained ponawia go bez zadnej kopii).
    // Porownania przez ODEJMOWANIE - millis() przekreca sie co ~49 dni, a proste
    // ">=" zostawiloby wtedy hopRetryPending na stale. To z kolei blokuje i wysylke,
    // i beacony, czyli wezel milknie w sieci az do resetu.
    if (hopRetryPending && (long) (millis() - hopRetryAtMillis) >= 0 && !manager->waitingForAck) {
        if ((long) (millis() - hopRetryDeadlineMillis) > 0 || !manager->hasRetainedFrame()) {
            hopRetryPending = false;
            giveUpHop();
        } else if (manager->resendRetained(hopDest, sHopAckOk, sHopAckFail)) {
            hopRetryPending = false;
            if (hopRetriesLeft > 0) hopRetriesLeft--; // proba zuzyta: poszla w eter
        } else {
            hopRetryAtMillis = millis() + 150 + (micros() & 0x7F);
        }
    }

    // Odlozony forward: poprzedni skok juz potwierdzil te ramke, wiec nikt jej
    // nie ponowi - odsylamy ja, gdy tylko slot transakcji sie zwolni.
    if (pendingForwardLen > 0 && !manager->waitingForAck && !hopRetryPending) {
        if ((long) (millis() - pendingForwardDeadline) > 0) {
            Serial.println(F("MESH | odlozony forward przeterminowany - porzucam"));
            pendingForwardLen = 0;
        } else {
            if (manager->sendBytes(pendingForward, pendingForwardLen, pendingForwardHop,
                                   RADIO_TYPE_MESH, true, sHopAckOk, sHopAckFail)) {
                pendingForwardLen = 0;
                hopRetriesLeft = MESH_HOP_RETRIES;
                hopDest = pendingForwardHop;
                appOkCallback = nullptr;   // forward nie jest nasza wysylka
                appFailCallback = nullptr;
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
            // Radio zajete: ponow szybko, zamiast czekac cala kadencje - przy gestym
            // ruchu gubilismy w ten sposob wiekszosc slotow beaconowych.
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
// do martwego celu az do nasycenia metryki (count-to-infinity).
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
    // Beacony PRZEMIENNE: co drugi na suficie (odkrywanie odleglych wezlow), co
    // drugi na biezacej mocy regulowanej. Sam sufit okazal sie pulapka: z malej
    // odleglosci odbiornik ulega saturacji i to wlasnie beacony gina pierwsze,
    // zabierajac cale trasy - dane na niskiej mocy przeszlyby bez problemu, ale
    // bez tras nikt ich nie wysyla i mesh staje na dobre (widziane na sprzecie).
    // Moc wpisana ponizej w tresc czyni pomiar tlumienia poprawnym przy kazdej mocy.
    int8_t beaconPower = (seqToSend & 1) ? manager->getEffectiveTxPower()
                                         : manager->apcMaxDbm;
    // Beacon skladany wprost w buforze nadawczym radia - zero alokacji.
    // Najwiekszy rozmiar: 4 + MESH_MAX_ROUTES * 3 = 22 B.
    uint8_t *beacon = manager->acquireTxBuffer();
    if (beacon == nullptr) return false;
    uint8_t n = 0;
    beacon[n++] = MESH_MSG_BEACON;
    beacon[n++] = (uint8_t) beaconPower; // moc, z jaka beacon FAKTYCZNIE poleci
    beacon[n++] = seqToSend;
    uint8_t countIndex = n++;
    uint8_t count = 0;
    for (auto &r : routes) {
        if (r.dest == 0) continue;
        beacon[n++] = r.dest;
        beacon[n++] = r.metric;
        beacon[n++] = r.seq;
        count++;
    }
    beacon[countIndex] = count;
    // Broadcast bez ACK, moc przemienna - patrz komentarz wyzej.
    bool queued = manager->commitTxBuffer(n, RADIO_BROADCAST_ID, RADIO_TYPE_MESH, false,
                                          nullptr, nullptr, beaconPower);
    if (queued) ownSeq = seqToSend; // seq rosnie tylko dla beaconow, ktore poszly
    return queued;
}

// body wskazuje ZA bajt rodzaju: [moc nadania int8][seq][liczba tras] + 3 B na trase.
void MeshRouter::handleBeacon(const uint8_t *body, uint8_t len, uint8_t radioSender) {
    if (radioSender == manager->nodeId || radioSender == 0) return; // nigdy sasiad z wlasnym id
    if (len < 3) return;
    int8_t beaconTxPower = (int8_t) body[0];
    uint8_t senderSeq = body[1];
    uint8_t count = body[2];
    // Ramka obcieta albo przeklamana: liczba tras nie moze wykraczac poza tresc.
    if ((uint16_t) 3 + (uint16_t) count * MESH_BEACON_ROUTE_LEN > len) return;

    // Tlumienie lacza do nadawcy: znamy moc nadania (z beaconu) i RSSI odbioru.
    // EMA 3/4 starej + 1/4 nowej probki - RSSI pojedynczej ramki skacze o kilka dB.
    int pathLoss = (int) beaconTxPower - manager->getLastRssi();
    // Minimum 1: zero znaczy "jeszcze nie mierzone" i przy dokladnie zerowym
    // tlumieniu srednia kroczaca nigdy by nie wystartowala.
    if (pathLoss < 1) pathLoss = 1;
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
        bool newer = (int8_t) (uint8_t) (senderSeq - r->seq) > 0;
        bool older = (int8_t) (uint8_t) (senderSeq - r->seq) < 0;
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
            r->seq = senderSeq;
        }
    }

    // Trasy ogloszone przez nadawce: DSDV - nowszy seq wygrywa zawsze, w ramach
    // tego samego seq obowiazuje histereza (ruch = wahania RSSI = ryzyko trzepotania).
    const uint8_t *entry = body + 3;
    for (uint8_t i = 0; i < count; i++, entry += MESH_BEACON_ROUTE_LEN) {
        uint8_t dest = entry[0];
        uint8_t metric = entry[1];
        uint8_t seq = entry[2];

        if (dest == manager->nodeId) {
            // Odzysk ciaglosci seq po restarcie (DSDV): jesli siec pamieta nas z
            // wyzszym numerem, przeskakujemy go - inaczej nasze swieze beacony
            // bylyby "starsze" od widma sprzed restartu nawet przez kilka minut.
            if ((int8_t) (uint8_t) (seq - ownSeq) > 0) ownSeq = seq;
            continue;
        }
        if (dest == radioSender || dest == 0 || dest > 250) continue;
        uint16_t total = (uint16_t) directCost + (metric >= MESH_METRIC_INFINITY
                                                  ? MESH_METRIC_INFINITY : (uint16_t) metric);
        uint8_t candidate = total >= MESH_METRIC_INFINITY ? MESH_METRIC_INFINITY : (uint8_t) total;
        Route *route = findRoute(dest, candidate < MESH_METRIC_INFINITY);
        if (route == nullptr) continue;
        bool newer = (int8_t) (uint8_t) (seq - route->seq) > 0;
        bool older = (int8_t) (uint8_t) (seq - route->seq) < 0;
        bool sameHop = route->nextHop == radioSender;
        if (newer || (!older && (sameHop || route->metric >= MESH_METRIC_INFINITY
            || candidate + MESH_ROUTE_SWITCH_MARGIN < route->metric))) {
            if (!sameHop && route->metric < MESH_METRIC_INFINITY
                && candidate < MESH_METRIC_INFINITY) {
                Serial.print(F("MESH | trasa do "));
                Serial.print(dest);
                Serial.print(F(": via "));
                Serial.print(radioSender);
                Serial.print(F(" (koszt "));
                Serial.print(candidate);
                Serial.println(')');
            }
            route->nextHop = radioSender;
            route->metric = candidate;
            route->seq = seq;
        }
    }
}

// ==================== DANE ====================

// "[2][zrodlo][cel][TTL][id]" + tresc - wspolne dla wlasnych wysylek i forwardu.
// Zwraca calkowita dlugosc zlozonej wiadomosci mesh.
uint8_t MeshRouter::composeDataFrame(uint8_t *out, uint8_t origin, uint8_t finalDest, uint8_t ttl,
                                     uint8_t flowId, const uint8_t *payload, uint8_t payloadLen) {
    out[0] = MESH_MSG_DATA;
    out[1] = origin;
    out[2] = finalDest;
    out[3] = ttl;
    out[4] = flowId;
    memcpy(out + MESH_DATA_HEADER, payload, payloadLen);
    return (uint8_t) (MESH_DATA_HEADER + payloadLen);
}

bool MeshRouter::send(uint8_t finalDest, const uint8_t *payload, uint8_t len,
                      void (*okCallback)(), RadioFailCallback failCallback) {
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
    if (!RadioManager::txBufferFits((uint16_t) MESH_DATA_HEADER + len)) {
        Serial.println(F("MESH | tresc za dluga - nie wyslano"));
        return false;
    }
    // Ramka skladana WPROST w buforze nadawczym radia: zero alokacji i zero kopii
    // ponad te jedna, ktora i tak trzeba zrobic.
    uint8_t *frame = manager->acquireTxBuffer();
    if (frame == nullptr) return false;
    uint8_t total = composeDataFrame(frame, manager->nodeId, finalDest, MESH_MAX_TTL,
                                     nextFlowId, payload, len);
    if (!manager->commitTxBuffer(total, nextHop, RADIO_TYPE_MESH, true,
                                 sHopAckOk, sHopAckFail)) {
        return false;
    }
    // Kontekst skoku dopiero po udanym zakolejkowaniu - inaczej nieudana proba
    // zostawiala liczniki i callbacki poprzedniej transakcji nadpisane.
    hopRetriesLeft = MESH_HOP_RETRIES;
    hopDest = nextHop;
    // ACK skok po skoku: "OK" u aplikacji = dotarlo do PIERWSZEGO posrednika.
    appOkCallback = okCallback;
    appFailCallback = failCallback;
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

void MeshRouter::sHopAckFail(const uint8_t *payload, uint8_t len) {
    (void) payload;
    (void) len;
    if (instance != nullptr) instance->hopAckFail();
}

// ACK skoku nie doszedl. Ponawiamy ograniczona liczbe razy, a potem uniewazniamy
// wszystkie trasy przez tego sasiada - wezly sa w ruchu, wiec brak ACK to zwykle
// "odjechal", a nastepne beacony i tak przyniosa swieza topologie.
void MeshRouter::hopAckFail() {
    if (frozen) return; // OTA: transakcja sprzed zamrozenia wygasa bez ponowien i kar
    if (hopRetriesLeft > 0 && manager->hasRetainedFrame()) {
        // NIGDY nie ponawiamy natychmiast. Timeout ACK jest deterministyczny (1 s od
        // nadania), wiec dwa wezly, ktorych ramki raz sie zderzyly, ponawialy je w tej
        // samej milisekundzie i zderzaly ponownie - az do wyczerpania prob, utraty tras
        // i sondowania mocy (ktore kolizji nie leczy). Czarna skrzynka pokazala
        // "nie potwierdza" na OBU wezlach w identycznych znacznikach czasu. Losowy
        // odstep 100-611 ms (ALOHA) rozrywa te synchronizacje; proba jest zuzywana
        // dopiero, gdy ponowienie naprawde pojdzie w eter (obsluga w loop()).
        Serial.println(F("MESH | skok bez ACK - ponawiam po losowym odstepie"));
        if (!hopRetryPending) hopRetryDeadlineMillis = millis() + 2500;
        hopRetryPending = true;
        hopRetryAtMillis = millis() + 100 + (micros() & 0x1FF);
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
        // Aplikacji oddajemy JEJ tresc, bez naglowka routingu - inaczej w logu
        // ladowalo piec bajtow binarnych przed wiadomoscia. Gdy bufor zdazyl juz
        // zostac zajety przez cos innego, nie ma czego pokazac.
        uint8_t kept = manager->retainedLength();
        if (kept >= MESH_DATA_HEADER) {
            appFailCallback(manager->retainedFrame() + MESH_DATA_HEADER,
                            (uint8_t) (kept - MESH_DATA_HEADER));
        } else {
            appFailCallback(nullptr, 0);
        }
        appFailCallback = nullptr;
    }
    appOkCallback = nullptr;
}

// payload = tresc ramki radiowej typu MESH (radio zdjelo juz swoj naglowek).
void MeshRouter::radioMeshDataReceived(uint8_t *payload, uint8_t len, uint8_t radioSender) {
    if (len < 1) return;
    if (payload[0] == MESH_MSG_BEACON) {
        handleBeacon(payload + 1, (uint8_t) (len - 1), radioSender);
    } else if (payload[0] == MESH_MSG_DATA) {
        handleData(payload + 1, (uint8_t) (len - 1), radioSender);
    }
}

// body wskazuje ZA bajt rodzaju: [zrodlo][cel koncowy][TTL][id strumienia] + tresc
void MeshRouter::handleData(uint8_t *body, uint8_t len, uint8_t radioSender) {
    (void) radioSender;
    if (len < MESH_DATA_HEADER - 1) return;
    uint8_t origin = body[0];
    uint8_t finalDest = body[1];
    uint8_t ttl = body[2];
    uint8_t flowId = body[3];
    uint8_t *payload = body + (MESH_DATA_HEADER - 1);
    uint8_t payloadLen = (uint8_t) (len - (MESH_DATA_HEADER - 1));

    if (origin == 0 || origin > 250 || finalDest == 0 || finalDest > 250) return;
    if (isDuplicate(origin, flowId)) return;

    if (finalDest == manager->nodeId) {
        // Dostarczenie: tresc lezy w buforze odbiorczym radia, ktory jest zakonczony
        // zerem - warstwa wyzej dostaje ja jako gotowy C-string, bez zadnej kopii.
        if (dataReceivedCallback) dataReceivedCallback((const char *) payload, payloadLen, origin);
        return;
    }
    if (frozen) return;      // OTA: nie forwardujemy cudzych ramek
    if (ttl <= 1) {
        Serial.println(F("MESH | TTL wyczerpany - porzucam ramke"));
        return;
    }
    forwardData(origin, finalDest, (uint8_t) (ttl - 1), flowId, payload, payloadLen);
}

bool MeshRouter::forwardData(uint8_t origin, uint8_t finalDest, uint8_t ttl,
                             uint8_t flowId, const uint8_t *payload, uint8_t payloadLen) {
    uint8_t nextHop = getNextHop(finalDest);
    if (nextHop == 0) {
        Serial.print(F("MESH | forward: brak trasy do "));
        Serial.println(finalDest);
        return false;
    }
    uint16_t frameLen = (uint16_t) MESH_DATA_HEADER + payloadLen;
    if (!RadioManager::txBufferFits(frameLen)) {
        Serial.println(F("MESH | forward: tresc za dluga - porzucam"));
        return false;
    }
    if (!manager->waitingForAck && !hopRetryPending) {
        // Slot wolny: ramka skladana wprost w buforze nadawczym radia (bez kopii).
        uint8_t *frame = manager->acquireTxBuffer();
        if (frame != nullptr) {
            uint8_t total = composeDataFrame(frame, origin, finalDest, ttl, flowId,
                                             payload, payloadLen);
            if (manager->commitTxBuffer(total, nextHop, RADIO_TYPE_MESH, true,
                                        sHopAckOk, sHopAckFail)) {
                hopRetriesLeft = MESH_HOP_RETRIES;
                hopDest = nextHop;
                appOkCallback = nullptr;   // forward nie jest nasza aplikacyjna wysylka
                appFailCallback = nullptr;
                Serial.print(F("MESH | forward "));
                Serial.print(origin);
                Serial.print(F("->"));
                Serial.print(finalDest);
                Serial.print(F(" via "));
                Serial.println(nextHop);
                return true;
            }
        }
    }
    // Slot transakcji zajety. Ramki NIE wolno porzucic: poprzedni skok juz dostal
    // jej radiowe ACK, wiec zadne ponowienie z tamtej strony nie nadejdzie.
    // Odkladamy ja do jednego gniazda i wysylamy z loop(), gdy slot sie zwolni.
    if (pendingForwardLen == 0) {
        pendingForwardLen = composeDataFrame(pendingForward, origin, finalDest, ttl, flowId,
                                             payload, payloadLen);
        pendingForwardHop = nextHop;
        pendingForwardDeadline = millis() + 2500;
        Serial.println(F("MESH | forward odlozony (slot transakcji zajety)"));
        return true;
    }
    Serial.println(F("MESH | forward porzucony - gniazdo odlozen zajete"));
    return false;
}

// "DIAG mesh nb=<id>/<tlumienie>/<wiek s> ... rt=<cel>>via<skok>:<koszt>:<seq> ... flagi"
void MeshRouter::printState() {
    Serial.print(F("DIAG mesh nb="));
    for (auto &n : neighbors) {
        if (n.id == 0) continue;
        Serial.print(n.id); Serial.print('/'); Serial.print(n.pathLossDb); Serial.print('/');
        Serial.print((millis() - n.lastHeardMillis) / 1000); Serial.print(' ');
    }
    Serial.print(F("rt="));
    for (auto &r : routes) {
        if (r.dest == 0) continue;
        Serial.print(r.dest); Serial.print('>'); Serial.print(r.nextHop); Serial.print(':');
        Serial.print(r.metric); Serial.print(':'); Serial.print(r.seq); Serial.print(' ');
    }
    Serial.print(F("seq=")); Serial.print(ownSeq);
    Serial.print(F(" retry=")); Serial.print(hopRetryPending);
    Serial.print(F(" fwd=")); Serial.print(pendingForwardLen);
    Serial.print(F(" frozen=")); Serial.println(frozen);
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
