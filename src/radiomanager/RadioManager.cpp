//
// Created by LukaszLibront on 19.12.2023.
//

#include "RadioManager.h"


RadioManager::RadioManager() {
    // Bufory sa tablicami skladowymi - nie ma tu nic do zarezerwowania. Konstruktor
    // zostaje, bo warstwy wyzej trzymaja wskaznik na obiekt tworzony przez new.
}

void RadioManager::setupRadio(long frequency, int ss, int reset, int dio0, uint8_t _nodeId, void(*receiveDoneCallback)(int), void(*txDoneCallback)()) {
    nodeId = _nodeId;
    LoRa.setPins(ss, reset, dio0);
    // Po samoczynnym reboocie (brown-out w trakcie nadawania) modul potrafi nie
    // odpowiedziec przy pierwszym podejsciu. Ciche while(true) zamienialo to w
    // wieczna cegle - teraz probujemy do skutku i mowimy o tym glosno.
    while (!LoRa.begin(frequency)) {
        Serial.println(F("[RADIO] LoRa.begin nie odpowiada - ponawiam za 500 ms"));
        delay(500);
    }
    // enableCrc() musi byc PO begin() - begin() resetuje modul i czysci ten bit
    LoRa.enableCrc();
    LoRa.onReceive(receiveDoneCallback);
    LoRa.onTxDone(txDoneCallback);

    DEBUGlogln(F("LoRa init succeeded."));

    LoRa_rxMode();
}

void RadioManager::onDataReceived(RadioTextCallback callback) {
    dataReceivedCallback = callback;
}

void RadioManager::onOtaDataReceived(RadioTextCallback callback) {
    otaDataReceivedCallback = callback;
}

void RadioManager::onMeshDataReceived(RadioBytesCallback callback) {
    meshDataReceivedCallback = callback;
}

void RadioManager::onAnyFrameReceived(void (*callback)(uint8_t senderId)) {
    anyFrameReceivedCallback = callback;
}

void RadioManager::onDataSent(void(*callback)()) {
    dataSentCallback = callback;
}

int RadioManager::getReceivedPacketSize() {
    return receivedPacketSize;
}

bool RadioManager::isAckReceived() {
    return ackReceived;
}

uint8_t RadioManager::getSenderIdOfLastMessage() {
    return senderIdOfLastMessage;
}

int RadioManager::getLastRssi() {
    return lastRssi;
}

bool RadioManager::isTransmissionFinished() {
    return transmissionFinished;
}

bool RadioManager::isNeedToSendAckToSender() {
    return needToSendAckToSender;
}

bool RadioManager::isDataSent() {
    return transmissionFinished;
}

void RadioManager::setSendAckAutomaticly(bool value) {
    sendAckAutomaticly = value;
}

void RadioManager::loop() {
    // receiveLoop PRZED sendLoop: odebrana, jeszcze nieodczytana ramka lezy w FIFO
    // radia, a beginPacket() przy nadawaniu zeruje wskaznik FIFO i nadpisuje ja
    // wlasnym payloadem. Wezel czytal wtedy WLASNA ramke jako odebrana (czarna
    // skrzynka: klient z sasiadem o wlasnym id), a prawdziwa ramka przepadala.
    receiveLoop();
    sendLoop();
    waitForAckTimeoutLoop();
    txStuckWatchdogLoop();
}

// Awaryjne odblokowanie nadajnika: gdyby przerwanie TxDone przepadlo (wyscig w
// bibliotece przy async endPacket), transmissionFinished nigdy nie wroci na true,
// bufor nie zostalby oprozniony i kazda kolejna wysylka bylaby odrzucana
// w nieskonczonosc. Po timeoucie wymuszamy powrot do RX i domkniecie cyklu.
void RadioManager::txStuckWatchdogLoop() {
    if (!transmissionFinished && millis() - txStartMillis > txStuckTimeout) {
        DEBUGlogln(F("[RFM96] TX stuck - forcing RX mode"));
        LoRa_rxMode();
        txDoneTime = micros();
        transmissionFinished = true;
    }
}

void RadioManager::sendLoop() {
    if (!transmissionFinished) return;
    if (!transmissionClenedUp) {
        transmissionClenedUp = true;
        DEBUGlogln(F("transmission finished!"));
        if (dataSentCallback) {
            dataSentCallback();
        }
        return;
    }
    // Potwierdzenia maja pierwszenstwo: druga strona odlicza swoj timeout ACK,
    // a nasza wlasna ramka moze poczekac jeden obieg petli.
    if (ackPending) {
        DEBUGlogln(F("[RFM96] Sending ACK packet ... "));
        frameTxPwrOverride = -1; // ACK-i zawsze moca regulowana
        if (startSending(ackPayload, RADIO_ACK_PAYLOAD_SIZE, ackSendBufferDest,
                         RADIO_TYPE_ACK, false)) {
            ackPending = false;
        }
        return;
    }
    if (txPending) {
        DEBUGlogln(F("[RFM96] Sending normal packet ... "));
        // Bufor zostaje przy niepowodzeniu - kolejny obieg petli sprobuje ponownie,
        // zamiast po cichu gubic pakiet (razem z ewentualnym wymuszeniem mocy).
        frameTxPwrOverride = sendBufferTxPwrOverride;
        if (startSending(txPayload, txLength, sendBufferDest, txType, sendBufferAckReq)) {
            // Tresc NIE jest kasowana: ramka z zadaniem ACK zostaje jako payload
            // ewentualnego ponowienia (resendRetained) i callbacku bledu.
            txPending = false;
            retainedFrameValid = sendBufferAckReq;
            sendBufferTxPwrOverride = -1;
        }
        frameTxPwrOverride = -1;
    }
}

// Odczyt ramki z FIFO do rxPayload. false = ramka nie jest dla nas (obcy adresat,
// wlasne echo, smiec albo stary format tekstowy) i nie wolno jej dalej obrabiac.
bool RadioManager::readReceivedFrame() {
    // RSSI odczytany zanim cokolwiek innego zdazy sie wydarzyc - rejestr PktRssiValue
    // nadpisuje dopiero kolejny odebrany pakiet. Kontekst petli glownej (nie ISR),
    // wiec dostep po SPI jest bezpieczny.
    lastRssi = LoRa.packetRssi();
    int size = receivedPacketSize;
    rxLength = 0;
    if (size < RADIO_HEADER_SIZE || size > RADIO_HEADER_SIZE + RADIO_PAYLOAD_CAPACITY) {
        while (LoRa.available()) LoRa.read();
        DEBUGlogln(F("RadioManager | ramka o zlej dlugosci - odrzucam"));
        return false;
    }
    uint8_t header[RADIO_HEADER_SIZE];
    for (uint8_t i = 0; i < RADIO_HEADER_SIZE; i++) header[i] = (uint8_t) LoRa.read();
    uint8_t len = (uint8_t) (size - RADIO_HEADER_SIZE);
    for (uint8_t i = 0; i < len; i++) rxPayload[i] = (uint8_t) LoRa.read();
    rxPayload[len] = 0; // tresci tekstowe ida w gore jako gotowy C-string
    while (LoRa.available()) LoRa.read();

    // Znacznik protokolu odsiewa smiec i ramki starego, tekstowego firmware
    // (tam bajt 3 to cyfra albo malpa) zanim zinterpretujemy cokolwiek innego.
    if ((header[3] & RADIO_PROTO_MASK) != RADIO_PROTO_MARK) {
        DEBUGlogln(F("RadioManager | obcy format ramki - odrzucam"));
        return false;
    }
    destinationIdOfLastMessage = header[0];
    senderIdOfLastMessage = header[1];
    receivedMessageIdOfLastMessage = header[2];
    lastFrameType = header[3] & RADIO_TYPE_MASK;
    rxLength = len;

    if (destinationIdOfLastMessage != nodeId
        && destinationIdOfLastMessage != RADIO_BROADCAST_ID) {
        DEBUGlogln(F("This is not a destination address, ignoring message"));
        return false;
    }
    // Ramka "od nas samych" nie ma prawa istniec: to echo wlasnego payloadu
    // odczytane z FIFO po wyscigu z nadawaniem albo przeklamany naglowek.
    if (senderIdOfLastMessage == nodeId || senderIdOfLastMessage == 0) {
        Serial.println(F("RadioManager | ramka z wlasnym id nadawcy - odrzucam"));
        return false;
    }
    // Broadcast nikt nie kwituje - nie ma jednego adresata, ktory mialby to zrobic.
    needToSendAckToSender = (header[3] & RADIO_FLAG_ACK_REQ) != 0
                            && destinationIdOfLastMessage == nodeId;
    return true;
}

void RadioManager::receiveLoop() {
    if (zeroLengthPacketReceived) {
        zeroLengthPacketReceived = false;
        DEBUGlogln(F("ERROR: Received 0 lenght packet!!!"));
    }
    if (!receivedFlag) return;
    receivedFlag = false;
    if (!readReceivedFrame()) return;

    // KAZDA poprawnie zaadresowana ramka (dane, ACK, beacon) jest dowodem, ze
    // lacze od nadawcy zyje - mesh odswieza tym swoich sasiadow, zeby zgubione
    // beacony (broadcast bez ACK, gina w kolizjach) nie usmiercaly zywych tras.
    if (anyFrameReceivedCallback) {
        anyFrameReceivedCallback(senderIdOfLastMessage);
    }

    // Typ ramki zdjety z naglowka w readReceivedFrame - dalej decyduje o tym,
    // ktora warstwa dostanie tresc.
    uint8_t frameType = lastFrameType;

    if (frameType == RADIO_TYPE_ACK) {
        // Potwierdzenie: [id kwitowanej ramki][RSSI, z jakim ja uslyszano].
        if (rxLength >= RADIO_ACK_PAYLOAD_SIZE && waitingForAck
            && pendingAckMessageId != 0 && rxPayload[0] == pendingAckMessageId) {
            DEBUGlogln(F("Received ACK"));
            ackReceived = true;
            waitingForAck = false;
            pendingAckMessageId = 0;
            apcOnAck((int8_t) rxPayload[1]); // zwrotka RSSI -> krok regulatora mocy
            if (ackReceivedCallback) {
                ackReceivedCallback();
            }
        } else {
            // spozniony/niedopasowany ACK - nie publikuj go jako danych
            DEBUGlogln(F("RadioManager | stale/unmatched ACK - ignoring"));
        }
        return;
    }

    if (needToSendAckToSender && sendAckAutomaticly) {
        sendAck();
    }

    if (frameType == RADIO_TYPE_OTA) {
        DEBUGlogln(F("Received OTA message"));
        if (otaDataReceivedCallback) {
            otaDataReceivedCallback((char *) rxPayload, rxLength, senderIdOfLastMessage);
        }
    } else if (frameType == RADIO_TYPE_MESH) {
        if (meshDataReceivedCallback) {
            meshDataReceivedCallback(rxPayload, rxLength, senderIdOfLastMessage);
        }
    } else if (frameType == RADIO_TYPE_APP) {
        DEBUGlogln(F("Received DATA message"));
        if (dataReceivedCallback) {
            dataReceivedCallback((char *) rxPayload, rxLength, senderIdOfLastMessage);
        }
    }
}

void RadioManager::sendAck() {
    DEBUGlog(F("Sending ACK to address: "));
    DEBUGlogln(senderIdOfLastMessage);
    // Dwa bajty w osobnym, malym buforze: id kwitowanej ramki i RSSI, z jakim ja
    // uslyszelismy (zwrotka dla regulatora mocy drugiej strony). Osobny bufor,
    // bo potwierdzenie musi moc wyjsc takze wtedy, gdy nasza wlasna ramka czeka
    // w buforze nadawczym na swoje ACK.
    ackPayload[0] = receivedMessageIdOfLastMessage;
    int rssi = lastRssi;
    if (rssi < -128) rssi = -128;
    if (rssi > 127) rssi = 127;
    ackPayload[1] = (uint8_t) (int8_t) rssi;
    ackSendBufferDest = senderIdOfLastMessage;
    ackPending = true;
}

void RadioManager::waitForAckTimeoutLoop() {
    // ackFramePendingTx: nie odliczaj timeoutu, dopoki ramka nie zostala nadana
    if (waitingForAck && !ackReceived && !ackFramePendingTx) {
        if (millis() - waitForAckStartTime >= ackTimeout) {
            waitingForAck = false;
            pendingAckMessageId = 0;
            DEBUGlogln(F("ACK NOT RECEIVED - TIMEOUT"));
            apcOnAckTimeout();
            if (ackNotReceivedCallback) {
                ackNotReceivedCallback(txPayload, txLength); // nadana ramka wciaz lezy w buforze
            }
        }
    }
    // Bezpiecznik ostateczny: zadna kombinacja flag nie ma prawa trzymac waitingForAck
    // dluzej niz 3x timeout (ramka w buforze nadaje sie najpozniej po 2 s dzieki
    // txStuckWatchdog). Widziane na sprzecie jako trwale zakleszczenie po zbiegu
    // restartow obu wezlow: wysylki wiecznie "pominiete", timeout nigdy nie strzelal.
    if (waitingForAck && millis() - waitForAckStartTime >= ackTimeout * 3) {
        Serial.println(F("RadioManager | ACK watchdog: zwalniam zakleszczone flagi"));
        // Jesli uzbrojona ramka wciaz tkwi w buforze, zdejmij jej zadanie ACK -
        // wyjdzie w eter jako zwykla ramka, zamiast nadac sie PO zgloszonym bledzie
        // i sciagnac spozniony ACK bez pary.
        if (ackFramePendingTx && txPending) sendBufferAckReq = false;
        waitingForAck = false;
        ackFramePendingTx = false;
        pendingAckMessageId = 0;
        apcOnAckTimeout(); // strata jak kazda inna - APC ma ja widziec
        if (ackNotReceivedCallback) {
            ackNotReceivedCallback(txPayload, txLength);
        }
    }
}

// ==================== AUTOMATYCZNA REGULACJA MOCY (APC) ====================

// Zadania zmiany mocy sa odkladane do apcPendingDbm i aplikowane w startSending,
// tuz przed nadaniem kolejnej ramki. Bezposredni zapis rejestrow PA w trakcie
// trwajacej transmisji (transmissionFinished == false, np. ACK odebrany, gdy
// sendLoop juz nadaje kolejna wiadomosc z bufora) zmienilby moc W SRODKU ramki.
void RadioManager::apcRequestPower(int8_t dbm) {
    if (dbm > apcMaxDbm) dbm = apcMaxDbm;
    if (dbm < TX_POWER_MIN_DBM) dbm = TX_POWER_MIN_DBM;
    apcPendingDbm = (dbm == txPowerDbm) ? -1 : dbm; // ostatnie zadanie wygrywa
}

void RadioManager::setApcMaxPower(int8_t dbm) {
    if (dbm > TX_POWER_MAX_DBM) dbm = TX_POWER_MAX_DBM;
    if (dbm < TX_POWER_MIN_DBM) dbm = TX_POWER_MIN_DBM;
    apcMaxDbm = dbm;
}

// Moc, z jaka wyjdzie NASTEPNA ramka: zmiana zadana przez APC moze jeszcze czekac
// w apcPendingDbm na przerwe miedzy ramkami - znacznik [P] w tresci wiadomosci musi
// opisywac moc FAKTYCZNEGO nadania, nie stan sprzed zmiany.
int8_t RadioManager::getEffectiveTxPower() {
    return (apcPendingDbm >= 0) ? apcPendingDbm : txPowerDbm;
}

void RadioManager::apcOnAck(int8_t reportedRssi) {
    if (!APC_ENABLED) return;
    ackMissStreak = 0;
    // RSSI >= 0 dBm jest fizycznie niemozliwe dla LoRa: to smiec albo zwrotka
    // z uszkodzonej ramki. Regulowanie wedlug takiej wartosci sciagaloby moc W DOL
    // dokladnie wtedy, gdy lacze jest w najgorszym stanie.
    if (reportedRssi >= 0) return;
    peerReportedRssi = reportedRssi;
    peerRssiValid = true;
    if (apcFrozen) return;
    // Jeden krok na jedna zwrotke - tempo regulacji ogranicza naturalnie rytm
    // wymian ACK, bez dodatkowego timera. Histereza tlumi skoki RSSI miedzy ramkami.
    int8_t current = (apcPendingDbm >= 0) ? apcPendingDbm : txPowerDbm;
    if (peerReportedRssi > APC_TARGET_RSSI_DBM + APC_HYSTERESIS_DB) {
        apcRequestPower(current - APC_STEP_DB); // slychac nas az za dobrze - oszczedzaj
    } else if (peerReportedRssi < APC_TARGET_RSSI_DBM - APC_HYSTERESIS_DB) {
        apcRequestPower(current + APC_STEP_DB); // za cicho - doloz
    }
}

void RadioManager::apcOnAckTimeout() {
    if (!APC_ENABLED || apcFrozen) return;
    if (ackMissStreak < 255) ackMissStreak++;
    // Seria strat = sonda mocy: kroki W GORE od mocy biezacej, po przekroczeniu
    // sufitu zawiniecie do minimum. Poprzednia polityka (skok od razu na sufit)
    // byla pulapka: z bliska sufit saturuje odbiornik, a nocny zapis pokazal 545
    // takich epizodow, kazdy konczony dopiero powolnym zejsciem sondy w dol.
    if (ackMissStreak >= APC_ACK_MISS_LIMIT
        && ackMissStreak % APC_ACK_MISS_LIMIT == 0) {
        int8_t current = getEffectiveTxPower();
        int8_t next = current + 2 * APC_STEP_DB;
        if (next > apcMaxDbm) {
            next = (current >= apcMaxDbm) ? TX_POWER_MIN_DBM : apcMaxDbm;
        }
        Serial.print(F("RadioManager | APC: brak ACK - sonduje moc "));
        Serial.print(next);
        Serial.println(F(" dBm"));
        apcRequestPower(next);
    }
}

void RadioManager::setApcFrozen(bool frozen) {
    if (!APC_ENABLED || frozen == apcFrozen) return;
    apcFrozen = frozen;
    if (frozen) {
        // Transfer OTA: niezawodnosc wazniejsza niz oszczedzanie - przypnij sufit.
        apcRequestPower(apcMaxDbm);
        ackMissStreak = 0;
    }
}

// Wolny RAM = odleglosc miedzy szczytem sterty a wierzcholkiem stosu (chwila obecna).
int RadioManager::freeRam() {
    extern int __heap_start, *__brkval;
    int stackTop;
    return (int) &stackTop - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

// Wzor malowania - dowolny bajt, ktory rzadko wystepuje w danych i adresach.
#define STACK_PAINT_BYTE 0xA5

// Maluje wolny obszar od szczytu sterty do tuz pod biezacy SP. Przerwanie w trakcie
// petli nie szkodzi: jego ramka zyje ponizej SP tylko, gdy MY jestesmy wstrzymani,
// a po powrocie jest martwa - najwyzej zostanie policzona jako "uzyty stos" (prawda).
void RadioManager::paintFreeStack() {
    extern int __heap_start, *__brkval; // te same deklaracje co w freeRam (LTO pilnuje typow)
    uint8_t *p = (uint8_t *) ((__brkval == 0) ? &__heap_start : __brkval);
    uint8_t *top = (uint8_t *) SP - 8;
    while (p < top) *p++ = STACK_PAINT_BYTE;
}

// Najdluzszy nietkniety pas wzoru miedzy szczytem sterty a SP = najmniejszy zapas
// miedzy sterta a stosem, jaki wystapil od startu (razem z przerwaniami). To ta
// liczba, a nie chwilowe "ram=", mowi, czy zapas pamieci naprawde jest bezpieczny.
uint16_t RadioManager::minStackGap() {
    extern int __heap_start, *__brkval;
    uint8_t *p = (uint8_t *) ((__brkval == 0) ? &__heap_start : __brkval);
    uint8_t *top = (uint8_t *) SP;
    uint16_t best = 0, run = 0;
    for (; p < top; p++) {
        if (*p == STACK_PAINT_BYTE) {
            if (++run > best) best = run;
        } else {
            run = 0;
        }
    }
    return best;
}

// ==================== SKLADANIE I NADAWANIE RAMEK ====================
//
// Tresc jest skladana wprost w txPayload (tablica skladowa o stalym rozmiarze),
// a naglowek dopisywany dopiero w startSending, prosto do FIFO radia. Na tej
// sciezce nie ma ani jednej alokacji i ani jednej kopii wiecej, niz trzeba.

uint8_t *RadioManager::acquireTxBuffer() {
    // Jedna transakcja naraz: ramka czekajaca na nadanie ALBO trwajaca transakcja ACK
    // blokuje bufor takze dla ramek bez ACK - inaczej beacon nadpisalby tresc, ktora
    // moze byc jeszcze potrzebna do ponowienia.
    if (txPending) {
        DEBUGlogln(F("RadioManager | busy: ramka czeka na nadanie"));
        return nullptr;
    }
    if (waitingForAck) {
        DEBUGlogln(F("RadioManager | busy: waiting for ACK"));
        return nullptr;
    }
    retainedFrameValid = false; // to, co lezalo w buforze, wlasnie przepada
    txLength = 0;
    return txPayload;
}

void RadioManager::releaseTxBuffer() {
    txLength = 0;
    retainedFrameValid = false;
}

bool RadioManager::commitTxBuffer(uint8_t len, uint8_t address, uint8_t type, bool ackRequested,
                                  void (*_ackReceivedCallback)(),
                                  RadioFailCallback _ackNotReceivedCallback,
                                  int8_t txPowerDbmOverride) {
    // Pusta ramka nigdy nie moze uzbroic transakcji: sendLoop nadaje tylko niepuste
    // bufory, wiec ksiegowosc ACK dla pustej ramki wisialaby az do watchdoga.
    if (len == 0 || len > RADIO_PAYLOAD_CAPACITY) {
        Serial.print(F("RadioManager | ERROR: zla dlugosc ramki ("));
        Serial.print(len);
        Serial.println(F(" B) - nie wyslano"));
        releaseTxBuffer();
        return false;
    }
    txLength = len;
    txType = type;
    sendBufferDest = address;
    sendBufferAckReq = ackRequested;
    sendBufferTxPwrOverride = txPowerDbmOverride;
    if (ackRequested) {
        ackReceivedCallback = _ackReceivedCallback;
        ackNotReceivedCallback = _ackNotReceivedCallback;
        waitingForAck = true;
        ackReceived = false;
        waitForAckStartTime = millis(); // re-stemplowane w startSending przy faktycznym nadaniu
        ackFramePendingTx = true;
    }
    if (messageId == 0) messageId = 1;
    // Nadanie CELOWO odlozone do nastepnego obiegu petli (manager->loop() i tak
    // wola sendLoop co obieg) - wolajacy moze jeszcze pracowac na swoich danych.
    txPending = true;
    return true;
}

bool RadioManager::hasRetainedFrame() {
    return retainedFrameValid && !txPending && !waitingForAck && txLength > 0;
}

// Ponowienie ostatniej nadanej ramki z zadaniem ACK - tresc nadal lezy w buforze,
// wiec nie ma zadnej kopii. false = ramki juz nie ma albo radio zajete.
bool RadioManager::resendRetained(uint8_t address, void (*_ackReceivedCallback)(),
                                  RadioFailCallback _ackNotReceivedCallback) {
    if (!hasRetainedFrame()) return false;
    return commitTxBuffer(txLength, address, txType, true,
                          _ackReceivedCallback, _ackNotReceivedCallback);
}

bool RadioManager::sendBytes(const uint8_t *data, uint8_t len, uint8_t address, uint8_t type,
                             bool ackRequested, void (*_ackReceivedCallback)(),
                             RadioFailCallback _ackNotReceivedCallback) {
    uint8_t *frame = acquireTxBuffer();
    if (frame == nullptr) return false;
    if (!txBufferFits(len) || len == 0) {
        Serial.print(F("RadioManager | ERROR: tresc za dluga ("));
        Serial.print(len);
        Serial.println(F(" B) - nie wyslano"));
        releaseTxBuffer();
        return false;
    }
    memcpy(frame, data, len);
    return commitTxBuffer(len, address, type, ackRequested,
                          _ackReceivedCallback, _ackNotReceivedCallback);
}

bool RadioManager::sendText(const char *text, uint8_t address, uint8_t type,
                            void (*_ackReceivedCallback)(),
                            RadioFailCallback _ackNotReceivedCallback) {
    // Zero konczace NIE leci w eter - dlugosc niesie naglowek LoRa, a odbiorca
    // dopisuje zero sobie, w swoim buforze.
    size_t len = strlen(text);
    return sendBytes((const uint8_t *) text, (uint8_t) len, address, type,
                     _ackReceivedCallback != nullptr || _ackNotReceivedCallback != nullptr,
                     _ackReceivedCallback, _ackNotReceivedCallback);
}

bool RadioManager::sendOta(const char *text, uint8_t address,
                           void (*_ackReceivedCallback)(),
                           RadioFailCallback _ackNotReceivedCallback) {
    return sendText(text, address, RADIO_TYPE_OTA, _ackReceivedCallback, _ackNotReceivedCallback);
}

bool RadioManager::startSending(const uint8_t *payload, uint8_t len, uint8_t address,
                                uint8_t type, bool ackRequested) {
    // Najpierw warunki odroczenia BEZ zadnych efektow ubocznych - wczesniej odroczony
    // beacon aplikowal i cofal moc w kazdym obiegu petli (70 zapisow SPI i 70 linii
    // "APC: moc" na jeden 400-ms nasluch kanalu).
    // (a) Nieodczytana ramka w FIFO: beginPacket() by ja zniszczyl - czekamy obieg.
    if (receivedFlag) return false;
    // (b) Nasluch przed nadaniem (CSMA): przy kadencji 1 s i ramkach 150-200 ms dwa
    // wezly zderzaly sie co kilka sekund. Zajety kanal = odkladamy ramke (bufor
    // zostaje), ale nie dluzej niz CS_MAX_WAIT_MS, zeby halas nie zaglodzil nadajnika.
    if (LoRa.rssi() > CS_BUSY_RSSI_DBM) {
        if (csBusySinceMillis == 0) csBusySinceMillis = millis();
        if (millis() - csBusySinceMillis < CS_MAX_WAIT_MS) return false;
    }
    csBusySinceMillis = 0;
    // Odlozona zmiana mocy APC - tu transmissionFinished jest na pewno true
    // (gwarantuje to sendLoop), wiec zapis rejestrow PA nie trafi w trwajaca ramke.
    if (apcPendingDbm >= 0) {
        setTxPower(apcPendingDbm);
        apcPendingDbm = -1;
        Serial.print(F("RadioManager | APC: moc "));
        Serial.print(txPowerDbm);
        Serial.println(F(" dBm"));
    }
    // Wymuszona moc pojedynczej ramki (beacon mesh na suficie): ustaw na te ramke,
    // a powrot do mocy regulowanej odloz do nastepnej - chyba ze APC zdazy o cos
    // poprosic, wtedy jego zadanie ma pierwszenstwo.
    if (frameTxPwrOverride >= 0 && frameTxPwrOverride != txPowerDbm) {
        int8_t restore = txPowerDbm;
        setTxPower(frameTxPwrOverride);
        if (apcPendingDbm < 0) apcPendingDbm = restore;
    }
    LoRa_txMode();
    // beginPacket zwraca 0, gdy radio "wciaz nadaje" (tryb TX/CAD) - wtedy nie
    // resetuje FIFO ani dlugosci payloadu, a my nadalibysmy smieci. Zamiast tego
    // wymuszamy standby i probujemy raz jeszcze; po drugiej odmowie ramka przepada
    // (ksiegowosc ACK i tak ruszy i zglosi timeout), ale radio wraca do nasluchu,
    // zamiast czekac 2 s na watchdog TX.
    bool fifoReady = LoRa.beginPacket();
    if (!fifoReady) {
        Serial.println(F("RadioManager | beginPacket odrzucony - wymuszam standby"));
        LoRa.idle();
        fifoReady = LoRa.beginPacket();
    }
    messageId++;
    if (messageId == 0) messageId = 1;
    if (ackRequested) {
        pendingAckMessageId = messageId; // to id trafia do ramki i wroci w potwierdzeniu
        // Okno ACK liczymy od faktycznego nadania, nie od zakolejkowania. Ramka mogla
        // czekac w buforze (np. az watchdog odblokuje zawieszony TX) - ze stemplem
        // z chwili zakolejkowania okno wygasaloby w momencie startu nadawania.
        waitForAckStartTime = millis();
        ackFramePendingTx = false;
    }
    if (!fifoReady) {
        transmissionFinished = true;
        LoRa_rxMode();
        return true; // ramka przepada, ale cykl jest domkniety
    }
    uint8_t header[RADIO_HEADER_SIZE];
    header[0] = address;
    header[1] = nodeId;
    header[2] = messageId;
    header[3] = (uint8_t) (RADIO_PROTO_MARK | (ackRequested ? RADIO_FLAG_ACK_REQ : 0)
                           | (type & RADIO_TYPE_MASK));
    LoRa.write(header, RADIO_HEADER_SIZE);
    LoRa.write(payload, len);
    // Dopiero teraz - ramka jest w FIFO i nadawanie na pewno ruszy.
    transmissionFinished = false;
    transmissionClenedUp = false;
    sendingTime = micros();
    txStartMillis = millis();
    LoRa.endPacket(true);
    return true;
}

// Jedna linia stanu radia i warstwy ACK - do czarnej skrzynki (co ~10 s z main.cpp).
// Rejestry SX1276: 0x01 OP_MODE (RX_CONT=0x85, STDBY=0x81, TX=0x83), 0x40 DIO_MAPPING_1
// (0x00 = RxDone na DIO0, 0x40 = TxDone), 0x12 IRQ_FLAGS (0x40 RxDone, 0x08 TxDone,
// 0x20 CRC err). EIMSK bit0 = przerwanie INT0 (DIO0) odmaskowane.
void RadioManager::printRadioDiag() {
    Serial.print(F("DIAG rf mode=0x"));
    Serial.print(LoRa.peekRegister(0x01), HEX);
    Serial.print(F(" dio=0x"));
    Serial.print(LoRa.peekRegister(0x40), HEX);
    Serial.print(F(" irq=0x"));
    Serial.print(LoRa.peekRegister(0x12), HEX);
    Serial.print(F(" eimsk=0x"));
    Serial.print(EIMSK, HEX);
    Serial.print(F(" txFin="));
    Serial.print(transmissionFinished);
    Serial.print(F(" rxFlag="));
    Serial.print(receivedFlag);
    Serial.print(F(" wait="));
    Serial.print(waitingForAck);
    Serial.print(F(" pendTx="));
    Serial.print(ackFramePendingTx);
    Serial.print(F(" buf="));
    Serial.print(txPending ? txLength : 0);
    Serial.print(F(" pwr="));
    Serial.print(txPowerDbm);
    Serial.print(F(" ram="));
    Serial.print(freeRam());
    Serial.print(F(" stk="));       // najmniejszy zapas sterta<->stos od startu (malowanie)
    Serial.println(minStackGap());
}

void RadioManager::LoRa_txMode() {
    LoRa.idle();                          // set standby mode
}

// Kontekst przerwania (ISR)! Zadnych Serial.print ani String (malloc nie jest
// reentrantny - alokacja w ISR w trakcie alokacji w petli glownej psuje sterte).
void RadioManager::receiveDone(int packetSize) {
    if (packetSize > 0) {
        receivedFlag = true;
        receivedPacketSize = packetSize;
    } else {
        zeroLengthPacketReceived = true;
    }
}

// Kontekst przerwania (ISR) - jak wyzej. Logi i dataSentCallback z sendLoop().
void RadioManager::txDone() {
    LoRa_rxMode();
    txDoneTime = micros();
    transmissionFinished = true;
}

void RadioManager::LoRa_rxMode() {
    LoRa.receive();                       // set receive mode
}


void RadioManager::DEBUGlogln(const __FlashStringHelper *ifsh) {
    if (LOG_ACTIVE) Serial.println(ifsh);
}

void RadioManager::DEBUGlog(const __FlashStringHelper *ifsh) {
    if (LOG_ACTIVE) Serial.print(ifsh);
}

void RadioManager::DEBUGlogln() {
    if (LOG_ACTIVE) Serial.println();
}

void RadioManager::DEBUGlogln(int n, int base) {
    if (LOG_ACTIVE) Serial.println(n, base);
}

void RadioManager::DEBUGlog(int n, int base) {
    if (LOG_ACTIVE) Serial.print(n, base);
}

void RadioManager::dumpRegisters() {
    LoRa.dumpRegisters(Serial);
}

// Ustawia moc nadawania w dBm (PA_BOOST). Wartosci spoza zakresu sa przycinane
// do TX_POWER_MIN_DBM..TX_POWER_MAX_DBM. Mozna wywolywac w dowolnym momencie,
// takze w trakcie pracy - zmiana obowiazuje od nastepnej transmisji.
int8_t RadioManager::setTxPower(int8_t dbm) {
    if (dbm < TX_POWER_MIN_DBM) dbm = TX_POWER_MIN_DBM;
    if (dbm > TX_POWER_MAX_DBM) dbm = TX_POWER_MAX_DBM;

    LoRa.setTxPower(dbm, PA_OUTPUT_PA_BOOST_PIN);
    if (dbm > 17) {
        // biblioteka ustawia w tym miejscu OCP=140 mA, czyli ponizej poboru PA przy 20 dBm
        LoRa.setOCP(TX_OCP_HIGH_POWER_MA);
    }

    txPowerDbm = dbm;
    return dbm;
}

int8_t RadioManager::getTxPower() {
    return txPowerDbm;
}

// Wartosci orientacyjne z noty SX1276 - sluza do oceny, czy zasilanie wyrobi.
uint16_t RadioManager::getTxCurrentEstimate_mA() {
    if (txPowerDbm > 17) return 130;
    if (txPowerDbm >= 15) return 90;
    if (txPowerDbm >= 10) return 40;
    if (txPowerDbm >= 5) return 30;
    return 25;
}

void RadioManager::printTxPower() {
    Serial.print(F("[RADIO] TX power: "));
    Serial.print(txPowerDbm);
    Serial.print(F(" dBm (PA_BOOST, ~"));
    Serial.print(getTxCurrentEstimate_mA());
    Serial.println(F(" mA podczas nadawania)"));
    if (txPowerDbm > TX_POWER_FTDI_SAFE_DBM) {
        Serial.println(F("[RADIO] UWAGA: przy tej mocy zasilaj plytke z baterii albo 5V"));
        Serial.println(F("[RADIO] przez regulator - pin 3V3 FTDI spowoduje reset (BROWN_OUT)."));
    }
    if (txPowerDbm > 17) {
        Serial.println(F("[RADIO] Tryb wysokiej mocy (PA_DAC): Semtech zaleca duty cycle <= 1%."));
    }
}
