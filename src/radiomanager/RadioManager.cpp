//
// Created by LukaszLibront on 19.12.2023.
//

#include "RadioManager.h"


// Bufory nadawcze rezerwowane tutaj, czyli w inicjalizacji globali - sterta jest
// pusta, wiec laduja na jej dnie i nigdy nie fragmentuja niczego. Gdyby ta jedna
// alokacja padla, plytka i tak nie nadaje sie do pracy - komunikat pojawi sie przy
// pierwszej probie nadania ("ramka pusta albo za dluga").
RadioManager::RadioManager() {
    sendBuffer.reserve(RADIO_TX_BUFFER_CAPACITY);
    ackSendBuffer.reserve(RADIO_ACK_BUFFER_CAPACITY);
}

void RadioManager::setupRadio(long frequency, int ss, int reset, int dio0, uint8_t _nodeId, void(*receiveDoneCallback)(int), void(*txDoneCallback)()) {
    nodeId = _nodeId;
//    LoRa.setPins(10, 7, 2);
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

void RadioManager::onDataReceived(void(*callback)(String &receivedText, uint8_t senderId)) {
    dataReceivedCallback = callback;
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

bool RadioManager::isTransmissionFinished() {
    return transmissionFinished;
}

bool RadioManager::isNeedToSendAckToSender(){
    return needToSendAckToSender;
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
// sendBuffer nie zostalby oprozniony i kazda kolejna wysylka bylaby odrzucana
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
    if (transmissionFinished) {
        if (!transmissionClenedUp) {
            transmissionClenedUp = true;
            DEBUGlogln(F("transmission finished!"));
            if (LOG_ACTIVE) {
                DEBUGlog(F("SENDING TIME = "));
                DEBUGlog(String(txDoneTime - sendingTime));
                DEBUGlogln(F(" us"));
            }
            if (dataSentCallback) {
                dataSentCallback();
            }
        } else if (txPending || ackSendBuffer.length() > 0) {
            // Flagi transmisji ustawia dopiero startSending, i to tylko gdy naprawde
            // ruszy nadawanie. Wczesniej zerowalismy je TUTAJ, wiec gdy startSending
            // przerwal z braku RAM, transmissionFinished zostawalo false na zawsze -
            // przerwanie TxDone nigdy nie mialo skad przyjsc i radio wisialo do
            // zadzialania watchdoga (2 s), a pakiet i tak przepadal.
            DEBUGlogln(F("[RFM96] Sending another packet ... "));
            if (ackSendBuffer.length() > 0) {
                DEBUGlogln(F("[RFM96] Sending ACK packet ... "));
                frameTxPwrOverride = -1; // ACK-i zawsze moca regulowana
                if (startSending(ackSendBuffer, ackSendBufferDest, false)) ackSendBuffer = "";
            } else {
                DEBUGlogln(F("[RFM96] Sending normal packet ... "));
                // Bufor zostaje przy niepowodzeniu - kolejny obieg petli sprobuje ponownie,
                // zamiast po cichu gubic pakiet (razem z ewentualnym wymuszeniem mocy).
                frameTxPwrOverride = sendBufferTxPwrOverride;
                if (startSending(sendBuffer, sendBufferDest, sendBufferAckReq)) {
                    // Tresc NIE jest kasowana: ramka z zadaniem ACK zostaje jako payload
                    // ewentualnego ponowienia (resendRetained) i callbacku bledu.
                    txPending = false;
                    retainedFrameValid = sendBufferAckReq;
                    sendBufferTxPwrOverride = -1;
                }
                frameTxPwrOverride = -1;
            }
        }
    }
}

void RadioManager::receiveLoop() {
    if (zeroLengthPacketReceived) {
        zeroLengthPacketReceived = false;
        DEBUGlogln(F("ERROR: Received 0 lenght packet!!!"));
    }
    if (receivedFlag) {
        receivedFlag = false;
        String str = readReceivedData();

        DEBUGlog(F("[ACK] | before extractMessageIdAndSenderIdAndDestinationIdFromReceivedData = "));
        DEBUGlogln(str);
        extractMessageIdAndSenderIdAndDestinationIdFromReceivedData(str);
        DEBUGlog(F("[ACK] | extracted senderIdOfLastMessage = "));
        DEBUGlogln(senderIdOfLastMessage);
        DEBUGlog(F("[ACK] | extracted receivedMessageIdOfLastMessage = "));
        DEBUGlogln(receivedMessageIdOfLastMessage);
        DEBUGlog(F("[ACK] | extracted needToSendAckToSender = "));
        DEBUGlogln(needToSendAckToSender);
        DEBUGlog(F("[ACK] | after extractMessageIdAndSenderIdAndDestinationIdFromReceivedData = "));
        DEBUGlogln(str);

        if (destinationIdOfLastMessage == nodeId
            || destinationIdOfLastMessage == RADIO_BROADCAST_ID) {
            DEBUGlogln(F("This is a destination address"));
        } else {
            DEBUGlogln(F("This is not a destination address, ignoring message"));
            return;
        }
        // Ramka "od nas samych" nie ma prawa istniec: to echo wlasnego payloadu
        // odczytane z FIFO po wyscigu z nadawaniem albo przeklamany naglowek.
        if (senderIdOfLastMessage == nodeId) {
            Serial.println(F("RadioManager | ramka z wlasnym id nadawcy - odrzucam"));
            return;
        }

        // KAZDA poprawnie zaadresowana ramka (dane, ACK, beacon) jest dowodem, ze
        // lacze od nadawcy zyje - mesh odswieza tym swoich sasiadow, zeby zgubione
        // beacony (broadcast bez ACK, gina w kolizjach) nie usmiercaly zywych tras.
        if (anyFrameReceivedCallback && senderIdOfLastMessage != 0) {
            anyFrameReceivedCallback(senderIdOfLastMessage);
        }

        if (isAckPayloadAndValidMessageId(str) && waitingForAck) {
            DEBUGlogln(F("Received ACK"));
            if (LOG_ACTIVE) {
                DEBUGlog(F("RECEIVED ACK TIME = "));
                DEBUGlog(String(micros() - sendingTime));
                DEBUGlogln(F(" us"));
            }
            ackReceived = true;
            waitingForAck = false;
            pendingAckMessageId = 0;
            apcOnAckPayload(str); // zwrotka "@<rssi>" -> krok regulatora mocy
            if (ackReceivedCallback) {
                ackReceivedCallback();
            }
        } else {
            if (isAckPayload(str)) {
                // spozniony/niedopasowany ACK - nie publikuj go jako danych
                DEBUGlogln(F("RadioManager | stale/unmatched ACK - ignoring"));
                return;
            }
            if (receivedMessageIdOfLastMessage != 0) {
                DEBUGlogln(F("RadioManager | checking if it is need to send ACK"));
                DEBUGlog(F("RadioManager | !isAckPayload(str) = "));
                DEBUGlogln(!isAckPayload(str));
                DEBUGlog(F("RadioManager | needToSendAckToSender = "));
                DEBUGlogln(needToSendAckToSender);
                if (!isAckPayload(str) && needToSendAckToSender) {
                    DEBUGlogln(F("RadioManager | inside: !isAckPayload(str) && needToSendAckToSender && sendAckAutomaticly"));
                    sendAck();
                }
            } else {
                DEBUGlogln(F("[ACK] | Can not extract message id from received message"));
            }


            // Znacznik zdejmowany remove() W MIEJSCU: substring(5) tworzyl druga
            // pelna kopie ramki (do ~130 B) w najciasniejszym momencie odbioru.
            if (isOtaPayload(str)) {
                DEBUGlogln(F("Received OTA message"));
                str.remove(0, 5);
                if (otaDataReceivedCallback) {
                    otaDataReceivedCallback(str, senderIdOfLastMessage);
                }
            } else if (isMeshPayload(str)) {
                str.remove(0, 5);
                if (meshDataReceivedCallback) {
                    meshDataReceivedCallback(str, senderIdOfLastMessage);
                }
            } else if (isDataPayload(str)){
                DEBUGlogln(F("Received DATA message"));
                str.remove(0, 5);
                if (dataReceivedCallback) {
                    dataReceivedCallback(str, senderIdOfLastMessage);
                }
            } else {
                DEBUGlog(F("Wrong message format, message: "));
                DEBUGlogln(str);
            }
        }
    }
}

void RadioManager::sendAck() {
    DEBUGlog(F("Sending ACK to address: "));
    DEBUGlogln(senderIdOfLastMessage);
    // Skladane wprost w buforze ACK (pojemnosc zarezerwowana raz) - bez Stringa
    // tymczasowego i bez kopii. Nowszy ACK moze nadpisac starszy, ktory jeszcze nie
    // wyszedl - nadawca tamtej ramki i tak ja ponowi.
    ackSendBuffer = "!";
    ackSendBuffer.concat(receivedMessageIdOfLastMessage);
    // Zwrotka APC: RSSI, z jakim uslyszelismy kwitowana ramke (lastRssi jest z TEJ
    // ramki - ta sama iteracja receiveLoop). Stary parser czyta "!<id>@<rssi>" bez
    // zmian, bo toInt() konczy na '@'.
    ackSendBuffer.concat('@');
    ackSendBuffer.concat(lastRssi);
    ackSendBufferDest = senderIdOfLastMessage;
}

int RadioManager::getLastRssi() {
    return lastRssi;
}

String RadioManager::readReceivedData() {
    // RSSI odczytany zanim cokolwiek innego zdazy sie wydarzyc - rejestr PktRssiValue
    // nadpisuje dopiero kolejny odebrany pakiet. Kontekst petli glownej (nie ISR),
    // wiec dostep po SPI jest bezpieczny.
    lastRssi = LoRa.packetRssi();
    String str = "";
    str.reserve(receivedPacketSize + 1); // jedna alokacja zamiast realokacji przy kazdym znaku
//    for(int & receivedByte : receivedBytes) {
//        receivedByte = 0;
//    }
//    int index = 0;
    while (LoRa.available()) {
//        receivedBytes[index] = LoRa.read();
        str += (char) LoRa.read();
//        str += (char) receivedBytes[index];
//        index++;
    }
//    Serial.print(F("readReceivedData, str.length()")); Serial.println(str.length());

//    int receivedBytes[256];
//    int index = 0;
//    while (LoRa.available()) {
//        receivedBytes[index++] = LoRa.read();
//    }
//    Serial.print(F("readReceivedData, str.length()")); Serial.println(str.length());


//    DEBUGlog(F("BEFORE remove: "));
//    DEBUGlogln(str);
//    str.remove(str.indexOf("`")); //TODO usunięcie tylko ostatniego takiego znaku

    str.remove(str.lastIndexOf("`")); //TODO usunięcie tylko ostatniego takiego znaku
//    str.remove(str.length() - 1);
//    DEBUGlog(F("AFTER remove: "));
//    DEBUGlogln(str);

    DEBUGlogln();
    DEBUGlogln(F("[RFM96] Received packet!"));

    // print data of the packet
    DEBUGlog(F("[RFM96] Data:\t\t"));
    DEBUGlogln(str);

//        // print senderIdOfLastMessage
//        DEBUGlog(F("[RFM96] SendeId:\t\t"));
//        DEBUGlogln(radio.getSenderId());

    // Calosc pod if: DEBUGlog wycisza tylko wydruk, argumenty i tak by sie policzyly,
    // a packetSnr/packetFrequencyError to lacznie ~5 odczytow rejestrow po SPI na
    // KAZDA odebrana ramke (takze kazdy pakiet OTA). RSSI mamy juz w lastRssi.
    if (LOG_ACTIVE) {
        DEBUGlog(F("[RFM96] RSSI:\t\t"));
        DEBUGlog(lastRssi);
        DEBUGlogln(F(" dBm"));

        DEBUGlog(F("[RFM96] SNR:\t\t"));
        DEBUGlog(LoRa.packetSnr());
        DEBUGlogln(F(" dB"));

        DEBUGlog(F("[RFM96] Frequency error:\t"));
        DEBUGlog(LoRa.packetFrequencyError());
        DEBUGlogln(F(" Hz"));
    }

    return str;
}

// Naglowek "adr@nadawca@id@ack@" parsowany W MIEJSCU (strtoul po c_str) i zdejmowany
// jednym remove(). Poprzednik (splitString) robil 4 kolejne kopie reszty ramki przez
// substring - do ~130 B chwilowej sterty na kazda odebrana ramke, akurat wtedy, gdy
// zywa jest juz pelna kopia ramki z FIFO. Zly naglowek zeruje adresata i id, wiec
// receiveLoop odrzuci ramke, zamiast dzialac na polach z POPRZEDNIEJ ramki.
void RadioManager::extractMessageIdAndSenderIdAndDestinationIdFromReceivedData(String &str) {
    const char *base = str.c_str();
    char *cursor;
    unsigned long dest = strtoul(base, &cursor, 10);
    bool ok = (cursor != base) && *cursor == '@';
    unsigned long sender = ok ? strtoul(cursor + 1, &cursor, 10) : 0;
    ok = ok && *cursor == '@';
    unsigned long id = ok ? strtoul(cursor + 1, &cursor, 10) : 0;
    ok = ok && *cursor == '@';
    unsigned long ack = ok ? strtoul(cursor + 1, &cursor, 10) : 0;
    ok = ok && *cursor == '@';
    if (!ok || dest > 255 || sender > 255 || id > 255) {
        destinationIdOfLastMessage = 0;
        senderIdOfLastMessage = 0;
        receivedMessageIdOfLastMessage = 0;
        needToSendAckToSender = false;
        return;
    }
    destinationIdOfLastMessage = (uint8_t) dest;
    senderIdOfLastMessage = (uint8_t) sender;
    receivedMessageIdOfLastMessage = (uint8_t) id;
    needToSendAckToSender = ack != 0;
    str.remove(0, (unsigned int) ((cursor + 1) - base));
}

bool RadioManager::isAckPayload(const String &str) {
//    DEBUGlog(F("isAckPayload, str = "));
//    DEBUGlogln(str);
    return str.charAt(0) == '!';
}

// Przez referencje (bylo: przez wartosc = kopia calego payloadu przy KAZDEJ ramce).
bool RadioManager::isAckPayloadAndValidMessageId(const String &str) {
    if (str.charAt(0) != '!') return false;
    // porownujemy z id ramki DAT czekajacej na ACK, a nie z zywym licznikiem
    // messageId - ten podbija tez kazda wyslana ramka ACK
    unsigned long id = strtoul(str.c_str() + 1, nullptr, 10); // strtoul konczy na '@'
    return pendingAckMessageId != 0 && id == pendingAckMessageId;
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
                ackNotReceivedCallback(sendBuffer); // nadana ramka wciaz lezy w buforze
            }
        }
    }
    // Bezpiecznik ostateczny: zadna kombinacja flag nie ma prawa trzymac waitingForAck
    // dluzej niz 3x timeout (ramka w buforze nadaje sie najpozniej po 2 s dzieki
    // txStuckWatchdog). Widziane na sprzecie jako trwale zakleszczenie po zbiegu
    // restartow obu wezlow: wysylki wiecznie "pominiete", timeout nigdy nie strzelal.
    // Zgłaszamy to normalna sciezka bledu, zeby warstwa wyzej (mesh) zadzialala.
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
            ackNotReceivedCallback(sendBuffer);
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

void RadioManager::apcOnAckPayload(const String &ackPayload) {
    if (!APC_ENABLED) return;
    ackMissStreak = 0;
    int atPos = ackPayload.indexOf('@');
    if (atPos < 0) return; // ACK ze starszego firmware - bez zwrotki RSSI
    long reported = atol(ackPayload.c_str() + atPos + 1);
    // RSSI >= 0 dBm jest fizycznie niemozliwe dla LoRa: to obcieta zwrotka ("!17@"
    // po cichym OOM u peera daje atol("") == 0) albo smieci. Regulowanie wedlug takiej
    // wartosci sciagaloby moc W DOL dokladnie wtedy, gdy peerowi brakuje pamieci.
    if (reported >= 0) return;
    peerReportedRssi = (int) reported;
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
    // takich epizodow, kazdy konczony dopiero powolnym zejsciem sondy w dol
    // (rozklad markerow [P10]/[P8]/[P6]/[P4] po ~5 tys. ramek). Krok w gore
    // z bliska nie opuszcza strefy dzialajacych mocy (epizod konczy sie po
    // jednej serii), a w terenie dociera do sufitu w 2-4 serie - regulacja ze
    // zwrotek i tak obsluguje stopniowe slabniecie lacza, sonda jest od strat
    // calkowitych.
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
// liczba, a nie chwilowe "ram=", mowi, czy __malloc_margin (128 B) jest za duzy.
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
// Bufor nadawczy ma stala pojemnosc (RADIO_TX_BUFFER_CAPACITY) zarezerwowana raz w
// konstruktorze. Wolajacy sklada ramke WPROST w nim: acquireTxBuffer -> zapis ->
// commitTxBuffer. Zadnych Stringow tymczasowych ani kopii po drodze: ta goraca
// sciezka nie alokuje nic, wiec "brak RAM" nie ma gdzie wystapic.
//
// Arduino String sygnalizuje brak pamieci wylacznie po cichu (operator+= ignoruje
// blad, operator+ czysci CALY string), dlatego rozmiar sprawdza sie PRZED skladaniem
// (txBufferFits) - wtedy concat w granicach pojemnosci nie ma prawa zawiesc.

String *RadioManager::acquireTxBuffer() {
    // Jedna transakcja naraz: ramka czekajaca na nadanie ALBO trwajaca transakcja ACK
    // blokuje bufor takze dla ramek bez ACK. Wczesniej ramka bez ACK (beacon) mogla
    // wjechac w oczekiwanie na ACK; teraz nadpisalaby zachowany payload ponowienia.
    if (txPending) {
        DEBUGlogln(F("RadioManager | busy: ramka czeka na nadanie"));
        return nullptr;
    }
    if (waitingForAck) {
        DEBUGlogln(F("RadioManager | busy: waiting for ACK"));
        return nullptr;
    }
    retainedFrameValid = false; // to, co lezalo w buforze, wlasnie przepada
    sendBuffer = "";            // pojemnosc zostaje - to nie jest realokacja
    return &sendBuffer;
}

void RadioManager::releaseTxBuffer() {
    sendBuffer = "";
}

bool RadioManager::commitTxBuffer(uint8_t address, bool ackRequested,
                                  void (*_ackReceivedCallback)(),
                                  void (*_ackNotReceivedCallback)(String &payload),
                                  int8_t txPowerDbmOverride) {
    // Pusta ramka nigdy nie moze uzbroic transakcji: sendLoop nadaje tylko niepuste
    // bufory, wiec ksiegowosc ACK dla pustej ramki wisialaby az do watchdoga (zrodlem
    // pustych ramek byl kiedys cichy OOM Stringa). Za dluga = String realokowal bufor
    // poza umowiona pojemnosc - takiej ramki tez nie chcemy w eterze.
    if (sendBuffer.length() == 0 || sendBuffer.length() > RADIO_TX_BUFFER_CAPACITY) {
        Serial.print(F("RadioManager | ERROR: ramka pusta albo za dluga ("));
        Serial.print(sendBuffer.length());
        Serial.println(F(" B) - nie wyslano"));
        releaseTxBuffer();
        return false;
    }
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
    // wola sendLoop co obieg) - wolajacy moze jeszcze trzymac wlasne Stringi.
    txPending = true;
    return true;
}

bool RadioManager::hasRetainedFrame() {
    return retainedFrameValid && !txPending && !waitingForAck && sendBuffer.length() > 0;
}

String &RadioManager::retainedFrame() {
    static String none;
    return retainedFrameValid ? sendBuffer : none;
}

// Ponowienie ostatniej nadanej ramki z zadaniem ACK - tresc nadal lezy w buforze,
// wiec nie ma zadnej kopii. false = ramki juz nie ma (ktos zajal bufor) albo radio zajete.
bool RadioManager::resendRetained(uint8_t address, void (*_ackReceivedCallback)(),
                                  void (*_ackNotReceivedCallback)(String &payload)) {
    if (!hasRetainedFrame()) return false;
    return commitTxBuffer(address, true, _ackReceivedCallback, _ackNotReceivedCallback);
}

// "<TAG>" + tresc, skladane wprost w buforze nadawczym.
bool RadioManager::sendWithTag(const __FlashStringHelper *tag, const String &body, uint8_t address,
                               void (*_ackReceivedCallback)(), void (*_ackNotReceivedCallback)(String &payload)) {
    String *frame = acquireTxBuffer();
    if (frame == nullptr) return false;
    if (!txBufferFits(body.length() + 5)) {
        Serial.print(F("RadioManager | ERROR: ramka za dluga ("));
        Serial.print(body.length() + 5);
        Serial.println(F(" B) - nie wyslano"));
        releaseTxBuffer();
        return false;
    }
    *frame = tag;
    *frame += body;
    return commitTxBuffer(address, _ackReceivedCallback || _ackNotReceivedCallback,
                          _ackReceivedCallback, _ackNotReceivedCallback);
}

bool RadioManager::sendOta(String &str, uint8_t address, void (*_ackReceivedCallback)(), void (*_ackNotReceivedCallback)(String &payload)) {
    return sendWithTag(F("<OTA>"), str, address, _ackReceivedCallback, _ackNotReceivedCallback);
}

bool RadioManager::send(String &str, uint8_t address, void (*_ackReceivedCallback)(), void (*_ackNotReceivedCallback)(String &payload)) {
    return sendWithTag(F("<DAT>"), str, address, _ackReceivedCallback, _ackNotReceivedCallback);
}

// Payload JUZ ze znacznikiem ("<MSH>...", "<OTA>...") - jedna kopia do bufora nadawczego.
bool RadioManager::sendTagged(String &taggedPayload, uint8_t address,
                              void (*_ackReceivedCallback)(), void (*_ackNotReceivedCallback)(String &payload)) {
    return sendDirectly(taggedPayload, address,
                        _ackReceivedCallback || _ackNotReceivedCallback,
                        _ackReceivedCallback, _ackNotReceivedCallback);
}

// Zwraca false gdy radio jest zajete (ramka czeka na wyslanie albo trwa transakcja
// ACK) - wtedy NIC nie jest nadpisywane i nalezy ponowic wysylke pozniej. Ciche
// nadpisywanie bufora gubilo ramki OTA, gdy ruch testowy i transfer szly rownoczesnie.
bool RadioManager::sendDirectly(String &str, uint8_t address, bool ackRequested,
                                void (*_ackReceivedCallback)(), void (*_ackNotReceivedCallback)(String &payload)) {
    if (str.length() == 0) {
        DEBUGlogln(F("RadioManager | pusta ramka - odrzucam"));
        return false;
    }
    String *frame = acquireTxBuffer();
    if (frame == nullptr) return false;
    if (!txBufferFits(str.length())) {
        Serial.print(F("RadioManager | ERROR: ramka za dluga ("));
        Serial.print(str.length());
        Serial.println(F(" B) - nie wyslano"));
        releaseTxBuffer();
        return false;
    }
    *frame = str; // pojemnosc juz jest - kopia bez malloc
    return commitTxBuffer(address, ackRequested, _ackReceivedCallback, _ackNotReceivedCallback);
}

bool RadioManager::startSending(String &str, uint8_t address, bool ackRequested) {
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
    DEBUGlog(F("Sending: ["));
    DEBUGlog(str);
    DEBUGlog(F("] to "));
    DEBUGlogln(address);
    // Ramka idzie do FIFO radia KAWALKAMI: naglowek "adr@nadawca@id@ack@", tresc,
    // ogranicznik. Nie ma juz Stringa z gotowa ramka - to byla najwieksza pojedyncza
    // alokacja chwilowa w systemie (~130 B na 2 KB RAM) i jedyny powod, dla ktorego
    // nadanie z gotowego bufora moglo w ogole zawiesc na braku pamieci.
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
        pendingAckMessageId = messageId; // to id trafia do ramki i wroci w "!id"
        // Okno ACK liczymy od faktycznego nadania, nie od zakolejkowania. Ramka mogla
        // czekac w buforze (np. az watchdog odblokuje zawieszony TX) - ze stemplem
        // z chwili zakolejkowania okno wygasaloby w momencie startu nadawania i
        // timeout zerowalby pendingAckMessageId tuz przed nadejsciem prawdziwego ACK.
        waitForAckStartTime = millis();
        ackFramePendingTx = false;
    }
    if (!fifoReady) {
        transmissionFinished = true;
        LoRa_rxMode();
        return true; // ramka przepada, ale cykl jest domkniety
    }
    LoRa.print(address);
    LoRa.print('@');
    LoRa.print(nodeId);
    LoRa.print('@');
    LoRa.print(messageId);
    LoRa.print('@');
    LoRa.print(ackRequested ? '1' : '0');
    LoRa.print('@');
    LoRa.print(str);
    LoRa.print('`');
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
    Serial.print(txPending ? sendBuffer.length() : 0);
    Serial.print(F(" pwr="));
    Serial.print(txPowerDbm);
    Serial.print(F(" ram="));
    Serial.print(freeRam());
    Serial.print(F(" stk="));       // najmniejszy zapas sterta<->stos od startu (malowanie)
    Serial.println(minStackGap());
}

void RadioManager::LoRa_txMode() {
    LoRa.idle();                          // set standby mode
//    LoRa.disableInvertIQ();               // node
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
//    LoRa.enableInvertIQ();                // node
    LoRa.receive();                       // set receive mode
}


void RadioManager::DEBUGlogln(const __FlashStringHelper *ifsh) {
    if (LOG_ACTIVE) Serial.println(ifsh);
}

void RadioManager::DEBUGlog(const __FlashStringHelper *ifsh) {
    if (LOG_ACTIVE) Serial.print(ifsh);
}

void RadioManager::DEBUGlogln(const String &s) {
    if (LOG_ACTIVE) Serial.println(s);
}

void RadioManager::DEBUGlog(const String &s) {
    if (LOG_ACTIVE) Serial.print(s);
}

void RadioManager::DEBUGlogln(unsigned char b, int base) {
    if (LOG_ACTIVE) Serial.println(b, base);
}

void RadioManager::DEBUGlog(unsigned char b, int base) {
    if (LOG_ACTIVE) Serial.print(b, base);
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

void RadioManager::DEBUGlogln(double n, int digits) {
    if (LOG_ACTIVE) Serial.println(n, digits);
}

void RadioManager::DEBUGlog(double n, int digits) {
    if (LOG_ACTIVE) Serial.print(n, digits);
}

void RadioManager::DEBUGlogln(long n, int base) {
    if (LOG_ACTIVE) Serial.println(n, base);
}

void RadioManager::DEBUGlog(long n, int base) {
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

void RadioManager::onOtaDataReceived(void (*callback)(String &, uint8_t)) {
    otaDataReceivedCallback = callback;
}

void RadioManager::onMeshDataReceived(void (*callback)(String &, uint8_t)) {
    meshDataReceivedCallback = callback;
}

void RadioManager::onAnyFrameReceived(void (*callback)(uint8_t senderId)) {
    anyFrameReceivedCallback = callback;
}

bool RadioManager::isMeshPayload(String &str) {
    const char *data = str.c_str();
    return str.length() >= 5 && data[0] == '<' && data[1] == 'M' && data[2] == 'S'
           && data[3] == 'H' && data[4] == '>';
}

bool RadioManager::isOtaPayload(String &str) {
    DEBUGlog(F("isOtaPayload, str = "));
    DEBUGlogln(str);

    const char* data = str.c_str();
    bool returnValue = data[0] == '<' && data[1] == 'O' && data[2] == 'T' && data[3] == 'A' && data[4] == '>';
//    bool returnValue = str.startsWith("<OTA>");

    DEBUGlog(F("isOtaPayload, returnValue = "));
    DEBUGlogln(returnValue);
    return returnValue;
}

bool RadioManager::isDataPayload(String &str) {
    DEBUGlog(F("isDataPayload, str = "));
    DEBUGlogln(str);
    const char* data = str.c_str();
    bool returnValue = data[0] == '<' && data[1] == 'D' && data[2] == 'A' && data[3] == 'T' && data[4] == '>';
//    bool returnValue = str.startsWith("<DAT>");

    DEBUGlog(F("isDataPayload, returnValue = "));
    DEBUGlogln(returnValue);
    return returnValue;
}

bool RadioManager::isDataSent() {
    return transmissionFinished;
}
