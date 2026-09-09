//
// Created by LukaszLibront on 29.12.2023.
//

#include "RadioOta.h"


#define DEBUG false


//TODO numerowanie pakietó i sprawdzaniae czy przysZedł kolejny, czy są brakulub powtórzenia
//TODO więcej logów

void RadioOta::loop() {
if (otaState == OtaState(SENDING_WIRELESS_HANDSHAKE)) {
        if (handshakeTryes >= HANDSHAKE_SENDING_TRYES_LIMIT) {
            Serial.println(F("FLX?HANDSHAKE?TIMEOUT"));
            resetStateAndValues();
            Serial.println(F("OTA | Handshake response not received, retries number exceeded. Not sending again"));
        } else if (radioSendHandshake()) { // false = radio zajete, ponow w nastepnym obiegu petli
            Serial.println(F("OTA | state = SENDING_WIRELESS_HANDSHAKE"));
            handshakeTryes++;
            handshakeSendStartTime = millis();
            Serial.println(F("OTA | going to WAITING_FOR_WIRELESS_HANDSHAKE_RESPONSE state"));
            otaState = OtaState(WAITING_FOR_WIRELESS_HANDSHAKE_RESPONSE);
        }
    } else if (otaState == OtaState(WAITING_FOR_WIRELESS_HANDSHAKE_RESPONSE)) {
        // jitter 0-255 ms: obie strony ponawiaja co ~1 s, bez jittera raz zderzone
        // retransmisje kolidowalyby ze soba w rytmie az do wyczerpania prob
        if (millis() - handshakeSendStartTime > 1000 + (micros() & 0xFF)) {
            otaState = OtaState(SENDING_WIRELESS_HANDSHAKE);
            Serial.println(F("OTA | Trying to send handshake again"));
        }
    } else if (otaState == OtaState(WIRELESS_HANDSHAKE_RESPONSE_RECEIVED)) {
        Serial.println(F("OTA | state = WIRELESS_HANDSHAKE_RESPONSE_RECEIVED"));
        // Do "OK" doklejamy wlasny limit dlugosci linii, zeby PC mogl sprawdzic, czy jego
        // rozmiar pakietu w ogole tu przejdzie - ZANIM wysle tysiac pakietow, ktore
        // wszystkie failowalyby na CRC przez ucinana koncowke.
        Serial.print(F("FLX?HANDSHAKE?OK?"));
        Serial.println(OTA_SERIAL_LINE_MAX);
        hexDataFromSerialStartTime = millis();
        otaState = OtaState(WAITING_FOR_HEX_DATA_FROM_SERIAL);
        Serial.println(F("OTA | going to WAITING_FOR_HEX_DATA_FROM_SERIAL state"));
    } else if (otaState == OtaState(WAITING_FOR_HEX_DATA_FROM_SERIAL)) {
        if (millis() - hexDataFromSerialStartTime > OTA_SERIAL_WAIT_MS) {
            if (serialResendRequests < OTA_SERIAL_RESEND_LIMIT) {
                // Zgubiona linia z PC (albo zgubione nasze "FLX?HEX?OK"): prosba o ponowienie
                // biezacego pakietu ta sama komenda, ktorej uzywa target przy zlym numerze.
                // Spozniona linia, ktora jednak dojdzie, jest po prostu obsluzona; duplikat
                // z ponowienia trafi w stan SENDING/WAITING i zostanie zignorowany.
                serialResendRequests++;
                Serial.print(F("FLX?HEX?WRONG_NUM?"));
                Serial.println(currentHexPacketNumber + 1);
                hexDataFromSerialStartTime = millis();
            } else {
                Serial.println(F("FLX?HEX?SERIAL_TIMEOUT"));
                resetStateAndValues();
                Serial.println(F("OTA | Hex from serial not received, timeout reached. Not waiting anymore"));
            }
        }
    } else if (otaState == OtaState(SENDING_WIRELESS_HEX)) {
        if (hexSendTryes >= HEX_SENDING_TRYES_LIMIT) {
            Serial.println(F("FLX?HEX?WIRELESS_TIMEOUT"));
            resetStateAndValues();
            Serial.println(F("OTA | FLASH IMG TRANSMISSION FAIL"));
            Serial.println(F("OTA | Handshake response not received, retries number exceeded. Not sending again"));
        } else if (radioSendHexFromSerial()) { // false = radio zajete, ponow w nastepnym obiegu petli
            if (DEBUG) Serial.println(F("OTA | state = SENDING_WIRELESS_HEX"));
            hexSendTryes++;
            hexSendStartTime = millis();
            if (DEBUG) Serial.println(F("OTA | going to WAITING_FOR_WIRELESS_HEX_RESPONSE state"));
            otaState = OtaState(WAITING_FOR_WIRELESS_HEX_RESPONSE);
        }
    } else if (otaState == OtaState(WAITING_FOR_WIRELESS_HEX_RESPONSE)) {
        if (millis() - hexSendStartTime > 1000 + (micros() & 0xFF)) { // jitter jak przy handshake
            otaState = OtaState(SENDING_WIRELESS_HEX);
            Serial.println(F("OTA | Trying to send hex again"));
        }
    } else if (otaState == OtaState(SENDING_WIRELESS_EOF)) {
        if (eofSendTryes >= EOF_SENDING_TRYES_LIMIT) {
            Serial.println(F("FLX?EOF?WIRELESS_TIMEOUT"));
            resetStateAndValues();
            Serial.println(F("OTA | FLASH IMG TRANSMISSION FAIL"));
            Serial.println(F("OTA | EOF response not received, retries number exceeded. Not sending again"));
        } else if (radioSendEof()) { // false = radio zajete, ponow w nastepnym obiegu petli
            Serial.println(F("OTA | state = SENDING_WIRELESS_EOF"));
            eofSendTryes++;
            eofSendStartTime = millis();
            Serial.println(F("OTA | going to WAITING_FOR_WIRELESS_EOF_RESPONSE state"));
            otaState = OtaState(WAITING_FOR_WIRELESS_EOF_RESPONSE);
        }
    } else if (otaState == OtaState(WAITING_FOR_WIRELESS_EOF_RESPONSE)) {
        if (millis() - eofSendStartTime > 10000 + (micros() & 0xFF)) { //aż 10 sek bo srawdzenie całego pliku może dlużej potrwać (sprawdzić doświadczalnei ile trwa)
            otaState = OtaState(SENDING_WIRELESS_EOF);
            Serial.println(F("OTA | Trying to send EOF again"));
        }
    }


    // W trakcie nadawania pakietu HEX i czekania na odpowiedz NIE czytamy linii do
    // _input: leza tam dane, ktore wlasnie ida w eter przy kazdym ponowieniu.
    // Spozniony duplikat z PC podmienilby je w polowie transferu i target dostalby
    // inna tresc niz numer, o ktory prosil. Takie linie po prostu wyrzucamy.
    if (Serial.available() && (otaState == OtaState(SENDING_WIRELESS_HEX)
                               || otaState == OtaState(WAITING_FOR_WIRELESS_HEX_RESPONSE))) {
        while (Serial.available()) {
            if (Serial.read() == 10) break;
        }
    }
    if (Serial.available()) {
        byte inputLen = readSerialLine(_input, 10, OTA_SERIAL_LINE_MAX, 100);
        if (inputLen >= OTA_SERIAL_LINE_MAX) {
            // Odczyt skonczyl sie na limicie, a nie na '\n' - linia zostala ucieta,
            // reszta trafi do kolejnego odczytu jako smiec. Bez tego komunikatu objawem
            // bylby tylko niezgodny CRC kazdego pakietu i transfer bez wyjasnienia.
            Serial.println(F("OTA | ERROR: linia z PC ucieta - OTA_PACKET_SIZE_BYTES nie zgadza sie z PC"));
        }
        if (inputLen > 0) {
            boolean configChanged = false;
            char *colon = strchr(_input, ':');

            if (strstr(_input, "EEPROMRESET") == _input) {
                resetEEPROM();
            } else if (strstr(_input, "SETTINGS?") == _input) {
                printSettings();
//            } else if (strstr(_input, "TO?") == _input) {
//                Serial << F("TO:") << targetID << F(":OK") << endl;
            } else if (strstr(_input, "PROGRAMMERID:") == _input && strlen(colon + 1) > 0) {
                uint16_t newId = atoi(++colon); //extract ID from message
                if (newId <= 1023) {
                    CONFIG.PROGRAMMER_ID = newId;
                    configChanged = true;
                } else {
                    Serial << F("Invalid nodeId:") << newId << endl;
                }
            } else if (inputLen > 7 && _input[0] == 'F' && _input[1] == 'L' && _input[2] == 'X' && _input[3] == '?' && _input[4] == 'T' && _input[5] == 'O' && _input[6] == '?') {
                if (otaState == OtaState(WAITING_FOR_SERIAL_HANDSHAKE)) {
                    long requested = atol(_input + 7);
                    // Adres jedzie w ramce jako jeden bajt: 0 nie istnieje, 255 to
                    // broadcast. Bez tej kontroli "FLX?TO?300" programowaloby wezel 44.
                    if (requested < 1 || requested > 254) {
                        Serial.println(F("FLX?HANDSHAKE?TIMEOUT"));
                        Serial.println(F("OTA | ERROR: nodeId poza zakresem 1..254"));
                    } else {
                        targetID = (uint16_t) requested;
                        otaState = (OtaState(SENDING_WIRELESS_HANDSHAKE));
                    }
                }
            } else if (inputLen > 8 && _input[0] == 'F' && _input[1] == 'L' && _input[2] == 'X' && _input[3] == '?' && _input[4] == 'H' && _input[5] == 'E' && _input[6] == 'X' && _input[7] == '?') {
                if (otaState == OtaState(WAITING_FOR_HEX_DATA_FROM_SERIAL)) {
                    // Bez kopiowania do Stringa: tresc zostaje w _input, ktory jest polem
                    // i przetrwa do nastepnej linii z PC (a Java nie przysyla kolejnej,
                    // dopoki nie odpowiemy). Kopia kosztowala ~106 B sterty plus dwa
                    // Stringi tymczasowe - na 2 KB to bylo tyle, ile brakowalo do
                    // zbudowania ramki przy 64-bajtowych pakietach.
                    // format z Javy: "<numer>?<base64>?<crc>"
                    currentHexPacketNumber = atol(_input + 8);
                    serialResendRequests = 0; // linia doszla - licznik prosb od nowa
                    if (DEBUG) { Serial.print(F("OTA | hex line = ")); Serial.println(_input + 8); }
                    otaState = OtaState(SENDING_WIRELESS_HEX);
                }
            } else if (inputLen > 8 && _input[0] == 'F' && _input[1] == 'L' && _input[2] == 'X' && _input[3] == '?' && _input[4] == 'E' && _input[5] == 'O' && _input[6] == 'F' && _input[7] == '?') {
                if (otaState == OtaState(WAITING_FOR_HEX_DATA_FROM_SERIAL)) {
                    // strtoul, a nie toInt(): CRC32 zajmuje pelne 32 bity bez znaku, a toInt()
                    // opiera sie na atol(), ktorego avr-libc nie definiuje dla przepelnienia
                    // ("result value is not predictable"). W praktyce atol zawija mod 2^32,
                    // wiec rzutowanie na uint32_t dawalo dobra wartosc - ale strtoul jest
                    // zdefiniowane dla calego zakresu, wiec nie polegamy na przypadku.
                    finalCrc32 = strtoul(_input + 8, nullptr, 10); // bez String/substring - zero alokacji
                    serialResendRequests = 0;
                    otaState = OtaState(SENDING_WIRELESS_EOF);
                }
//            } else if (strstr(_input, "TO:") == _input && strlen(colon + 1) > 0) {
//                uint16_t newTarget = atoi(++colon);
//                if (newTarget > 0 && newTarget <= 255 && newTarget != CONFIG.PROGRAMMER_ID) {
//                    targetID = newTarget;
//                    Serial << F("TO:") << targetID << F(":OK") << endl;
//                } else Serial << _input << F(":INV") << endl;
            } else Serial << F("UNKNOWN_CMD: ") << _input << (F(", state = ")) << otaState << endl; //echo back un

            if (configChanged) {
                EEPROM.writeBlock(0, CONFIG); //save changes to EEPROM
                printSettings();
            }
        }
    }

}

bool RadioOta::radioSendHexFromSerial() {
    // Ramka skladana WPROST w buforze nadawczym radia: zero alokacji. To najwiekszy
    // pojedynczy pakiet danych w calym systemie, a naglowek radiowy to teraz 4 bajty
    // zamiast dwudziestu znakow tekstu - tyle wiecej miejsca na same dane.
    const char *hexLine = _input + 8; // "<numer>?<base64>?<crc>" prosto z bufora serialowego
    size_t lineLen = strlen(hexLine);
    if (!RadioManager::txBufferFits(OTA_DAT_PREFIX_LEN + lineLen)) {
        // To nie jest chwilowa przeszkoda, tylko zla konfiguracja rozmiaru pakietu -
        // ponawianie w nieskonczonosc zostawialoby isOtaInProgress() na zawsze, a
        // z nim zamrozony mesh i wstrzymany ruch testowy. Przerywamy transfer.
        Serial.println(F("FLX?HEX?WIRELESS_TIMEOUT"));
        Serial.println(F("OTA | ERROR: linia HEX za dluga na ramke radiowa - przerywam"));
        resetStateAndValues();
        return false;
    }
    uint8_t *frame = manager->acquireTxBuffer();
    if (frame == nullptr) return false; // radio zajete - ponow w nastepnym obiegu
    memcpy_P(frame, PSTR("FLX?DAT?"), OTA_DAT_PREFIX_LEN);
    memcpy(frame + OTA_DAT_PREFIX_LEN, hexLine, lineLen);
    if (DEBUG) { Serial.print(F("OTA | radioSendHexFromSerial(), data = ")); Serial.println(hexLine); }
    return manager->commitTxBuffer((uint8_t) (OTA_DAT_PREFIX_LEN + lineLen), targetID,
                                   RADIO_TYPE_OTA, false);
}

bool RadioOta::radioSendHandshake() {
    return manager->sendOta("FLX?", targetID);
}

bool RadioOta::radioSendEof() {
    // "FLX?EOF?" + do 10 cyfr CRC32 + zero konczace
    char line[20];
    memcpy_P(line, PSTR("FLX?EOF?"), 8);
    ultoa(finalCrc32, line + 8, 10);
    return manager->sendOta(line, targetID);
}

RadioOta::RadioOta(RadioManager *manager) {
    this->manager = manager;
}

// true = trwa transfer OTA; na ten czas wstrzymujemy ruch testowy z loop()
bool RadioOta::isOtaInProgress() {
    return otaState != OtaState(WAITING_FOR_SERIAL_HANDSHAKE);
}

// Odpowiedzi HEX niosa numer pakietu, ktorego dotycza ("FLX?HEX?OK?134"). Dzieki temu
// spozniona retransmisja odpowiedzi na pakiet n nie zostanie zaliczona jako odpowiedz
// na pakiet n+1 (co konczylo sie rozjazdem numeracji i cofka przez FLX?HEX?WRONG_NUM).
// Odpowiedz bez numeru = starszy firmware odbiornika -> akceptujemy ja jak dawniej.
bool RadioOta::isResponseForCurrentHexPacket(const char *str, uint8_t prefixLength) {
    if (strlen(str) <= prefixLength || str[prefixLength] != '?') {
        return true;
    }
    long responseNumber = atol(str + prefixLength + 1);
    if (responseNumber == currentHexPacketNumber) {
        return true;
    }
    Serial.print(F("OTA | Ignoring HEX response for packet "));
    Serial.print(responseNumber);
    Serial.print(F(", waiting for "));
    Serial.println(currentHexPacketNumber);
    return false;
}

// Spozniona odpowiedz dla poprzedniego pakietu jest dowodem, ze target zyje i ze
// wlasnie odebral od nas radiowy auto-ACK (RadioManager potwierdza ramke, zanim
// warstwa OTA zdecyduje, czy jej trescią sie zajac). Nie moze wiec kosztowac nas
// proby wyslania biezacego pakietu - inaczej seria zgubionych ACK wyczerpalaby
// limit 3 prob i zrywala caly transfer, choc kanal caly czas dziala.
// Nie zapetli sie: target po swoich 3 probach przestaje powtarzac odpowiedz.
void RadioOta::noteStaleHexResponse() {
    hexSendTryes = 0;
}

void RadioOta::radioOtaDataReceived(char *str, uint8_t len, uint8_t senderId) {
    (void) len;
    if (DEBUG) Serial.print(F("OTA | radioOtaDataReceived: \""));
    if (DEBUG) Serial.print(str);
    if (DEBUG) Serial.print(F("\" from senderId: "));
    if (DEBUG) Serial.println(senderId);

    // Wzorce w PROGMEM: szesc Stringow tymczasowych na kazda odebrana ramke OTA to
    // szesc malloc/free w najciasniejszym momencie transferu - i fragmentacja sterty.
    const char *s = str;
    const uint8_t hexOkLen = 10;  // strlen("FLX?HEX?OK")
    const uint8_t hexErrLen = 11; // strlen("FLX?HEX?ERR")

    // Odpowiedzi akceptujemy takze w stanach SENDING_* (tuz po timeoucie okna):
    // spozniona odpowiedz jest nadal wazna, a odbiornik dostal juz radiowy auto-ACK
    // i przeszedl dalej - odrzucenie jej tutaj rozsynchronizowaloby oba wezly.
    if (strcmp_P(s, PSTR("FLX?OK")) == 0) {
        if (otaState == OtaState(WAITING_FOR_WIRELESS_HANDSHAKE_RESPONSE)
            || otaState == OtaState(SENDING_WIRELESS_HANDSHAKE)) {
            otaState = OtaState(WIRELESS_HANDSHAKE_RESPONSE_RECEIVED);
            Serial.println(F("OTA | Handshake response received"));
        }
    } else if (strncmp_P(s, PSTR("FLX?HEX?OK"), hexOkLen) == 0) {
        if (otaState == OtaState(WAITING_FOR_WIRELESS_HEX_RESPONSE)
            || otaState == OtaState(SENDING_WIRELESS_HEX)) {
            if (isResponseForCurrentHexPacket(str, hexOkLen)) {
                hexSendTryes = 0;
                Serial.println(F("FLX?HEX?OK")); // do Javy zawsze bez numeru - format serialu bez zmian
                hexDataFromSerialStartTime = millis();
                otaState = OtaState(WAITING_FOR_HEX_DATA_FROM_SERIAL);
                if (DEBUG) Serial.println(F("OTA | HEX response received"));
            } else {
                noteStaleHexResponse();
            }
        }
    } else if (strcmp_P(s, PSTR("FLX?EOF?OK")) == 0) {
        if (otaState == OtaState(WAITING_FOR_WIRELESS_EOF_RESPONSE)
            || otaState == OtaState(SENDING_WIRELESS_EOF)) {
            Serial.println(F("FLX?EOF?OK"));
            Serial.println(F("FLASH IMG TRANSMISSION SUCCESS"));
            resetStateAndValues();
            Serial.println(F("OTA | EOF response received"));
        }
    } else if (strncmp_P(s, PSTR("FLX?HEX?ERR"), hexErrLen) == 0) {
        if (otaState == OtaState(WAITING_FOR_WIRELESS_HEX_RESPONSE)
            || otaState == OtaState(SENDING_WIRELESS_HEX)) {
            if (isResponseForCurrentHexPacket(str, hexErrLen)) {
                hexSendTryes = 0;
                Serial.println(F("FLX?HEX?ERR")); // do Javy zawsze bez numeru - format serialu bez zmian
                hexDataFromSerialStartTime = millis();
                otaState = OtaState(WAITING_FOR_HEX_DATA_FROM_SERIAL);
                Serial.println(F("OTA | HEX response received, but CRC32 is wrong"));
            } else {
                noteStaleHexResponse();
            }
        }
    // WRONG_NUM celowo bez sprawdzania numeru: niesie numer OCZEKIWANY przez target,
    // a nie ten, ktory wlasnie wyslalismy. Duplikat zawsze niesie te sama wartosc
    // (oczekiwanie nie ruszy, dopoki pakiet nie zostanie zapisany), wiec co najwyzej
    // powtarza Javie te sama komende cofki - jest samonaprawialny.
    } else if (strncmp_P(s, PSTR("FLX?HEX?WRONG_NUM"), 17) == 0) {
        if (otaState == OtaState(WAITING_FOR_WIRELESS_HEX_RESPONSE)
            || otaState == OtaState(SENDING_WIRELESS_HEX)) {
            hexSendTryes = 0;
//            Serial.println(F("FLX?HEX?WRONG_NUM"));
            Serial.println(str);
            hexDataFromSerialStartTime = millis();
            otaState = OtaState(WAITING_FOR_HEX_DATA_FROM_SERIAL);
            Serial.println(F("OTA | HEX response received, but it has wrong number"));
        }
    } else if (strcmp_P(s, PSTR("FLX?EOF?ERR")) == 0) {
        if (otaState == OtaState(WAITING_FOR_WIRELESS_EOF_RESPONSE)
            || otaState == OtaState(SENDING_WIRELESS_EOF)) {
            Serial.println(F("FLX?EOF?ERR"));
            Serial.println(F("FLASH IMG TRANSMISSION FAIL"));
            resetStateAndValues();
            Serial.println(F("OTA | EOF response received, but CRC32 is wrong"));
        }
    }
}



//===================================================================================================================
// readSerialLine() - reads a line feed (\n) terminated line from the serial stream
// returns # of bytes read, up to 254
// timeout in ms, will timeout and return after so long
// this is called at the OTA programmer side
//===================================================================================================================
uint8_t RadioOta::readSerialLine(char *input, char endOfLineChar, uint8_t maxLength, uint16_t timeout) {
    uint8_t inputLen = 0;
    Serial.setTimeout(timeout);
    inputLen = Serial.readBytesUntil(endOfLineChar, input, maxLength);
    input[inputLen] = 0;//null-terminate it
    Serial.setTimeout(0);
    return inputLen;
}

void RadioOta::resetEEPROM() {
    Serial.println("Resetting EEPROM to default values...");
    CONFIG.PROGRAMMER_ID = PROGRAMMERID_DEFAULT;
    EEPROM.writeBlock(0, CONFIG);
}

void RadioOta::printSettings() {
    Serial << endl << F("PROGRAMMERID:") << CONFIG.PROGRAMMER_ID << endl;
}

void RadioOta::Blink(int DELAY_MS) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(LED_BUILTIN, LOW);
}

void RadioOta::resetStateAndValues() {
    handshakeTryes = 0;
    hexSendTryes = 0;
    eofSendTryes = 0;
    serialResendRequests = 0;
    currentHexPacketNumber = -1;
    otaState = OtaState(WAITING_FOR_SERIAL_HANDSHAKE);
}
