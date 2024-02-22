//
// Created by LukaszLibront on 29.12.2023.
//

#include "RadioOta.h"


#define DEBUG false


//TODO numerowanie pakietó i sprawdzaniae czy przysZedł kolejny, czy są brakulub powtórzenia
//TODO więcej logów

void RadioOta::loop() {
if (otaState == OtaState(SENDING_WIRELESS_HANDSHAKE)) {
        Serial.println(F("OTA | state = SENDING_WIRELESS_HANDSHAKE"));
        handshakeTryes++;
        if (handshakeTryes >= HANDSHAKE_SENDING_TRYES_LIMIT) {
            Serial.println(F("FLX?HANDSHAKE?TIMEOUT"));
            resetStateAndValues();
            Serial.println(F("OTA | Handshake response not received, retries number exceeded. Not sending again"));
        } else {
            radioSendHandshake();
            handshakeSendStartTime = millis();
            Serial.println(F("OTA | going to WAITING_FOR_WIRELESS_HANDSHAKE_RESPONSE state"));
            otaState = OtaState(WAITING_FOR_WIRELESS_HANDSHAKE_RESPONSE);
        }
    } else if (otaState == OtaState(WAITING_FOR_WIRELESS_HANDSHAKE_RESPONSE)) {
        if (millis() - handshakeSendStartTime > 1000) {
            otaState = OtaState(SENDING_WIRELESS_HANDSHAKE);
            Serial.println(F("OTA | Trying to send handshake again"));
        }
    } else if (otaState == OtaState(WIRELESS_HANDSHAKE_RESPONSE_RECEIVED)) {
        Serial.println(F("OTA | state = WIRELESS_HANDSHAKE_RESPONSE_RECEIVED"));
        Serial.println(F("FLX?HANDSHAKE?OK"));
        hexDataFromSerialStartTime = millis();
        otaState = OtaState(WAITING_FOR_HEX_DATA_FROM_SERIAL);
        Serial.println(F("OTA | going to WAITING_FOR_HEX_DATA_FROM_SERIAL state"));
    } else if (otaState == OtaState(WAITING_FOR_HEX_DATA_FROM_SERIAL)) {
        if (millis() - hexDataFromSerialStartTime > 1000) {
            Serial.println(F("FLX?HEX?SERIAL_TIMEOUT"));
            resetStateAndValues();
            Serial.println(F("OTA | Hex from serial not received, timeout reached. Not waiting anymore"));
        }
    } else if (otaState == OtaState(SENDING_WIRELESS_HEX)) {
        if (DEBUG) Serial.println(F("OTA | state = SENDING_WIRELESS_HEX"));
        hexSendTryes++;
        if (hexSendTryes >= HEX_SENDING_TRYES_LIMIT) {
            Serial.println(F("FLX?HEX?WIRELESS_TIMEOUT"));
            resetStateAndValues();
            Serial.println(F("OTA | FLASH IMG TRANSMISSION FAIL"));
            Serial.println(F("OTA | Handshake response not received, retries number exceeded. Not sending again"));
        } else {
            radioSendHexFromSerial();
            hexSendStartTime = millis();
            if (DEBUG) Serial.println(F("OTA | going to WAITING_FOR_WIRELESS_HEX_RESPONSE state"));
            otaState = OtaState(WAITING_FOR_WIRELESS_HEX_RESPONSE);
        }
    } else if (otaState == OtaState(WAITING_FOR_WIRELESS_HEX_RESPONSE)) {
        if (millis() - hexSendStartTime > 1000) {
            otaState = OtaState(SENDING_WIRELESS_HEX);
            Serial.println(F("OTA | Trying to send hex again"));
        }
    } else if (otaState == OtaState(SENDING_WIRELESS_EOF)) {
        Serial.println(F("OTA | state = SENDING_WIRELESS_EOF"));
        eofSendTryes++;
        if (eofSendTryes >= EOF_SENDING_TRYES_LIMIT) {
            Serial.println(F("FLX?EOF?WIRELESS_TIMEOUT"));
            resetStateAndValues();
            Serial.println(F("OTA | FLASH IMG TRANSMISSION FAIL"));
            Serial.println(F("OTA | EOF response not received, retries number exceeded. Not sending again"));
        } else {
            radioSendEof();
            eofSendStartTime = millis();
            Serial.println(F("OTA | going to WAITING_FOR_WIRELESS_EOF_RESPONSE state"));
            otaState = OtaState(WAITING_FOR_WIRELESS_EOF_RESPONSE);
        }
    } else if (otaState == OtaState(WAITING_FOR_WIRELESS_EOF_RESPONSE)) {
        if (millis() - eofSendStartTime > 10000) { //aż 10 sek bo srawdzenie całego pliku może dlużej potrwać (sprawdzić doświadczalnei ile trwa)
            otaState = OtaState(SENDING_WIRELESS_EOF);
            Serial.println(F("OTA | Trying to send EOF again"));
        }
    }


    if (Serial.available()) {
        byte inputLen = readSerialLine(_input, 10, 64, 100);
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
                    targetID = String(_input).substring(7).toInt();
                    otaState = (OtaState(SENDING_WIRELESS_HANDSHAKE));
                }
            } else if (inputLen > 8 && _input[0] == 'F' && _input[1] == 'L' && _input[2] == 'X' && _input[3] == '?' && _input[4] == 'H' && _input[5] == 'E' && _input[6] == 'X' && _input[7] == '?') {
                if (otaState == OtaState(WAITING_FOR_HEX_DATA_FROM_SERIAL)) {
                    serialReceivedBuffer = String(_input).substring(8);
                    if (DEBUG) Serial.print(F("OTA | serialReceivedBuffer = "));
                    if (DEBUG) Serial.println(serialReceivedBuffer);
                    otaState = OtaState(SENDING_WIRELESS_HEX);
                }
            } else if (inputLen > 8 && _input[0] == 'F' && _input[1] == 'L' && _input[2] == 'X' && _input[3] == '?' && _input[4] == 'E' && _input[5] == 'O' && _input[6] == 'F' && _input[7] == '?') {
                if (otaState == OtaState(WAITING_FOR_HEX_DATA_FROM_SERIAL)) {
                    finalCrc32 = String(_input).substring(8).toInt();
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

void RadioOta::radioSendHexFromSerial() {
    if (DEBUG) Serial.print(F("OTA | radioSendHexFromSerial(), data = "));
    String dataToSend = "FLX?DAT?" + serialReceivedBuffer;
    if (DEBUG) Serial.println(dataToSend);
    manager->sendOta(dataToSend, targetID);
}

void RadioOta::radioSendHandshake() {
    String handshakeStr = "FLX?";
    manager->sendOta(handshakeStr, targetID);
}

void RadioOta::radioSendEof() {
    String handshakeStr = "FLX?EOF?" + String(finalCrc32);
    manager->sendOta(handshakeStr, targetID);
}

RadioOta::RadioOta(RadioManager *manager) {
    this->manager = manager;
}

void RadioOta::radioOtaDataReceived(String &str, uint8_t senderId) {
    if (DEBUG) Serial.print(F("OTA | radioOtaDataReceived: \""));
    if (DEBUG) Serial.print(str);
    if (DEBUG) Serial.print(F("\" from senderId: "));
    if (DEBUG) Serial.println(senderId);

    String handshakeResponse = "FLX?OK";
    String hexOkResponse = "FLX?HEX?OK";
    String hexErrResponse = "FLX?HEX?ERR";
    String hexWrongNumResponse = "FLX?HEX?WRONG_NUM";
    String eofOkResponse = "FLX?EOF?OK";
    String eofErrResponse = "FLX?EOF?ERR";

    if (str.equals(handshakeResponse)) {
        if (otaState == OtaState(WAITING_FOR_WIRELESS_HANDSHAKE_RESPONSE)) {
            otaState = OtaState(WIRELESS_HANDSHAKE_RESPONSE_RECEIVED);
            Serial.println(F("OTA | Handshake response received"));
        }
    } else if (str.equals(hexOkResponse)) {
        if (otaState == OtaState(WAITING_FOR_WIRELESS_HEX_RESPONSE)) {
            hexSendTryes = 0;
            Serial.println(F("FLX?HEX?OK"));
            hexDataFromSerialStartTime = millis();
            otaState = OtaState(WAITING_FOR_HEX_DATA_FROM_SERIAL);
            if (DEBUG) Serial.println(F("OTA | HEX response received"));
        }
    } else if (str.equals(eofOkResponse)) {
        if (otaState == OtaState(WAITING_FOR_WIRELESS_EOF_RESPONSE)) {
            Serial.println(F("FLX?EOF?OK"));
            Serial.println(F("FLASH IMG TRANSMISSION SUCCESS"));
            resetStateAndValues();
            Serial.println(F("OTA | EOF response received"));
        }
    } else if (str.equals(hexErrResponse)) {
        if (otaState == OtaState(WAITING_FOR_WIRELESS_HEX_RESPONSE)) {
            hexSendTryes = 0;
            Serial.println(F("FLX?HEX?ERR"));
            hexDataFromSerialStartTime = millis();
            otaState = OtaState(WAITING_FOR_HEX_DATA_FROM_SERIAL);
            Serial.println(F("OTA | HEX response received, but CRC32 is wrong"));
        }
    } else if (str.startsWith(hexWrongNumResponse)) {
        if (otaState == OtaState(WAITING_FOR_WIRELESS_HEX_RESPONSE)) {
            hexSendTryes = 0;
//            Serial.println(F("FLX?HEX?WRONG_NUM"));
            Serial.println(str);
            hexDataFromSerialStartTime = millis();
            otaState = OtaState(WAITING_FOR_HEX_DATA_FROM_SERIAL);
            Serial.println(F("OTA | HEX response received, but it has wrong number"));
        }
    } else if (str.equals(eofErrResponse)) {
        if (otaState == OtaState(WAITING_FOR_WIRELESS_EOF_RESPONSE)) {
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
    otaState = OtaState(WAITING_FOR_SERIAL_HANDSHAKE);
}
