//
// Created by LukaszLibront on 29.12.2023.
//

#include "RadioOta.h"


void RadioOta::otaLoop() {
    if (otaState == OtaState(SENDING_HANDSHAKE)) { //TODO sprawdzenie czy komp coś wysyłą
        Serial.println(F("OTA | state = SENDING_HANDSHAKE"));
        handshakeTryes++;
        sendHandshake();
        handshakeSendStartTime = millis();
        Serial.println(F("OTA | going to WAITING_FOR_HANDSHAKE_RESPONSE state"));
        otaState = OtaState(WAITING_FOR_HANDSHAKE_RESPONSE);
    } else if (otaState == OtaState(WAITING_FOR_HANDSHAKE_RESPONSE)) {
        if (handshakeTryes >= HANDSHAKE_SENDING_TRYES_LIMIT) {
            otaState = OtaState(WAITING_FOR_START);
            //TODO - wyświetlenie błędu w apce Javovej
            Serial.println(F("OTA | Handshake response not received, retries number exceeded. Not sending again"));
        } else if (millis() - handshakeSendStartTime > 1000) {
            otaState = OtaState(SENDING_HANDSHAKE);
            Serial.println(F("OTA | Trying to send handshake again"));
        }
    } else if (otaState == OtaState(HANDSHAKE_RESPONSE_RECEIVED)) {
        Serial.println(F("OTA | state = HANDSHAKE_RESPONSE_RECEIVED"));
        handshakeTryes = 0;
        hexSendingIndex = 0;
        hexSendTryes = 0;
        otaState = OtaState(SENDING_HEX);
    } else if (otaState == OtaState(SENDING_HEX)) {
        Serial.println(F("OTA | state = SENDING_HEX"));
        hexSendTryes++;
        sendHex();
        hexSendStartTime = millis();
        Serial.println(F("OTA | going to WAITING_FOR_HEX_RESPONSE state"));
        otaState = OtaState(WAITING_FOR_HEX_RESPONSE);
    } else if (otaState == OtaState(WAITING_FOR_HEX_RESPONSE)) {
        if (hexSendTryes >= HEX_SENDING_TRYES_LIMIT) {
            otaState = OtaState(WAITING_FOR_START);
            //TODO - wyświetlenie błędu w apce Javovej
            Serial.println(F("OTA | Handshake response not received, retries number exceeded. Not sending again"));
        } else if (millis() - hexSendStartTime > 1000) {
            //timeout pojedynczej próby wysłania hexa
            otaState = OtaState(SENDING_HEX);
            Serial.println(F("OTA | Trying to send hex again"));
        }
    } else if (otaState == OtaState(HEX_RESPONSE_RECEIVED)) {
        Serial.println(F("OTA | state = HEX_RESPONSE_RECEIVED"));
        hexSendTryes = 0;
        hexSendingIndex++;
        if (hexSendingIndex == HEX_SIZE) {
            otaState = OtaState(SENDING_EOF);
            //TODO wysłanie EOF
        } else {
            otaState = OtaState(SENDING_HEX);
        }
    } else if (otaState == OtaState(SENDING_EOF)) {
        Serial.println(F("OTA | state = SENDING_EOF"));
        eofSendTryes++;
        sendEof();
        eofSendStartTime = millis();
        Serial.println(F("OTA | going to WAITING_FOR_EOF_RESPONSE state"));
        otaState = OtaState(WAITING_FOR_EOF_RESPONSE);
    } else if (otaState == OtaState(WAITING_FOR_EOF_RESPONSE)) {
        if (eofSendTryes >= EOF_SENDING_TRYES_LIMIT) {
            otaState = OtaState(WAITING_FOR_START);
            //TODO - wyświetlenie błędu w apce Javovej
            Serial.println(F("OTA | EOF response not received, retries number exceeded. Not sending again"));
        } else if (millis() - eofSendStartTime > 1000) {
            otaState = OtaState(SENDING_EOF);
            Serial.println(F("OTA | Trying to send EOF again"));
        }
    } else if (otaState == OtaState(EOF_RESPONSE_RECEIVED)) {
        Serial.println(F("OTA | state = EOF_RESPONSE_RECEIVED"));
        eofSendTryes = 0;
        otaState = OtaState(WAITING_FOR_START);
        Serial.println(F("OTA | state = WAITING_FOR_START"));
    }
}


void RadioOta::sendHex() {
    auto inputLength = sizeof(source[hexSendingIndex]);
    char output[base64::encodeLength(inputLength)];
    base64::encode(source[hexSendingIndex], inputLength, output);

    Serial.println(F("OTA | Calculating checksum"));
    String sourceStr = String(output);
    Serial.print(F("OTA | sourceStr = "));
    Serial.println(sourceStr);
    uint32_t checksum = CRC32::calculate(sourceStr.c_str(), sourceStr.length());
    Serial.print(F("OTA | checksum = "));
    Serial.println(checksum);
    sourceStr += "?" + String(checksum);
    Serial.print(F("OTA | sourceStr+checksum "));
    Serial.println(sourceStr);

    Serial.print(F("OTA | SENDING_HEX: ["));
    Serial.print(sourceStr);
    Serial.println(F("]"));

    manager->sendOta(sourceStr, NODE_ID_TO_SEND, false);

    delay(2000);
}

void RadioOta::sendHandshake() {
    String handshakeStr = "FLX?";
    manager->sendOta(handshakeStr, NODE_ID_TO_SEND, false);
}

void RadioOta::sendEof() {
    String handshakeStr = "EOF?" + String(finalCrc32);
    manager->sendOta(handshakeStr, NODE_ID_TO_SEND, false);
}

RadioOta::RadioOta(RadioManager *manager) {
    this->manager = manager;
}

void RadioOta::otaDataReceived(String &str, uint8_t senderId) {
    Serial.print(F("OTA | otaDataReceived: \""));
    Serial.print(str);
    Serial.print(F("\" from senderId: "));
    Serial.println(senderId);

    String handshakeResponse = "FLX?OK";
    String hexResponse = "HEX?OK";
    String hexWrongCrc32Response = "HEX?ERR";
    String eofResponse = "EOF?OK"; //TODO czy czekać na sprawdzenie CRC32  całego pliku? Nie wiem jak długo to potrwa.
    // TODO może lepiej wysłać EOF?OK i dopiero po tym sprawdzać CRC32 całego pliku? i na końću wysłać EOF?CRC32 lub EOF?CRC32?OK
    String eofWrongCrc32Response = "EOF?CRC32";

    if (str.equals(handshakeResponse)) {
        if (otaState == OtaState(WAITING_FOR_HANDSHAKE_RESPONSE)) {
            otaState = OtaState(HANDSHAKE_RESPONSE_RECEIVED);
            Serial.println(F("OTA | Handshake response received"));
        }
    } else if (str.equals(hexResponse)) {
        if (otaState == OtaState(WAITING_FOR_HEX_RESPONSE)) {
            otaState = OtaState(HEX_RESPONSE_RECEIVED);
            Serial.println(F("OTA | HEX response received"));
        }
    } else if (str.equals(eofResponse)) {
        if (otaState == OtaState(WAITING_FOR_EOF_RESPONSE)) {
            otaState = OtaState(EOF_RESPONSE_RECEIVED);
            Serial.println(F("OTA | EOF response received"));
        }
    } else if (str.equals(hexWrongCrc32Response)) {
        if (otaState == OtaState(WAITING_FOR_HEX_RESPONSE)) {
            otaState = OtaState(SENDING_HEX);
            Serial.println(F("OTA | HEX response received, but CRC32 is wrong"));
        }
    } else if (str.equals(eofWrongCrc32Response)) {
        if (otaState == OtaState(WAITING_FOR_EOF_RESPONSE)) {
            otaState = OtaState(EOF_RESPONSE_RECEIVED);
            //TODO - wyświetlenie błędu w apce Javovej
            Serial.println(F("OTA | EOF response received, but CRC32 is wrong"));
        }
    }
}
