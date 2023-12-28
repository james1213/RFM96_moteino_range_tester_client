#include <Arduino.h>
#include <radiomanager/RadioManager.h>
#include <SPIFlash.h>
#include "arduino_base64.hpp"
#include <CRC32.h>

#define NODE_ID 0x01
#define NODE_ID_TO_SEND 0x02


int count = 0;

SPIFlash flash(SS_FLASHMEM, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

RadioManager *manager = new RadioManager();

boolean runEvery(unsigned long interval);

#define HEX_SIZE 6

uint8_t source[HEX_SIZE][10] = {{0x17, 0x77, 0x3B, 0x11, 0x82, 0xA4, 0xC4, 0xC8, 0xFF, 0x2B},
                         {0x14, 0x77, 0x3B, 0x11, 0x82, 0xA4, 0xC4, 0xC8, 0xFF, 0x3B},
                         {0x11, 0x77, 0x3B, 0x11, 0x82, 0xA4, 0xC4, 0xC8, 0xFF, 0x1F},
                         {0x4, 0x77, 0x3B, 0x11, 0x82, 0xA4, 0xC4, 0xC8, 0xFF, 0x6F},
                         {0x7, 0x77, 0x3B, 0x11, 0x82, 0xA4, 0xC4, 0xC8, 0xFF, 0x2C},
                         {0x15, 0x77, 0x3B, 0x11, 0x82, 0xA4, 0xC4, 0xC8, 0xFF, 0x4D}};

unsigned long handshakeSendStartTime = 0;
uint8_t handshakeTryes = 0;
const uint8_t HANDSHAKE_SENDING_TRYES_LIMIT = 3;

unsigned long hexSendStartTime = 0;
uint8_t hexSendTryes = 0;
const uint8_t HEX_SENDING_TRYES_LIMIT = 3;
int hexSendingIndex = 0;

unsigned long eofSendStartTime = 0;
uint8_t eofSendTryes = 0;
const uint8_t EOF_SENDING_TRYES_LIMIT = 3;

uint32_t finalCrc32 = 0x4A17B156; // TODO obliczać ją rzeczywistą na podstawie odczytu z flasha

enum OtaState {
    WAITING_FOR_START,
    SENDING_HANDSHAKE,
    WAITING_FOR_HANDSHAKE_RESPONSE,
    HANDSHAKE_RESPONSE_RECEIVED,
    SENDING_HEX,
    WAITING_FOR_HEX_RESPONSE,
    HEX_RESPONSE_RECEIVED,
    SENDING_EOF,
    WAITING_FOR_EOF_RESPONSE,
    EOF_RESPONSE_RECEIVED
};

OtaState otaState = OtaState(SENDING_HANDSHAKE);

void setupSerial();

void setupRadio();

void setupFlash();

void dataReceived(String &str, uint8_t senderId);

void otaLoop();

void sendHandshake();

void sendHex();

void sendEof();

void otaLoop() {
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

void sendHex() {
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
    manager->sendOta(sourceStr, NODE_ID_TO_SEND, true, true,
                  []() {
                      Serial.println(F("OTA | HEX ACK received"));
                  },
                  [](String &payload) {
                      Serial.println(F("OTA | HEX ACK not received"));
                  });
    delay(2000);
}

void sendHandshake() {
//    String handshakeStr = "<OTA>FLX?";
    String handshakeStr = "FLX?";
    manager->sendOta(handshakeStr, NODE_ID_TO_SEND, true, true,
                  []() {
                      Serial.println(F("OTA | handshake ACK received"));
                  },
                  [](String &payload) {
                      Serial.println(F("OTA | handshake ACK not received"));
//                      handshakeReceived = false;
                  });
}

void sendEof() {
    String handshakeStr = "EOF?" + String(finalCrc32);
    manager->sendOta(handshakeStr, NODE_ID_TO_SEND, true, true,
                  []() {
                      Serial.println(F("OTA | EOF ACK received"));
                  },
                  [](String &payload) {
                      Serial.println(F("OTA | EOF ACK not received"));
//                      handshakeReceived = false;
                  });
}

void otaDataReceived(String &str, uint8_t senderId) {
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

void setup() {
    setupSerial();
    Serial.println(F("ver. 1.1"));
    setupRadio();
    setupFlash();
    delay(5000);
}

void setupRadio() {
    manager->onDataReceived(dataReceived);
    manager->onOtaDataReceived(otaDataReceived);

    manager->onDataSent([]() {
//        Serial.println(F("MAIN | data sent"));
    });

    manager->setupRadio(NODE_ID,
                        [](int packetSize) {
                            manager->onReceiveDone(packetSize);
                        },
                        []() {
                            manager->onTxDone();
                        });

    manager->dumpRegisters();
}

void setupFlash() {
    Serial.println(F("[FLASH] Setup started"));
    if (flash.initialize()) {
        Serial.println(F("[FLASH] SPI Flash Init OK"));
        Serial.print(F("[FLASH] UniqueID (MAC): "));
        flash.readUniqueId();
        for (byte i = 0; i < 8; i++) {
            Serial.print(flash.UNIQUEID[i], HEX);
            Serial.print(':');
        }
        Serial.println();

        char flashBuff[50];
        sprintf(flashBuff, "[FLASH] DeviceID: 0x%X", flash.readDeviceId());
        Serial.println(flashBuff);
        Serial.println(F("[FLASH] Setup finished"));
    } else {
        Serial.println(F("[FLASH] SPI Flash MEM not found (is chip soldered?)..."));
    }
}

void setupSerial() {
    Serial.begin(115200);
    while (!Serial);
}


void loop() {
    manager->radioLoop();

//    if (runEvery(2000)) { // repeat every 1000 millis
//        Serial.println();
//
//        String str = "Hello World [#" + String(count++) + "] with ACK";
//        Serial.print(F("Sending payload: \""));
//        Serial.print(str);
//        Serial.println(F("\""));
//        manager->send(str, NODE_ID_TO_SEND, true, true,
//                      []() {
//                          Serial.println(F("MAIN | OK"));
//                      },
//                      [](String &payload) {
//                          Serial.print(F("MAIN | NOT OK, payload = "));
//                          Serial.println(payload);
//                      });
//    }

    otaLoop();
}

void dataReceived(String &str, uint8_t senderId) {
    Serial.print(F("MAIN | Received data: \""));
    Serial.print(str);
    Serial.print(F("\" from senderId: "));
    Serial.println(senderId);
}


boolean runEvery(unsigned long interval) {
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        return true;
    }
    return false;
}

