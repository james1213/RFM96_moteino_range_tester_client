#include <Arduino.h>
#include <radiomanager/RadioManager.h>
#include <ota/RadioOta.h>
#include <ota/RFM95_OTA.h>
#include <SPIFlash.h>
#include "arduino_base64.hpp"

#define NODE_ID 0x01
#define NODE_ID_BROADCAST 0xFF
#define NODE_ID_TO_SEND 0x02


int count = 0;

//RadioOta radioOta;
SPIFlash flash(SS_FLASHMEM, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

RadioManager *manager = new RadioManager();
//RadioOta *radioOta = new RadioOta(manager);

boolean runEvery(unsigned long interval);

#define HEX_SIZE 3

uint8_t source[3][10] = {{0x17, 0x77, 0x3B, 0x11, 0x82, 0xA4, 0xC4, 0xC8, 0xFF, 0x2B},
                         {0x14, 0x77, 0x3B, 0x11, 0x82, 0xA4, 0xC4, 0xC8, 0xFF, 0x3B},
                         {0x15, 0x77, 0x3B, 0x11, 0x82, 0xA4, 0xC4, 0xC8, 0xFF, 0x4D}};

bool handshakeReceived = true;
bool waitingForSendHex = false;
unsigned long handshakeSendStartTime = 0;
uint8_t handshakeTryes = 0;
int hexSendingIndex = 0;
bool hexAckReceived = false;
uint8_t hexSendTryes = 0;
const uint8_t HEX_SENDING_TRYES = 3;
const uint8_t HANDSHAKE_SENDING_TRYES = 3;

enum OtaState {
    WAITING_FOR_START,
    SENDING_HANDSHAKE,
    WAITING_FOR_HANDSHAKE_RESPONSE,
    HANDSHAKE_RESPONSE_RECEIVED,
    SENDING_HEX,
    WAITING_FOR_HEX_ACK,
    WAITING_FOR_HEX_RESPONSE,
    HEX_RESPONSE_RECEIVED
};

//OtaState otaState = OtaState(WAITING_FOR_START);
OtaState otaState = OtaState(SENDING_HANDSHAKE);

void setupSerial();

void setupRadio();

void setupFlash();

void dataReceived(String &str, uint8_t senderId);

void otaLoop();

void sendHandshake();

void otaLoop() {
    if (otaState == OtaState(SENDING_HANDSHAKE)) { //TODO sprawdzenie czy komp coś wysyłą
        Serial.println(F("OTA | SENDING_HANDSHAKE"));
        handshakeTryes++;
        sendHandshake();
        handshakeSendStartTime = millis();
        otaState = OtaState(WAITING_FOR_HANDSHAKE_RESPONSE);
    } else if (otaState == OtaState(WAITING_FOR_HANDSHAKE_RESPONSE)) {
        if (handshakeTryes >= HANDSHAKE_SENDING_TRYES) {
            otaState = OtaState(WAITING_FOR_START);
            Serial.println(F("OTA | Handshake response not received, retries number exceeded. Not sending again"));
        } else if (millis() - handshakeSendStartTime > 1000) {
            otaState = OtaState(SENDING_HANDSHAKE);
            Serial.println(F("OTA | Trying to send handshake again"));
        }
    } else if (otaState == OtaState(HANDSHAKE_RESPONSE_RECEIVED)) {
        Serial.println(F("OTA | HANDSHAKE_RESPONSE_RECEIVED"));
        handshakeTryes = 0;
        hexSendingIndex = 0;
        hexSendTryes = 0;
        otaState = OtaState(SENDING_HEX);
    } else if (otaState == OtaState(SENDING_HEX)) {

        auto inputLength = sizeof(source[hexSendingIndex]);
        char output[base64::encodeLength(inputLength)];
        base64::encode(source[hexSendingIndex], inputLength, output);

        String sourceStr = "<OTA>" + String(output);

        Serial.print(F("OTA | SENDING_HEX: ["));
        Serial.print(sourceStr);
        Serial.println(F("]"));
        manager->send(sourceStr, NODE_ID_TO_SEND, true, true,
                      []() {
                          Serial.println(F("OTA | HEX ACK received"));
                          hexSendTryes = 0;
                          hexSendingIndex++;
                          if (hexSendingIndex == HEX_SIZE) {
                              otaState = OtaState(WAITING_FOR_START);
                              Serial.println(F("OTA | FINISHED"));
                          } else {
                              otaState = OtaState(SENDING_HEX);
                          }
                      },
                      [](String &payload) {
                          Serial.println(F("OTA | HEX ACK not received"));
                          hexSendTryes++;
                          if (hexSendTryes >= HEX_SENDING_TRYES) {
                              otaState = OtaState(WAITING_FOR_START);
                              Serial.println(F("OTA | HEX ACK not received, retries number exceeded. Not sending again"));
                          } else {
                              otaState = OtaState(SENDING_HEX);
                          }
                      });
        otaState = OtaState(WAITING_FOR_HEX_ACK);
        delay(2000);
    }
}

void sendHandshake() {
    String handshakeStr = "<OTA>FLX?";
    manager->send(handshakeStr, NODE_ID_TO_SEND, true, true,
                  []() {
                      Serial.println(F("OTA | handshake ACK received"));
                  },
                  [](String &payload) {
                      Serial.println(F("OTA | handshake ACK not received"));
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
    if (str.equals(handshakeResponse)) {
        otaState = OtaState(HANDSHAKE_RESPONSE_RECEIVED);
        Serial.println(F("OTA | Handshake response received"));
    } else if (str.equals(hexResponse)) {
//        otaState = OtaState(HEX_RESPONSE_RECEIVED);
        Serial.println(F("OTA | HEX response received"));
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

//    manager->setSendAckAutomaticly(false);
//    sendAckAutomaticly = false;

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

