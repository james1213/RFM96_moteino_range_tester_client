#include <Arduino.h>
#include <radiomanager/RadioManager.h>
#include <ota/RadioOta.h>
#include <ota/RFM95_OTA.h>
#include <SPIFlash.h>

#define NODE_ID 0x01
#define NODE_ID_BROADCAST 0xFF
#define NODE_ID_TO_SEND 0x02


int count = 0;

//RadioOta radioOta;
SPIFlash flash(SS_FLASHMEM, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

RadioManager *manager = new RadioManager();
//RadioOta *radioOta = new RadioOta(manager);

boolean runEvery(unsigned long interval);

//uint8_t source[3][10] = {{2,5,4,8,5,4,7,9,2, 7},
//                         {8,1,3,5,2,7,1,8,0, 0},
//                         {9,2,2,3,1,3,6,3,6, 1}};
//byte destination[3][10];
//
//bool otaSend = false;

void setupSerial();

void setupRadio();

void setupFlash();

void dataReceived(String &str, uint8_t senderId);

//void otaLoop();

//bool sendHandshake();

//void otaLoop() {
//    if (!otaSend) { //TODO sprawdzenie czy komp coś wysyłą
//        otaSend = true;
//        Serial.println(F("OTA | Sending..."));
//        sendHandshake();
//    }
//}

//void sendHandshake() {
//    String handshakeStr = "<OTA>FLX?";
//    manager->send(handshakeStr, NODE_ID_TO_SEND, true, true,
//                  []() {
//                      Serial.println(F("OTA | OK"));
//                  },
//                  [](String &payload) {
//                      Serial.print(F("OTA | NOT OK, payload = "));
//                      Serial.println(payload);
//                  });
//}

void setup() {
    setupSerial();
    Serial.println(F("ver. 1.1"));
    setupRadio();
    setupFlash();
}

void setupRadio() {
    manager->onDataReceived(dataReceived);
//    manager->onOtaDataReceived();

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

    //non-blocking
//    if (!getOtaInProgress()) {




        if (runEvery(2000)) { // repeat every 1000 millis
            Serial.println();

            String str = "Hello World [#" + String(count++) + "] with ACK";
            Serial.print(F("Sending payload: \""));
            Serial.print(str);
            Serial.println(F("\""));
            manager->send(str, NODE_ID_TO_SEND, true, true,
                              []() {
                                  Serial.println(F("MAIN | OK"));
                              },
                              [](String &payload) {
                                  Serial.print(F("MAIN | NOT OK, payload = "));
                                  Serial.println(payload);
                              });
        }



//    }
//    if (radioOta->receiveDone()) {
//        CheckForWirelessHEX(*radioOta, flash, false);
//    }

//blocking
//    if (runEvery(2000)) {
//
//        String str = "Hello World [#" + String(count++) + "] with ACK";
////    uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK= false
////        radioOta->send(NODE_ID_TO_SEND, str.c_str(), str.length(), true);
//        radioOta->sendWithRetry(NODE_ID_TO_SEND, str.c_str(), str.length(), 2);
//    }
//
//    if (radioOta->receiveDone()) {
//        Serial.println(F("MAIN | receiveDone()"));
//        Serial.print(F("MAIN | SENDERID = "));
//        Serial.println(radioOta->SENDERID);
//        Serial.print(F("MAIN | DATALEN = "));
//        Serial.println(radioOta->DATALEN);
//        Serial.print(F("MAIN | DATA = "));
//        Serial.println((char*)radioOta->DATA);
//    }


//    otaLoop();
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

