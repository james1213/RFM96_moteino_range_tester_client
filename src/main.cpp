//#define digitalPinToInterrupt(p)  ((p) == 10 ? 0 : ((p) == 11 ? 1 : ((p) == 2 ? 2 : NOT_AN_INTERRUPT)))
#include <Arduino.h>
#include <radiomanager/RadioManager.h>
#include <SPIFlash.h>
//#include "arduino_base64.hpp"
#include "ota/RadioOta.h"
//#include <CRC32.h>

#define NODE_ID 0x01


int count = 0;

SPIFlash flash(SS_FLASHMEM, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

RadioManager *manager = new RadioManager();
RadioOta *radioOta = new RadioOta(manager);

boolean runEvery(unsigned long interval);


void setupSerial();

void setupRadio();

void setupFlash();

void dataReceived(String &str, uint8_t senderId);

void setup() {
    setupSerial();
    Serial.println(F("ver. 1.1"));
    setupRadio();
    setupFlash();
    delay(5000);
}

void setupRadio() {
    manager->onDataReceived(dataReceived);
//    manager->onOtaDataReceived(radioOtaDataReceived);
    manager->onOtaDataReceived([](String &str, uint8_t senderId){
        radioOta->radioOtaDataReceived(str, senderId);
    });

    manager->onDataSent([]() {
//        Serial.println(F("MAIN | data sent"));
    });

    manager->setupRadio(433E6, 10, 7, 2, NODE_ID,
                        [](int packetSize) {
                            manager->receiveDone(packetSize);
                        },
                        []() {
                            manager->txDone();
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
    manager->loop();

//    if (runEvery(2000)) { // repeat every 1000 millis
//        Serial.println();
//
//        String str = "Hello World [#" + String(count++) + "] with ACK | test string 1234567890ABCDEFGHIJKLMNOP";
//        Serial.print(F("Sending payload: \""));
//        Serial.print(str);
//        Serial.println(F("\""));
//        manager->send(str, 2,
//                      []() {
//                          Serial.println(F("MAIN | OK"));
//                      },
//                      [](String &payload) {
//                          Serial.print(F("MAIN | NOT OK, payload = "));
//                          Serial.println(payload);
//                      });
//    }

    radioOta->loop();
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

