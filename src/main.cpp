#include <Arduino.h>
#include <radiomanager/RadioManager.h>

//#define NODE_ID 0x01
#define NODE_ID_BROADCAST 0xFF
#define NODE_ID_TO_SEND 0x02


int count = 0;


boolean runEvery(unsigned long interval);
void setupSerial();
void setupRadio();
void dataReceived(String &str, uint8_t senderId);

void setup() {
    setupSerial();
    setupRadio();
}

void setupRadio() {
    radioManager_onDataReceived(dataReceived);
    radioManager_onDataSent([]() {
//        Serial.println(F("MAIN | data sent"));
    });
    radioManager_setupRadio();
}

void setupSerial() {
    Serial.begin(115200);
    while (!Serial);
}


void loop() {
    radioManager_radioLoop();
    if (runEvery(2000)) { // repeat every 1000 millis
        Serial.println();

        String str = "Hello World [#" + String(count++) + "] with ACK";
        Serial.print(F("Sending payload: \""));
        Serial.print(str);
        Serial.println(F("\""));
        bufferedSendAndWaitForAck(str, NODE_ID_TO_SEND,
//        bufferedSendAndWaitForAckWithWaitingUntilPreviousAckIsReceived(str,
                                  []() {
                                      Serial.println(F("MAIN | OK"));
                                  },
                                  [](String &payload) {
                                      Serial.print(F("MAIN | NOT OK, payload = "));
                                      Serial.println(payload);
                                  });
    }
}

void dataReceived(String &str, uint8_t senderId) {
    Serial.print(F("MAIN | Received data: \""));
    Serial.print(str);
    Serial.print(F("\""));
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

