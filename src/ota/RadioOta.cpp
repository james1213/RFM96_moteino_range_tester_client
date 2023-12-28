//
// Created by LukaszLibront on 20.12.2023.
//

#include "RadioOta.h"

RadioOta::RadioOta(RadioManager* manager) {
    this->manager = manager;
}

void RadioOta::send(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK) {
    String strBuffer = String((const char*)buffer); //TODO czy ta zamiana zadziała?

    Serial.print(F("   #RADIO_OTA | send(), strBuffer = "));
    Serial.print(strBuffer);


//    String strBuffer = "";
//    for(int i = 0; i < bufferSize; i++) {
//        strBuffer += (char) buffer[i];
//    }

    manager->send(strBuffer, toAddress, requestACK, true, []() {
                      Serial.print(F("   #RADIO_OTA | ACK OK"));
                  },
                  [](String &payload) {
                      Serial.print(F("   #RADIO_OTA | ACK NOT OK, payload = "));
//                                  Serial.println(payload);
                  });

    //czekania aż wyśle (nadejdzie przerwanie TXDone) i zostanie przestawiony na tryb nasłuchu
    uint32_t now = millis();
    while (!manager->isTransmissionFinished() && millis() - now < RF69_CSMA_LIMIT_MS) {
        manager->radioLoop();
    }
}

//wysyłka danyhc i ack  w jednym
void RadioOta::sendACK(const void* buffer, uint8_t bufferSize) {
    Serial.println(F("RADIO_OTA | sendACK()"));

    //TODO które pierwsze wysłać?
    send(SENDERID, buffer, bufferSize);
//    manager->sendAck();
}

bool RadioOta::receiveDone() {
    if (manager->isHaveDate()) {
//        Serial.print(F("   #RADIO_OTA | isHaveDate()"));
        manager->setHaveData(false);
        SENDERID = manager->getSenderIdOfLastMessage();
//        Serial.print(F("   #RADIO_OTA | SENDERID = "));
        Serial.print(SENDERID);
        String receivedData = manager->getLastReceivedData();
//        Serial.print(F("   #RADIO_OTA | receivedData = "));
        Serial.print(receivedData);
        DATALEN = receivedData.length();
//        Serial.print(F("   #RADIO_OTA | DATALEN = "));
//        Serial.print(DATALEN);

//        receivedData.c_str();
//        receivedData.toCharArray((char*)DATA, RF69_MAX_DATA_LEN+1);


        byte buf[RF69_MAX_DATA_LEN+1];
        receivedData.getBytes(buf, RF69_MAX_DATA_LEN+1);


        for (int i = 0; i < DATALEN; i++) {
            DATA[i] = buf[i];
        }
        return true;
    }

    for (int i = 0; i < RF69_MAX_DATA_LEN+1; i++) {
        DATA[i] = 0;
    }
//    DATA[RF69_MAX_DATA_LEN+1]; // RX/TX payload buffer, including end of string NULL char
    DATALEN = 0;
    SENDERID = 0;
//    Serial.println(F("RADIO_OTA | kasowanie"));
    return false;
}




//tutaj będzie blokujące
//bool RadioOta::sendWithRetry(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime) {
//    Serial.println(F("RADIO_OTA | sendWithRetry()"));
//    uint32_t sentTime;
//
//    // todo While na całość
//    for (uint8_t i = 0; i < retries; i++)
//    {
//        send(toAddress, buffer, bufferSize, true);
//        sentTime = millis();
//        while (millis() - sentTime < retryWaitTime)
//        {
//            manager->radioLoop();
//            if (ACKReceived(toAddress)) {
//                Serial.println(F("RADIO_OTA | sendWithRetry(), ACK RECEIVED"));
//                return true;
//            }
//        }
//    }
//    Serial.println(F("RADIO_OTA | sendWithRetry(), NO ACK RECEIVED"));
//    return false;
//}

bool RadioOta::sendWithRetry(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime) {
    Serial.print(F("   #RADIO_OTA | sendWithRetry()"));

    // todo While na całość
    for (uint8_t i = 0; i < retries; i++)
    {
        send(toAddress, buffer, bufferSize, true);
        while (isWaitingForAck()) {
            manager->radioLoop();
        }
        if (ACKReceived(toAddress)) {
            Serial.print(F("   #RADIO_OTA | sendWithRetry(), ACK RECEIVED"));
            return true;
        }

    }
    Serial.print(F("   #RADIO_OTA | sendWithRetry(), NO ACK RECEIVED"));
    return false;
}

bool RadioOta::ACKReceived(uint16_t fromNodeID) {
    return manager->isAckReceived();
}

bool RadioOta::isWaitingForAck() {
    return manager->waitingForAck;
}

void RadioOta::setFrequency(uint32_t freqHz) {

}

uint32_t RadioOta::getFrequency() {
    return 0;
}

void RadioOta::radioLoop() {
    manager->radioLoop();
}
