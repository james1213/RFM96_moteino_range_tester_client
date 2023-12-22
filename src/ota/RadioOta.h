//
// Created by LukaszLibront on 20.12.2023.
//

#ifndef RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOOTA_H
#define RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOOTA_H

#define RF69_MAX_DATA_LEN       61
#define RFM69_ACK_TIMEOUT   30  // 30ms roundtrip req for 61byte packets
#define RF69_CSMA_LIMIT_MS 1000

#include <Arduino.h>            // assumes Arduino IDE v1.0 or greater
#include <SPI.h>
#include <SPIFlash.h>
#include <radiomanager/RadioManager.h>

class RadioOta {
public:

    RadioOta(RadioManager* manager);

    RadioManager* manager;

    uint8_t DATA[RF69_MAX_DATA_LEN + 1]; // RX/TX payload buffer, including end of string NULL char
    uint8_t DATALEN;
    uint16_t SENDERID;

    void send(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK= false);
    void sendACK(const void* buffer = "", uint8_t bufferSize= 0);
    bool receiveDone();
    bool sendWithRetry(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries= 2, uint8_t retryWaitTime= RFM69_ACK_TIMEOUT);
    bool ACKReceived(uint16_t fromNodeID);
    bool isWaitingForAck();
    uint32_t getFrequency();
    void setFrequency(uint32_t freqHz);

    void radioLoop();
};


#endif //RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOOTA_H
