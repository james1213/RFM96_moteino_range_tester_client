//
// Created by LukaszLibront on 29.12.2023.
//

#ifndef RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOOTA_H
#define RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOOTA_H

#include <Arduino.h>
#include <radiomanager/RadioManager.h>
#include "arduino_base64.hpp"
#include <CRC32.h>

#define NODE_ID_TO_SEND 0x02

class RadioOta {
private:
    RadioManager *manager;

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

public:

    RadioOta(RadioManager *manager);

    void otaLoop();
    void sendHex();
    void sendHandshake();
    void sendEof();
    void otaDataReceived(String &str, uint8_t senderId);
};


#endif //RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOOTA_H
