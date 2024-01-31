//
// Created by LukaszLibront on 29.12.2023.
//

#ifndef RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOOTA_H
#define RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOOTA_H

#define RADIO_OTA_LOG_ACTIVE false

#include <Arduino.h>
#include <radiomanager/RadioManager.h>
#include "arduino_base64.hpp"
#include <CRC32.h>

////////////////////////////////Serial
#include <Streaming.h>  //easy C++ style output operators: http://arduiniana.org/libraries/streaming/
#include <EEPROMex.h>   //http://playground.arduino.cc/Code/EEPROMex
///////////////////



#define NODE_ID_TO_SEND 0x02


///////////////////////////////Serial
#define NODEID_DEFAULT     1023
#define NETWORKID_DEFAULT  100
#define FREQUENCY_DEFAULT  433000000
//#define FREQUENCY_DEFAULT  915000000
#define VALID_FREQUENCY(freq) ((freq >= 430000000 && freq <= 435000000) || (freq >= 860000000 && freq <= 870000000) || (freq >= 902000000 && freq <= 928000000))
#define ENCRYPTKEY_DEFAULT ""
//*********************************************************************************************
#define DEBUG_MODE  true  //'true' = verbose output from programming sequence, ~12% slower OTA!
#define SERIAL_BAUD 115200
#define ACK_TIME    50  // # of ms to wait for an ack
//#define ACK_TIME    5000  // # of ms to wait for an ack
#define TIMEOUT     3000
//#define TIMEOUT     60000


#define SHIFTCHANNEL


#ifndef DEFAULT_TIMEOUT
#define DEFAULT_TIMEOUT 3000
#endif

#ifndef ACK_TIMEOUT
#define ACK_TIMEOUT 20
#endif
/////////////////////////////////



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

    unsigned long eofSendStartTime = 0;
    uint8_t eofSendTryes = 0;
    const uint8_t EOF_SENDING_TRYES_LIMIT = 3;

    unsigned long hexDataFromSerialStartTime = 0;

    uint32_t finalCrc32 = 0x4A17B156; // TODO obliczać ją rzeczywistą na podstawie odczytu z flasha


    enum OtaState {
        WAITING_FOR_SERIAL_HANDSHAKE,
        SENDING_WIRELESS_HANDSHAKE,
        WAITING_FOR_WIRELESS_HANDSHAKE_RESPONSE,
        WIRELESS_HANDSHAKE_RESPONSE_RECEIVED,
        WAITING_FOR_HEX_DATA_FROM_SERIAL,
        SENDING_WIRELESS_HEX,
        SENDING_WIRELESS_EOF,
        WAITING_FOR_WIRELESS_HEX_RESPONSE,
        WAITING_FOR_WIRELESS_EOF_RESPONSE
    };

//    OtaState otaState = OtaState(SENDING_WIRELESS_HANDSHAKE);
    OtaState otaState = OtaState(WAITING_FOR_SERIAL_HANDSHAKE);







    /////////////////////////////////
    struct config {
        uint8_t NETWORKID;
        uint16_t NODEID;
        uint32_t FREQUENCY;
        uint8_t BR300KBPS;
        char ENCRYPTKEY[17]; //16+nullptr
    } CONFIG;

    char _input[128];
    char c = 0;
    uint16_t targetID=0;


    String serialReceivedBuffer = "";

    uint8_t readSerialLine(char* input, char endOfLineChar=10, uint8_t maxLength=115, uint16_t timeout=1000);
    boolean resetEEPROMCondition();
    void resetEEPROM();
    void printSettings();
    void Blink(int DELAY_MS);
    void serialCheckForHandshakeRequest(uint8_t* input, uint8_t inputLen);
    uint8_t handleSerialHEXDataWrapper(uint16_t targetID, uint16_t timeout,
                                       uint16_t ACKTIMEOUT, uint8_t
                                       DEBUG);
    uint8_t handleSerialHEXData(uint16_t targetID, uint16_t timeout, uint16_t ACKTIMEOUT,
                                uint8_t debug);
    uint8_t validateHEXData(void* data, uint8_t length);
    uint8_t prepareSendBuffer(char* hexdata, uint8_t*buf, uint8_t length, uint16_t seq);
    uint8_t byteFromHex(char msb, char lsb);

    void resetStateAndValues();
public:

    RadioOta(RadioManager *manager);

    void loop();
//    void radioSendHex();
    void radioSendHandshake();
    void radioSendEof();
    void radioOtaDataReceived(String &str, uint8_t senderId);

    ///////////////////////////////////////////////

    void serialSendHandshakeResponse(uint8_t *input, uint8_t inputLen, uint16_t targetID, uint16_t timeout, uint16_t ackTimeout,
                                     uint8_t debug);

    bool isHexEofMessage(String &str);

    void radioSendHexFromSerial();
};


#endif //RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOOTA_H
