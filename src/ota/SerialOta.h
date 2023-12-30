//
// Created by LukaszLibront on 29.12.2023.
//

#ifndef RFM96_MOTEINO_RANGE_TESTER_CLIENT_SERIALOTA_H
#define RFM96_MOTEINO_RANGE_TESTER_CLIENT_SERIALOTA_H

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

#include <Arduino.h>
#include <radiomanager/RadioManager.h>
#include <Streaming.h>  //easy C++ style output operators: http://arduiniana.org/libraries/streaming/
#include <EEPROMex.h>   //http://playground.arduino.cc/Code/EEPROMex
#include "ota/RadioOta.h"

class SerialOta {
private:
    struct config {
        uint8_t NETWORKID;
        uint16_t NODEID;
        uint32_t FREQUENCY;
        uint8_t BR300KBPS;
        char ENCRYPTKEY[17]; //16+nullptr
    } CONFIG;

    RadioManager *manager;
    RadioOta *radioOta;
    char _input[64];
    char c = 0;
    uint16_t targetID=0;

    uint8_t readSerialLine(char* input, char endOfLineChar=10, uint8_t maxLength=115, uint16_t timeout=1000);
    boolean resetEEPROMCondition();
    void resetEEPROM();
    void printSettings();
    void Blink(int DELAY_MS);
    void checkForSerialHandshake(uint8_t* input, uint8_t inputLen, uint16_t targetID, uint16_t timeout=DEFAULT_TIMEOUT, uint16_t ackTimeout=ACK_TIMEOUT, uint8_t debug=false);
    uint8_t handleSerialHEXDataWrapper(uint16_t targetID, uint16_t timeout,
                                       uint16_t ACKTIMEOUT, uint8_t
                                                  DEBUG);
    uint8_t handleSerialHEXData(uint16_t targetID, uint16_t timeout, uint16_t ACKTIMEOUT,
                                uint8_t debug);
    uint8_t validateHEXData(void* data, uint8_t length);
    uint8_t prepareSendBuffer(char* hexdata, uint8_t*buf, uint8_t length, uint16_t seq);
    uint8_t BYTEfromHEX(char MSB, char LSB);
public:
    void loop();
    SerialOta(RadioManager *manager, RadioOta *radioOta);

    void handleHexData(uint8_t *input, uint8_t inputLen, uint16_t targetID, uint16_t timeout, uint16_t ackTimeout,
                       uint8_t debug);
};


#endif //RFM96_MOTEINO_RANGE_TESTER_CLIENT_SERIALOTA_H
