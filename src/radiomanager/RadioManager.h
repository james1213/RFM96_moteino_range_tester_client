//
// Created by LukaszLibront on 19.12.2023.
//

#ifndef RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOMANAGER_H
#define RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOMANAGER_H

#pragma once

#include <Arduino.h>
#include <LoRa.h>

//#define NODE_ID 0x01

//volatile bool transmissionFinished;
//volatile bool receivedFlag;
//unsigned long sendingTime;
//volatile bool transmissionClenedUp;
//volatile bool ackReceived;
//bool waitingForAck;
//unsigned long waitForAckStartTime;
//unsigned long ackTimeout;
//String ackCallback_paylod;
//
////void (*ackNotReceivedCallback)(String &payload);
////
////void (*ackReceivedCallback)();
//
//String sendBuffer;
//uint8_t messageId;
//uint8_t destinationAddress;
//uint8_t destinationIdOfLastMessage;
//uint8_t senderIdOfLastMessage;
//uint8_t receivedMessageIdOfLastMessage;




class RadioManager {
public:
    uint8_t nodeId = 0;
    bool logActive = false;
    volatile bool transmissionFinished = true;
    volatile bool receivedFlag = false;
    unsigned long sendingTime = 0;
    volatile bool transmissionClenedUp = true;
    volatile bool ackReceived = false;
    bool waitingForAck = false;
    unsigned long waitForAckStartTime = 0;
    unsigned long ackTimeout = 1000; //ms
    String ackCallback_paylod = "";

    void (*ackNotReceivedCallback)(String &payload);
    void (*ackReceivedCallback)();
    void (*dataReceivedCallback)(String &receivedText, uint8_t senderId);
    void (*dataSentCallback)();

    String sendBuffer = "";
    uint8_t messageId = 0;
    uint8_t destinationAddress = 0;
    uint8_t destinationIdOfLastMessage = 0;
    uint8_t senderIdOfLastMessage = 0;
    uint8_t receivedMessageIdOfLastMessage = 0;
    String lastReceivedData;
    int receivedPacketSize = 0;

    bool _ackRequested = false;
    bool needToSendAckToSender = false;
    bool sendAckAutomaticly = true; //TODO czyba powinno być na stałe na false, a potem ręcznie wysyłać sendACK
    volatile bool _haveData;


    void onDataReceived(void(*callback)(String &receivedText, uint8_t senderId));
    void onDataSent(void(*callback)());

//    void setupRadio();
    virtual void onReceiveDone(int packetSize);
    virtual void onTxDone();
    void LoRa_rxMode();
    void radioLoop();
    void sendLoop();
    void receiveLoop();
    String readReceivedData();
    void extractMessageIdAndSenderIdAndDestinationIdFromReceivedData(String &str);
    int splitString(String &text, String *texts, char ch, int maxArrayLength);
    bool isAckPayload(String str);
    bool isAckPayloadAndValidMessageId(String str);
//void dataReceived(const String &str);
    void waitForAckTimeoutLoop();
//void bufferedSendAndWaitForAck(String &str, uint8_t address, void (*_ackReceivedCallback)(),void (*_ackNotReceivedCallback)(String &payload));
    void send(String &str, uint8_t address, bool ackRequested= false, bool _sendAckAutomaticly= true, void (*_ackReceivedCallback)() = nullptr, void (*_ackNotReceivedCallback)(String &payload) = nullptr);
    void startSending(String &str, uint8_t address);
    void LoRa_sendMessage(String message);
    void LoRa_txMode();
    void sendAck();
    void setSendAckAutomaticly(bool value);
    bool isAckReceived();
    uint8_t getSenderIdOfLastMessage();
    String getLastReceivedData();
    bool isHaveDate();
    bool setHaveData(bool value);
    bool isTransmissionFinished();
    void setupRadio(uint8_t _nodeId, void(*onReceiveDoneCallback)(int), void(*onTxDoneCallback)());
    bool isNeedToSendAckToSender();
    int getReceivedPacketSize();

    void DEBUGlogln(const __FlashStringHelper *ifsh);
    void DEBUGlog(const __FlashStringHelper *ifsh);
    void DEBUGlogln(const String &s);
    void DEBUGlog(const String &s);
    void DEBUGlogln(unsigned char b, int base = 10);
    void DEBUGlog(unsigned char b, int base = 10);
    void DEBUGlogln();
    void DEBUGlogln(int n, int base = 10);
    void DEBUGlog(int n, int base = 10);
    void DEBUGlogln(double n, int digits = 2);
    void DEBUGlog(double n, int digits = 2);
    void DEBUGlogln(long n, int base = 10);
    void DEBUGlog(long n, int base = 10);

    void dumpRegisters();
};


#endif //RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOMANAGER_H
