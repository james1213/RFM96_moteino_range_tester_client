//
// Created by LukaszLibront on 19.12.2023.
//

#ifndef RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOMANAGER_H
#define RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOMANAGER_H

#pragma once

#include <Arduino.h>
#include <LoRa.h>

#define NODE_ID 0x01

//volatile bool radioManager_transmissionFinished;
//volatile bool radioManager_receivedFlag;
//unsigned long radioManager_sendingTime;
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


void radioManager_onDataReceived(void(*callback)(String &receivedText, uint8_t senderId));
void radioManager_onDataSent(void(*callback)());

void radioManager_setupRadio();
void radioManager_onReceiveDone(int packetSize);
void radioManager_onTxDone();
void radioManager_LoRa_rxMode();
void radioManager_radioLoop();
void radioManager_sendLoop();
void radioManager_receiveLoop();
String radioManager_readReceivedData();
void radioManager_extractMessageIdAndSenderIdAndDestinationIdFromReceivedData(String &str);
int splitString(String &text, String *texts, char ch);
bool isAckPayload(String str);
bool isAckPayloadAndValidMessageId(String str);
//void dataReceived(const String &str);
void waitForAckTimeoutLoop();
void bufferedSendAndWaitForAck(String &str, uint8_t address, void (*_ackReceivedCallback)(),
                               void (*_ackNotReceivedCallback)(String &payload));
void bufferedSend(String &str, uint8_t address);
void send(String &str, uint8_t address);
void LoRa_sendMessage(String message);
void LoRa_txMode();

void radioManager_logln(const __FlashStringHelper *ifsh);
void radioManager_log(const __FlashStringHelper *ifsh);
void radioManager_logln(const String &s);
void radioManager_log(const String &s);
void radioManager_logln(unsigned char b, int base = 10);
void radioManager_log(unsigned char b, int base = 10);
void radioManager_logln();
void radioManager_logln(int n, int base = 10);
void radioManager_log(int n, int base = 10);
void radioManager_logln(double n, int digits = 2);
void radioManager_log(double n, int digits = 2);
void radioManager_logln(long n, int base = 10);
void radioManager_log(long n, int base = 10);


#endif //RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOMANAGER_H
