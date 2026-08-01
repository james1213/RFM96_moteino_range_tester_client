//
// Created by LukaszLibront on 19.12.2023.
//

#ifndef RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOMANAGER_H
#define RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOMANAGER_H

#pragma once

#include <Arduino.h>
#include <LoRa.h>



#define LOG_ACTIVE false

// Moc nadawania RFM95/96: uzywamy wyjscia PA_BOOST, bo tylko ono jest podlaczone
// do anteny w modulach HopeRF (wyjscie RFO zostaje niepodlaczone - dalo by ~zero mocy).
// Powyzej 17 dBm uklad wchodzi w tryb wysokiej mocy (PA_DAC) i wymaga podniesienia
// progu OCP - biblioteka ustawia tam 140 mA, co przycina PA przy 20 dBm.
#define TX_POWER_MIN_DBM        2   // minimum dla PA_BOOST
#define TX_POWER_MAX_DBM        20  // maksimum SX1276 (tryb PA_DAC)
#define TX_POWER_FTDI_SAFE_DBM  10  // powyzej tego zasilanie z pinu 3V3 FTDI nie wyrabia
#define TX_OCP_HIGH_POWER_MA    150 // limit pradu PA dla trybu >17 dBm (Semtech 5.4.3)


class RadioManager {
public:
    uint8_t nodeId = 0;
    volatile bool transmissionFinished = true;
    volatile bool receivedFlag = false;
    volatile bool zeroLengthPacketReceived = false;
    unsigned long sendingTime = 0;
    volatile unsigned long txDoneTime = 0;
    unsigned long txStartMillis = 0;
    unsigned long txStuckTimeout = 2000; // ms; >> najdluzszy czas ramki w powietrzu
    volatile bool transmissionClenedUp = true;
    volatile bool ackReceived = false;
    bool waitingForAck = false;
    bool ackFramePendingTx = false; // ramka z zadaniem ACK zakolejkowana, ale jeszcze nie nadana
    unsigned long waitForAckStartTime = 0;
    unsigned long ackTimeout = 1000; //ms
    String ackCallback_paylod = "";

    void (*ackNotReceivedCallback)(String &payload);
    void (*ackReceivedCallback)();
    void (*dataReceivedCallback)(String &receivedText, uint8_t senderId);
    void (*otaDataReceivedCallback)(String &receivedText, uint8_t senderId);
    void (*dataSentCallback)();

    String sendBuffer = "";
    String ackSendBuffer = "";
    uint8_t messageId = 0;
    uint8_t pendingAckMessageId = 0; // id wyslanej ramki DAT, do ktorej dopasowujemy "!id"
    uint8_t sendBufferDest = 0;      // adresat i flaga ACK zwiazane z konkretnym buforem,
    uint8_t ackSendBufferDest = 0;   // zeby zakolejkowanie ACK nie nadpisalo metadanych
    bool sendBufferAckReq = false;   // czekajacej wiadomosci DAT (i odwrotnie)
    uint8_t destinationIdOfLastMessage = 0;
    uint8_t senderIdOfLastMessage = 0;
    uint8_t receivedMessageIdOfLastMessage = 0;
    volatile int receivedPacketSize = 0;

    int8_t txPowerDbm = 0;
    bool needToSendAckToSender = false;
    bool sendAckAutomaticly = true; //TODO czyba powinno być na stałe na false, a potem ręcznie wysyłać sendACK

    // Byly tu String lastReceivedData i flaga _haveData z para getterow - alternatywne,
    // odpytywane API odbioru. Nikt z niego nie korzystal (oba projekty uzywaja callbacku
    // onDataReceived), a lastReceivedData trzymal na stercie KOPIE kazdej odebranej
    // wiadomosci - przy 64-bajtowych pakietach ~115 B na stale plus jedna dodatkowa
    // pelna kopia w szczycie parsowania ramki. Usuniete.

//    int receivedBytes[256];

    void onDataReceived(void(*callback)(String &receivedText, uint8_t senderId));
    void onDataSent(void(*callback)());
    virtual void receiveDone(int packetSize);
    virtual void txDone();
    void LoRa_rxMode();
    void loop();
    void sendLoop();
    void receiveLoop();
    String readReceivedData();
    void extractMessageIdAndSenderIdAndDestinationIdFromReceivedData(String &str);
    int splitString(String &text, String *texts, char ch, int maxArrayLength);
    bool isAckPayload(const String &str);
    bool isAckPayloadAndValidMessageId(String str);
    void waitForAckTimeoutLoop();
    void txStuckWatchdogLoop();
    static int freeRam();
    bool buildTaggedPayload(String &out, const char *tag, const String &body);
    bool sendOta(String &str, uint8_t address, void (*_ackReceivedCallback)() = nullptr, void (*_ackNotReceivedCallback)(String &payload) = nullptr);
    bool sendTagged(String &taggedPayload, uint8_t address, void (*_ackReceivedCallback)() = nullptr, void (*_ackNotReceivedCallback)(String &payload) = nullptr);
    bool send(String &str, uint8_t address, void (*_ackReceivedCallback)() = nullptr, void (*_ackNotReceivedCallback)(String &payload) = nullptr);
    bool startSending(String &str, uint8_t address, bool ackRequested);
    unsigned long lastRamErrorMillis = 0;
    void LoRa_sendMessage(const String &message);
    void LoRa_txMode();
    void sendAck();
    void setSendAckAutomaticly(bool value);
    bool isAckReceived();
    uint8_t getSenderIdOfLastMessage();
    bool isTransmissionFinished();
    void setupRadio(long frequency, int ss, int reset, int dio0, uint8_t _nodeId, void(*receiveDoneCallback)(int), void(*txDoneCallback)());
    int8_t setTxPower(int8_t dbm);       // zwraca moc faktycznie ustawiona (po ograniczeniu do zakresu)
    int8_t getTxPower();
    uint16_t getTxCurrentEstimate_mA();  // szacunkowy pobor pradu radia w czasie nadawania
    void printTxPower();
    void dumpRegisters();
    void onOtaDataReceived(void (*callback)(String &, uint8_t));
    bool isOtaPayload(String &str);
    bool isDataPayload(String &str);
    int getReceivedPacketSize();
    bool isNeedToSendAckToSender();
    bool isDataSent();

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

private:
    bool sendDirectly(String &str, uint8_t address, bool ackRequested= false, void (*_ackReceivedCallback)() = nullptr, void (*_ackNotReceivedCallback)(String &payload) = nullptr, bool useAckBuffer = false);
};


#endif //RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOMANAGER_H
