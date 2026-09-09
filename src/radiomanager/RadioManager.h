//
// Created by LukaszLibront on 19.12.2023.
//

#ifndef RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOMANAGER_H
#define RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOMANAGER_H

#pragma once

#include <Arduino.h>
#include <LoRa.h>



#define LOG_ACTIVE false

// Adres rozgloszeniowy: ramka do 255 jest przyjmowana przez KAZDY wezel (mesh
// uzywa go do beaconow topologii). Broadcasty ida bez ACK - nie ma jednego adresata.
#define RADIO_BROADCAST_ID 255

// Moc nadawania RFM95/96: uzywamy wyjscia PA_BOOST, bo tylko ono jest podlaczone
// do anteny w modulach HopeRF (wyjscie RFO zostaje niepodlaczone - dalo by ~zero mocy).
// Powyzej 17 dBm uklad wchodzi w tryb wysokiej mocy (PA_DAC) i wymaga podniesienia
// progu OCP - biblioteka ustawia tam 140 mA, co przycina PA przy 20 dBm.
#define TX_POWER_MIN_DBM        2   // minimum dla PA_BOOST
#define TX_POWER_MAX_DBM        20  // maksimum SX1276 (tryb PA_DAC)
#define TX_POWER_FTDI_SAFE_DBM  10  // powyzej tego zasilanie z pinu 3V3 FTDI nie wyrabia
#define TX_OCP_HIGH_POWER_MA    150 // limit pradu PA dla trybu >17 dBm (Semtech 5.4.3)

// Nasluch kanalu przed nadaniem (CSMA na RSSI chwilowym). Podloga szumu SX1276
// przy SF7/125 kHz to ok. -110..-120 dBm; ramka sasiada z biurka to -20..-60 dBm.
#define CS_BUSY_RSSI_DBM  (-85)
#define CS_MAX_WAIT_MS    400

// ==================== BUFORY NADAWCZE O STALEJ POJEMNOSCI ====================
// Oba bufory sa rezerwowane RAZ, w konstruktorze (sterta jest wtedy pusta, wiec
// laduja na jej dnie) i nigdy nie rosna. Ramki sa skladane WPROST w nich
// (acquireTxBuffer/commitTxBuffer), wiec goraca sciezka nadawania nie robi zadnej
// alokacji. Wczesniej jedna wysylka testowa to byly 4-5 zywych kopii tresci
// (String w main, ramka mesh, sendBuffer, ackCallback_paylod, ramka radiowa w
// startSending - razem ok. 500 B chwilowo na 2 KB RAM) i przy 246 B wolnego
// mesh odmawial kazdej wysylki komunikatem "brak RAM na ramke danych".
// Najdluzsza ramka w systemie: OTA "<OTA>FLX?DAT?" + linia HEX (do 105) = 118 B.
#define RADIO_TX_BUFFER_CAPACITY   128
#define RADIO_ACK_BUFFER_CAPACITY  12   // "!255@-120" = 9 znakow

// ==================== AUTOMATYCZNA REGULACJA MOCY (APC) ====================
// Kazdy ACK niesie zwrotke "!<id>@<rssi>": RSSI, z jakim odbiorca uslyszal
// kwitowana ramke. Nadawca reguluje SWOJA moc tak, by u odbiorcy trafic w okno
// APC_TARGET_RSSI_DBM +- APC_HYSTERESIS_DB. Dwie niezalezne petle (po jednej na
// kierunek) - bez zalozenia symetrii lacza, wiec bez sprzezenia regulatorow.
// Stary parser ACK czyta "!<id>@<rssi>" poprawnie (toInt konczy na '@'),
// dzieki czemu mieszane wersje firmware wspolpracuja w oknie flashowania OTA.
#define APC_ENABLED           true
#define APC_TARGET_RSSI_DBM   (-85) // cel: tak ma nas slyszec druga strona
#define APC_HYSTERESIS_DB     5     // martwa strefa; RSSI i tak skacze o +-kilka dB
#define APC_STEP_DB           2     // krok pojedynczej korekty
#define APC_ACK_MISS_LIMIT    3     // tyle timeoutow ACK z rzedu -> skok na APC_MAX_DBM
// Sufit eskalacji. UWAGA na zasilanie: przy 3V3 z FTDI ustaw TX_POWER_FTDI_SAFE_DBM,
// inaczej automatyczny skok mocy przy slabym laczu wpedzi wezel w petle brown-outow.
#define APC_MAX_DBM           TX_POWER_MAX_DBM


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
    // Ostatnia nadana ramka z zadaniem ACK ZOSTAJE w sendBuffer az do rozstrzygniecia
    // transakcji (ACK / timeout) - to ona jest payloadem callbacku bledu i ponowien
    // skoku mesh (resendRetained). Zastapila osobna kopie ackCallback_paylod (~120 B
    // sterty na stale). retainedFrameValid gasnie, gdy ktos zajmie bufor nowa ramka.
    bool txPending = false;          // ramka w sendBuffer czeka na nadanie
    bool retainedFrameValid = false; // sendBuffer trzyma nadana ramke z zadaniem ACK

    void (*ackNotReceivedCallback)(String &payload);
    void (*ackReceivedCallback)();
    void (*dataReceivedCallback)(String &receivedText, uint8_t senderId);
    void (*otaDataReceivedCallback)(String &receivedText, uint8_t senderId);
    void (*meshDataReceivedCallback)(String &receivedText, uint8_t senderId) = nullptr;
    void (*anyFrameReceivedCallback)(uint8_t senderId) = nullptr; // kazda poprawna ramka (mesh: dowod zycia sasiada)
    void (*dataSentCallback)();

    String sendBuffer;    // pojemnosc RADIO_TX_BUFFER_CAPACITY, rezerwowana w konstruktorze
    String ackSendBuffer; // pojemnosc RADIO_ACK_BUFFER_CAPACITY, j.w.; niepusty = ACK czeka na nadanie
    uint8_t messageId = 0;
    uint8_t pendingAckMessageId = 0; // id wyslanej ramki DAT, do ktorej dopasowujemy "!id"
    uint8_t sendBufferDest = 0;      // adresat i flaga ACK zwiazane z konkretnym buforem,
    uint8_t ackSendBufferDest = 0;   // zeby zakolejkowanie ACK nie nadpisalo metadanych
    bool sendBufferAckReq = false;   // czekajacej wiadomosci DAT (i odwrotnie)
    int8_t sendBufferTxPwrOverride = -1; // wymuszona moc dla ramki w sendBuffer (-1 = regulowana)
    int8_t frameTxPwrOverride = -1;      // j.w., przekazywana do startSending przez sendLoop
    uint8_t destinationIdOfLastMessage = 0;
    uint8_t senderIdOfLastMessage = 0;
    uint8_t receivedMessageIdOfLastMessage = 0;
    volatile int receivedPacketSize = 0;
    int lastRssi = 0; // RSSI ostatnio odebranej ramki (dBm), lapany w readReceivedData

    // Stan APC. peerReportedRssi = ostatnie "slychac cie na X dBm" ze zwrotki w ACK.
    int peerReportedRssi = 0;
    bool peerRssiValid = false;  // false = zadna zwrotka jeszcze nie dotarla
    uint8_t ackMissStreak = 0;   // kolejne timeouty ACK; limit -> skok na pelna moc
    bool apcFrozen = false;      // OTA: moc przypieta do sufitu, regulator spi
    int8_t apcPendingDbm = -1;   // zadana moc czeka na przerwe miedzy ramkami (-1 = brak)
    int8_t apcMaxDbm = APC_MAX_DBM; // sufit eskalacji, konfigurowalny per wezel (setApcMaxPower)

    int8_t txPowerDbm = 0;
    bool needToSendAckToSender = false;
    bool sendAckAutomaticly = true; //TODO czyba powinno być na stałe na false, a potem ręcznie wysyłać sendACK

    // Byly tu String lastReceivedData i flaga _haveData z para getterow - alternatywne,
    // odpytywane API odbioru. Nikt z niego nie korzystal (oba projekty uzywaja callbacku
    // onDataReceived), a lastReceivedData trzymal na stercie KOPIE kazdej odebranej
    // wiadomosci - przy 64-bajtowych pakietach ~115 B na stale plus jedna dodatkowa
    // pelna kopia w szczycie parsowania ramki. Usuniete.

//    int receivedBytes[256];

    RadioManager();

    // Skladanie ramki wprost w buforze nadawczym (bez kopii):
    //   String *f = acquireTxBuffer();      // nullptr = radio zajete (ramka czeka albo trwa transakcja ACK)
    //   if (!txBufferFits(n)) releaseTxBuffer(); else { *f = ...; commitTxBuffer(...); }
    // Bufor ma STALA pojemnosc - przed skladaniem sprawdz txBufferFits, bo String
    // po przekroczeniu pojemnosci realokowalby bufor (albo po cichu uciął ramke).
    String *acquireTxBuffer();
    static bool txBufferFits(unsigned int bytes) { return bytes <= RADIO_TX_BUFFER_CAPACITY; }
    bool commitTxBuffer(uint8_t address, bool ackRequested,
                        void (*_ackReceivedCallback)() = nullptr,
                        void (*_ackNotReceivedCallback)(String &payload) = nullptr,
                        int8_t txPowerDbmOverride = -1); // >= 0: wymuszona moc tej jednej ramki (beacony)
    void releaseTxBuffer();               // wolajacy zrezygnowal po acquire (np. za dlugi payload)
    bool hasRetainedFrame();              // nadana ramka z zadaniem ACK wciaz lezy w buforze, slot wolny
    String &retainedFrame();              // jej tresc (pusty String, gdy juz jej nie ma)
    bool resendRetained(uint8_t address, void (*_ackReceivedCallback)(),
                        void (*_ackNotReceivedCallback)(String &payload)); // ponowienie bez kopii

    // Malowanie stosu: wolny obszar miedzy sterta a stosem dostaje znany wzor przy
    // starcie; minStackGap() = najdluzszy nietkniety pas wzoru = najmniejszy zapas,
    // jaki KIEDYKOLWIEK wystapil (stos zaciera wzor od gory, sterta chwilowa od dolu).
    static void paintFreeStack();         // wolac jako pierwsza instrukcje setup()
    static uint16_t minStackGap();

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
    bool isAckPayload(const String &str);
    bool isAckPayloadAndValidMessageId(const String &str);
    void waitForAckTimeoutLoop();
    void txStuckWatchdogLoop();
    static int freeRam();
    // Wygodne opakowania: kopiuja tresc do bufora nadawczego (jedna kopia, bez malloc).
    bool sendOta(String &str, uint8_t address, void (*_ackReceivedCallback)() = nullptr, void (*_ackNotReceivedCallback)(String &payload) = nullptr);
    bool sendTagged(String &taggedPayload, uint8_t address, void (*_ackReceivedCallback)() = nullptr, void (*_ackNotReceivedCallback)(String &payload) = nullptr);
    bool send(String &str, uint8_t address, void (*_ackReceivedCallback)() = nullptr, void (*_ackNotReceivedCallback)(String &payload) = nullptr);
    bool startSending(String &str, uint8_t address, bool ackRequested);
    unsigned long lastRamErrorMillis = 0;
    // Nasluch kanalu przed nadaniem (CSMA): jesli RSSI chwilowe przekracza prog,
    // ktos wlasnie nadaje - odkladamy ramke o obieg petli, najdluzej CS_MAX_WAIT_MS.
    unsigned long csBusySinceMillis = 0;
    void LoRa_txMode();
    void sendAck();
    void setSendAckAutomaticly(bool value);
    bool isAckReceived();
    uint8_t getSenderIdOfLastMessage();
    int getLastRssi();
    bool isTransmissionFinished();
    void setupRadio(long frequency, int ss, int reset, int dio0, uint8_t _nodeId, void(*receiveDoneCallback)(int), void(*txDoneCallback)());
    int8_t setTxPower(int8_t dbm);       // zwraca moc faktycznie ustawiona (po ograniczeniu do zakresu)
    int8_t getTxPower();
    void apcOnAckPayload(const String &ackPayload); // zwrotka RSSI z ACK -> krok regulatora
    void apcOnAckTimeout();                         // licznik strat ACK -> eskalacja mocy
    void apcRequestPower(int8_t dbm);               // zadanie zmiany, aplikowane miedzy ramkami
    void setApcFrozen(bool frozen);                 // true na czas OTA: sufit mocy + stop regulacji
    void setApcMaxPower(int8_t dbm);                // sufit eskalacji (FTDI: TX_POWER_FTDI_SAFE_DBM!)
    int8_t getEffectiveTxPower();                   // moc, z jaka wyjdzie NASTEPNA ramka
    uint16_t getTxCurrentEstimate_mA();  // szacunkowy pobor pradu radia w czasie nadawania
    void printTxPower();
    void printRadioDiag(); // jedna linia: tryb radia, DIO0, IRQ, EIMSK, flagi, RAM
    void dumpRegisters();
    void onOtaDataReceived(void (*callback)(String &, uint8_t));
    void onMeshDataReceived(void (*callback)(String &, uint8_t));
    void onAnyFrameReceived(void (*callback)(uint8_t senderId));
    bool isOtaPayload(String &str);
    bool isMeshPayload(String &str);
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
    bool sendWithTag(const __FlashStringHelper *tag, const String &body, uint8_t address,
                     void (*_ackReceivedCallback)(), void (*_ackNotReceivedCallback)(String &payload));
    bool sendDirectly(String &str, uint8_t address, bool ackRequested = false,
                      void (*_ackReceivedCallback)() = nullptr,
                      void (*_ackNotReceivedCallback)(String &payload) = nullptr);
};


#endif //RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOMANAGER_H
