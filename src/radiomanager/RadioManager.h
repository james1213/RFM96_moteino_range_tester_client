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

// ==================== BINARNY FORMAT RAMKI ====================
//
// Naglowek ma 4 BAJTY zamiast dawnych ~20 znakow tekstu ("255@255@255@1@<MSH>"
// plus konczacy apostrof). Zysk jest potrojny: krotsza ramka to krotszy czas w
// eterze (mniej kolizji przy 1-sekundowej kadencji), zero skladania i parsowania
// liczb w locie, i - najwazniejsze na 2 KB RAM - zaden bufor tekstowy nie musi
// juz powstac, zeby ramka mogla poleciec.
//
//   bajt 0: adresat        (1..254, 255 = broadcast)
//   bajt 1: nadawca        (1..254)
//   bajt 2: id wiadomosci  (1..255, nigdy 0 - 0 znaczy "brak transakcji")
//   bajt 3: 0xA0 | ACK_REQ | typ
//   bajt 4..: tresc
//
// Gorne trzy bity bajtu 3 to staly znacznik protokolu. Kosztuja zero (i tak by
// sie marnowaly), a pozwalaja odrzucic smiec i ramke starego, tekstowego
// firmware ZANIM cokolwiek z niej odczytamy. Dlugosc niesie naglowek LoRa,
// wiec ramka nie potrzebuje juz zadnego ogranicznika na koncu.
#define RADIO_HEADER_SIZE      4
#define RADIO_PROTO_MARK       0xA0 // 0b101 na bitach 5-7
#define RADIO_PROTO_MASK       0xE0
#define RADIO_FLAG_ACK_REQ     0x10
#define RADIO_TYPE_MASK        0x0F
#define RADIO_TYPE_APP         0    // tresc aplikacji wprost (dawne "<DAT>")
#define RADIO_TYPE_OTA         1    // transfer firmware (dawne "<OTA>")
#define RADIO_TYPE_MESH        2    // routing (dawne "<MSH>")
#define RADIO_TYPE_ACK         3    // potwierdzenie: [id potwierdzanej][RSSI int8]

// Najdluzsza tresc w systemie to pakiet OTA: "FLX?DAT?" + linia z PC (do 105
// znakow) = 113 B. 120 zostawia zapas na dluzsze pakiety bez ruszania buforow.
#define RADIO_PAYLOAD_CAPACITY 120

// Potwierdzenie niesie id kwitowanej ramki i RSSI, z jakim ja uslyszelismy
// (zwrotka dla regulatora mocy) - dwa bajty zamiast dawnego "!<id>@<rssi>".
#define RADIO_ACK_PAYLOAD_SIZE 2

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

// ==================== AUTOMATYCZNA REGULACJA MOCY (APC) ====================
// Kazdy ACK niesie RSSI, z jakim odbiorca uslyszal kwitowana ramke. Nadawca
// reguluje SWOJA moc tak, by u odbiorcy trafic w okno APC_TARGET_RSSI_DBM +-
// APC_HYSTERESIS_DB. Dwie niezalezne petle (po jednej na kierunek) - bez
// zalozenia symetrii lacza, wiec bez sprzezenia regulatorow.
#define APC_ENABLED           true
#define APC_TARGET_RSSI_DBM   (-85) // cel: tak ma nas slyszec druga strona
#define APC_HYSTERESIS_DB     5     // martwa strefa; RSSI i tak skacze o +-kilka dB
#define APC_STEP_DB           2     // krok pojedynczej korekty
#define APC_ACK_MISS_LIMIT    3     // tyle timeoutow ACK z rzedu -> sonda mocy w gore
// Sufit eskalacji. UWAGA na zasilanie: przy 3V3 z FTDI ustaw TX_POWER_FTDI_SAFE_DBM,
// inaczej automatyczny skok mocy przy slabym laczu wpedzi wezel w petle brown-outow.
#define APC_MAX_DBM           TX_POWER_MAX_DBM

// Callback bledu dostaje ramke, ktora nie doczekala sie ACK - to wciaz ta sama
// pamiec w buforze nadawczym, wiec nie ma tu zadnej kopii.
typedef void (*RadioFailCallback)(const uint8_t *payload, uint8_t len);
// Tresci tekstowe (aplikacja, OTA) sa w buforze odbiorczym zakonczone zerem,
// wiec odbiorca moze ich uzywac jak zwyklego C-stringa.
typedef void (*RadioTextCallback)(char *payload, uint8_t len, uint8_t senderId);
typedef void (*RadioBytesCallback)(uint8_t *payload, uint8_t len, uint8_t senderId);


class RadioManager {
public:
    RadioManager();

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

    // ==================== BUFORY ====================
    // Obie tablice sa STATYCZNE: zajmuja tyle samo miejsca co dawne Stringi na
    // stercie, ale nie moga sie nie udac ani jej pofragmentowac. Ramka jest
    // skladana wprost w txPayload, a naglowek dopisywany dopiero przy nadaniu -
    // wiec goraca sciezka nadawania nie wykonuje ANI JEDNEJ alokacji.
    uint8_t txPayload[RADIO_PAYLOAD_CAPACITY];
    uint8_t txLength = 0;
    bool txPending = false;          // ramka w buforze czeka na nadanie
    bool retainedFrameValid = false; // bufor trzyma NADANA ramke z zadaniem ACK
    uint8_t txType = RADIO_TYPE_APP;
    // +1 na zero konczace: tresci tekstowe (OTA, aplikacja) trafiaja do gory
    // jako gotowy C-string, bez kopiowania do osobnego bufora.
    uint8_t rxPayload[RADIO_PAYLOAD_CAPACITY + 1];
    uint8_t rxLength = 0;
    uint8_t lastFrameType = RADIO_TYPE_APP; // typ ostatnio odebranej ramki (z naglowka)
    uint8_t ackPayload[RADIO_ACK_PAYLOAD_SIZE];
    bool ackPending = false;         // potwierdzenie czeka na nadanie
    uint8_t ackSendBufferDest = 0;

    RadioFailCallback ackNotReceivedCallback = nullptr;
    void (*ackReceivedCallback)() = nullptr;
    RadioTextCallback dataReceivedCallback = nullptr;
    RadioTextCallback otaDataReceivedCallback = nullptr;
    RadioBytesCallback meshDataReceivedCallback = nullptr;
    void (*anyFrameReceivedCallback)(uint8_t senderId) = nullptr; // kazda poprawna ramka (mesh: dowod zycia sasiada)
    void (*dataSentCallback)() = nullptr;

    uint8_t messageId = 0;
    uint8_t pendingAckMessageId = 0; // id wyslanej ramki, do ktorej dopasowujemy potwierdzenie
    uint8_t sendBufferDest = 0;      // adresat i flaga ACK zwiazane z ramka w buforze
    bool sendBufferAckReq = false;
    int8_t sendBufferTxPwrOverride = -1; // wymuszona moc dla ramki w buforze (-1 = regulowana)
    int8_t frameTxPwrOverride = -1;      // j.w., przekazywana do startSending przez sendLoop
    uint8_t destinationIdOfLastMessage = 0;
    uint8_t senderIdOfLastMessage = 0;
    uint8_t receivedMessageIdOfLastMessage = 0;
    volatile int receivedPacketSize = 0;
    int lastRssi = 0; // RSSI ostatnio odebranej ramki (dBm), lapany przy odczycie z FIFO

    // Stan APC. peerReportedRssi = ostatnie "slychac cie na X dBm" ze zwrotki w ACK.
    int peerReportedRssi = 0;
    bool peerRssiValid = false;  // false = zadna zwrotka jeszcze nie dotarla
    uint8_t ackMissStreak = 0;   // kolejne timeouty ACK; limit -> sonda mocy
    bool apcFrozen = false;      // OTA: moc przypieta do sufitu, regulator spi
    int8_t apcPendingDbm = -1;   // zadana moc czeka na przerwe miedzy ramkami (-1 = brak)
    int8_t apcMaxDbm = APC_MAX_DBM; // sufit eskalacji, konfigurowalny per wezel (setApcMaxPower)

    int8_t txPowerDbm = 0;
    bool needToSendAckToSender = false;
    bool sendAckAutomaticly = true;

    unsigned long lastRamErrorMillis = 0;
    // Nasluch kanalu przed nadaniem (CSMA): jesli RSSI chwilowe przekracza prog,
    // ktos wlasnie nadaje - odkladamy ramke o obieg petli, najdluzej CS_MAX_WAIT_MS.
    unsigned long csBusySinceMillis = 0;

    // ==================== SKLADANIE RAMKI BEZ KOPII ====================
    // Wzorzec dla kazdego, kto chce cos nadac:
    //   uint8_t *p = manager->acquireTxBuffer();   // nullptr = radio zajete
    //   if (p == nullptr) return false;
    //   if (!RadioManager::txBufferFits(n)) { manager->releaseTxBuffer(); return false; }
    //   ...zapis n bajtow do p...
    //   return manager->commitTxBuffer(n, adresat, typ, czyACK, ok, blad);
    uint8_t *acquireTxBuffer();
    static bool txBufferFits(uint16_t bytes) { return bytes <= RADIO_PAYLOAD_CAPACITY; }
    bool commitTxBuffer(uint8_t len, uint8_t address, uint8_t type, bool ackRequested,
                        void (*_ackReceivedCallback)() = nullptr,
                        RadioFailCallback _ackNotReceivedCallback = nullptr,
                        int8_t txPowerDbmOverride = -1); // >= 0: wymuszona moc TEJ ramki (beacony)
    void releaseTxBuffer();               // wolajacy zrezygnowal po acquire

    // Nadana ramka z zadaniem ACK zostaje w buforze az do rozstrzygniecia
    // transakcji: to ona jest payloadem callbacku bledu i material na ponowienie.
    bool hasRetainedFrame();
    const uint8_t *retainedFrame() { return txPayload; }
    uint8_t retainedLength() { return retainedFrameValid ? txLength : 0; }
    uint8_t retainedType() { return txType; }
    bool resendRetained(uint8_t address, void (*_ackReceivedCallback)(),
                        RadioFailCallback _ackNotReceivedCallback);

    // Wygoda dla warstw, ktore maja gotowa tresc: jedna kopia do bufora, bez malloc.
    bool sendBytes(const uint8_t *data, uint8_t len, uint8_t address, uint8_t type,
                   bool ackRequested, void (*_ackReceivedCallback)() = nullptr,
                   RadioFailCallback _ackNotReceivedCallback = nullptr);
    bool sendText(const char *text, uint8_t address, uint8_t type,
                  void (*_ackReceivedCallback)() = nullptr,
                  RadioFailCallback _ackNotReceivedCallback = nullptr);
    bool sendOta(const char *text, uint8_t address,
                 void (*_ackReceivedCallback)() = nullptr,
                 RadioFailCallback _ackNotReceivedCallback = nullptr);

    // Malowanie stosu: wolny obszar miedzy sterta a stosem dostaje znany wzor przy
    // starcie; minStackGap() = najdluzszy nietkniety pas wzoru = najmniejszy zapas,
    // jaki KIEDYKOLWIEK wystapil (stos zaciera wzor od gory, sterta chwilowa od dolu).
    static void paintFreeStack();         // wolac jako pierwsza instrukcje setup()
    static uint16_t minStackGap();
    static int freeRam();

    void onDataReceived(RadioTextCallback callback);
    void onOtaDataReceived(RadioTextCallback callback);
    void onMeshDataReceived(RadioBytesCallback callback);
    void onAnyFrameReceived(void (*callback)(uint8_t senderId));
    void onDataSent(void (*callback)());

    virtual void receiveDone(int packetSize);
    virtual void txDone();
    void LoRa_rxMode();
    void LoRa_txMode();
    void loop();
    void sendLoop();
    void receiveLoop();
    bool readReceivedFrame();             // FIFO -> rxPayload; false = ramka nie dla nas
    void waitForAckTimeoutLoop();
    void txStuckWatchdogLoop();
    bool startSending(const uint8_t *payload, uint8_t len, uint8_t address,
                      uint8_t type, bool ackRequested);
    void sendAck();
    void setSendAckAutomaticly(bool value);
    bool isAckReceived();
    uint8_t getSenderIdOfLastMessage();
    int getLastRssi();
    bool isTransmissionFinished();
    void setupRadio(long frequency, int ss, int reset, int dio0, uint8_t _nodeId, void(*receiveDoneCallback)(int), void(*txDoneCallback)());
    int8_t setTxPower(int8_t dbm);       // zwraca moc faktycznie ustawiona (po ograniczeniu do zakresu)
    int8_t getTxPower();
    void apcOnAck(int8_t reportedRssi);  // zwrotka RSSI z ACK -> krok regulatora
    void apcOnAckTimeout();              // licznik strat ACK -> sonda mocy w gore
    void apcRequestPower(int8_t dbm);    // zadanie zmiany, aplikowane miedzy ramkami
    void setApcFrozen(bool frozen);      // true na czas OTA: sufit mocy + stop regulacji
    void setApcMaxPower(int8_t dbm);     // sufit eskalacji (FTDI: TX_POWER_FTDI_SAFE_DBM!)
    int8_t getEffectiveTxPower();        // moc, z jaka wyjdzie NASTEPNA ramka
    uint16_t getTxCurrentEstimate_mA();  // szacunkowy pobor pradu radia w czasie nadawania
    void printTxPower();
    void printRadioDiag(); // jedna linia: tryb radia, DIO0, IRQ, EIMSK, flagi, RAM
    void dumpRegisters();
    int getReceivedPacketSize();
    bool isNeedToSendAckToSender();
    bool isDataSent();

    void DEBUGlogln(const __FlashStringHelper *ifsh);
    void DEBUGlog(const __FlashStringHelper *ifsh);
    void DEBUGlogln();
    void DEBUGlogln(int n, int base = 10);
    void DEBUGlog(int n, int base = 10);
};


#endif //RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOMANAGER_H
