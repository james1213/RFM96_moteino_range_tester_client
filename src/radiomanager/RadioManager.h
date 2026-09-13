//
// Created by LukaszLibront on 19.12.2023.
//

#ifndef RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOMANAGER_H
#define RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOMANAGER_H

#pragma once

#include <Arduino.h>
#include <LoRa.h>



// ============ CO MOZNA USTAWIC Z platformio.ini (build_flags) ============
// Stale osloniete ponizej #ifndef to LOKALNA POLITYKA WEZLA: rozmiary tablic,
// czasy, progi, moc. Wolno je nadpisac per projekt, np. -DMESH_MAX_NEIGHBORS=8,
// i wolno, zeby rozne wezly mialy je rozne - nikt poza wlasnym wezlem ich nie
// oglada.
//
// Stale BEZ oslony opisuja FORMAT RAMKI: naglowek radiowy, typy wiadomosci,
// rozmiary naglowkow mesh. Musza byc identyczne na wszystkich wezlach, bo wezel
// o innej wartosci przestaje rozumiec ramki sasiadow. Dlatego celowo nie da sie
// ich podmienic z pliku projektu - zmienia sie je tutaj, dla calej sieci naraz.

#ifndef LOG_ACTIVE
#define LOG_ACTIVE false
#endif

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

// Potwierdzenia czekaja w krotkiej kolejce, a nie w jednym gniezdzie. Nadanie
// potrafi sie odlozyc o 400 ms (nasluch kanalu) albo o obieg petli (nieodczytana
// ramka w FIFO), a przy trzech wezlach i kadencji 1 s w tym czasie potrafi
// przyjsc kolejna ramka do skwitowania. Z jednym gniazdem pierwsze potwierdzenie
// przepadalo bezgłośnie: nadawca odliczal timeout, a mesh uniewaznial trasy
// przez zywego sasiada.
#ifndef RADIO_ACK_QUEUE_LEN
#define RADIO_ACK_QUEUE_LEN 3
#endif

// Moc nadawania RFM95/96: uzywamy wyjscia PA_BOOST, bo tylko ono jest podlaczone
// do anteny w modulach HopeRF (wyjscie RFO zostaje niepodlaczone - dalo by ~zero mocy).
// Powyzej 17 dBm uklad wchodzi w tryb wysokiej mocy (PA_DAC) i wymaga podniesienia
// progu OCP - biblioteka ustawia tam 140 mA, co przycina PA przy 20 dBm.
#define TX_POWER_MIN_DBM        2   // minimum dla PA_BOOST
#define TX_POWER_MAX_DBM        20  // maksimum SX1276 (tryb PA_DAC)
#ifndef TX_POWER_FTDI_SAFE_DBM
#define TX_POWER_FTDI_SAFE_DBM  10  // powyzej tego zasilanie z pinu 3V3 FTDI nie wyrabia
#endif
#define TX_OCP_HIGH_POWER_MA    150 // limit pradu PA dla trybu >17 dBm (Semtech 5.4.3)

// ==================== PARAMETRY RADIA (WSPOLNE DLA CALEJ SIECI) ====================
// Czestotliwosc, SF, BW i CR musza byc IDENTYCZNE na wszystkich wezlach - wezel
// z innym zestawem w ogole nie slyszy reszty. Osloniete #ifndef tylko po to, zeby
// dalo sie je przetestowac z platformio.ini; zmieniaj je wtedy w obu projektach
// naraz. Wartosci podawaj jako liczby calkowite (434000000, nie 434E6), bo
// sprawdzenia ponizej liczy preprocesor.
//
// SF7 / BW 500 kHz: ramka 4x krotsza niz przy 125 kHz (dane 52 B: 26 ms zamiast
// 103 ms), czulosc -116 dBm zamiast -123 dBm - zasieg mniejszy o ~40%. Wymiana
// na pojemnosc sieci: przy limicie 30% zajetosci kanalu ~20 wezlow zamiast ~6.
#ifndef RADIO_FREQUENCY_HZ
#define RADIO_FREQUENCY_HZ      434000000  // srodek pasma ISM 433,05-434,79 MHz
#endif
#ifndef RADIO_SPREADING_FACTOR
#define RADIO_SPREADING_FACTOR  7
#endif
#ifndef RADIO_BANDWIDTH_HZ
#define RADIO_BANDWIDTH_HZ      500000
#endif
#ifndef RADIO_CODING_RATE_DENOM
#define RADIO_CODING_RATE_DENOM 5          // CR 4/5
#endif

// SF6 dziala na SX1276 tylko z niejawnym naglowkiem (stala dlugosc ramki), a nasz
// protokol rozpoznaje ramki po dlugosci z naglowka LoRa - dlatego od SF7.
#if RADIO_SPREADING_FACTOR < 7 || RADIO_SPREADING_FACTOR > 12
#error "RADIO_SPREADING_FACTOR poza zakresem 7..12"
#endif
#if RADIO_BANDWIDTH_HZ != 7800L && RADIO_BANDWIDTH_HZ != 10400L && RADIO_BANDWIDTH_HZ != 15600L \
 && RADIO_BANDWIDTH_HZ != 20800L && RADIO_BANDWIDTH_HZ != 31250L && RADIO_BANDWIDTH_HZ != 41700L \
 && RADIO_BANDWIDTH_HZ != 62500L && RADIO_BANDWIDTH_HZ != 125000L && RADIO_BANDWIDTH_HZ != 250000L \
 && RADIO_BANDWIDTH_HZ != 500000L
#error "RADIO_BANDWIDTH_HZ: dozwolone 7800..500000 wg SX1276 (np. 125000, 250000, 500000)"
#endif
#if RADIO_CODING_RATE_DENOM < 5 || RADIO_CODING_RATE_DENOM > 8
#error "RADIO_CODING_RATE_DENOM: 5..8 (CR 4/5..4/8)"
#endif
// Caly kanal (srodek +- polowa BW) ma lezec w pasmie. Dawne 433E6 z BW 125 kHz
// wystawalo 12,5 kHz ponizej 433,05 MHz, a z BW 500 kHz wystawaloby o 300 kHz.
#if !defined(RADIO_ALLOW_OUT_OF_BAND) && \
    ((RADIO_FREQUENCY_HZ - RADIO_BANDWIDTH_HZ / 2) < 433050000L || \
     (RADIO_FREQUENCY_HZ + RADIO_BANDWIDTH_HZ / 2) > 434790000L)
#error "Kanal wychodzi poza pasmo ISM 433,05-434,79 MHz (swiadomie: -DRADIO_ALLOW_OUT_OF_BAND)"
#endif

// Przyblizony czas nadania NAJDLUZSZEJ ramki (120 B), zaokraglony w gore: ~200
// symboli po 2^SF/BW. SF7/125: 204 ms (dokladnie 200), SF7/500: 51 ms (dokladnie
// 50). Z niego wynikaja wszystkie czasy oczekiwania ponizej, wiec zmiana SF albo
// BW nie wymaga recznego przestrajania timeoutow.
#define RADIO_MAX_FRAME_MS ((200000UL << RADIO_SPREADING_FACTOR) / RADIO_BANDWIDTH_HZ)

// Zamiana wartosci stalej na napis (dwa poziomy, zeby rozwinac makro przed #).
#define RADIO_STR_(x) #x
#define RADIO_STR(x)  RADIO_STR_(x)

// Czas oczekiwania na ACK: nasza ramka + odlozenie odpowiedzi przez nasluch
// kanalu + sama ramka ACK + obieg petli u odbiorcy. SF7/125: 1070 ms (bylo 1000),
// SF7/500: 305 ms - przy 1000 ms ponowienia czekalyby 3 razy dluzej niz trzeba.
#ifndef RADIO_ACK_TIMEOUT_MS
#define RADIO_ACK_TIMEOUT_MS (5 * RADIO_MAX_FRAME_MS + 50)
#endif
// Straznik zawieszonego nadawania musi byc wyraznie dluzszy od najdluzszej ramki.
// Stale 2000 ms falszywie zabijaloby nadawanie juz przy SF11/125 (ramka 2,5 s).
#ifndef RADIO_TX_STUCK_TIMEOUT_MS
#define RADIO_TX_STUCK_TIMEOUT_MS (3 * RADIO_MAX_FRAME_MS + 1500)
#endif

// Nasluch kanalu przed nadaniem (CSMA na RSSI chwilowym). Podloga szumu SX1276
// przy SF7/125 kHz to ok. -110..-120 dBm; ramka sasiada z biurka to -20..-60 dBm.
#ifndef CS_BUSY_RSSI_DBM
#define CS_BUSY_RSSI_DBM  (-85)
#endif
// Dwie najdluzsze ramki: tyle trwa najgorszy przypadek zajetego kanalu, dalej to
// juz szum albo obcy nadajnik. SF7/125: 408 ms (bylo 400), SF7/500: 102 ms.
#ifndef CS_MAX_WAIT_MS
#define CS_MAX_WAIT_MS    (2 * RADIO_MAX_FRAME_MS)
#endif

// Dioda aktywnosci: swieci stale, a przy kazdym nadaniu i kazdym odebranym
// pakiecie gasnie na tyle milisekund. 40 ms to najkrotsze wyraznie widoczne
// mrugniecie - krotsze oko zlewa z ciaglym swieceniem.
#ifndef RADIO_ACTIVITY_LED_OFF_MS
#define RADIO_ACTIVITY_LED_OFF_MS 40
#endif

// ==================== AUTOMATYCZNA REGULACJA MOCY (APC) ====================
// Kazdy ACK niesie RSSI, z jakim odbiorca uslyszal kwitowana ramke. Nadawca
// reguluje SWOJA moc tak, by u odbiorcy trafic w okno APC_TARGET_RSSI_DBM +-
// APC_HYSTERESIS_DB. Dwie niezalezne petle (po jednej na kierunek) - bez
// zalozenia symetrii lacza, wiec bez sprzezenia regulatorow.
#ifndef APC_ENABLED
#define APC_ENABLED           true
#endif
#ifndef APC_TARGET_RSSI_DBM
#define APC_TARGET_RSSI_DBM   (-85) // cel: tak ma nas slyszec druga strona
#endif
#ifndef APC_HYSTERESIS_DB
#define APC_HYSTERESIS_DB     5     // martwa strefa; RSSI i tak skacze o +-kilka dB
#endif
#ifndef APC_STEP_DB
#define APC_STEP_DB           2     // krok pojedynczej korekty
#endif
#ifndef APC_ACK_MISS_LIMIT
#define APC_ACK_MISS_LIMIT    3     // tyle timeoutow ACK z rzedu -> sonda mocy w gore
#endif
// Sufit eskalacji. UWAGA na zasilanie: przy 3V3 z FTDI ustaw TX_POWER_FTDI_SAFE_DBM,
// inaczej automatyczny skok mocy przy slabym laczu wpedzi wezel w petle brown-outow.
#ifndef APC_MAX_DBM
#define APC_MAX_DBM           TX_POWER_MAX_DBM
#endif

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
    unsigned long txStuckTimeout = RADIO_TX_STUCK_TIMEOUT_MS; // ms; >> najdluzszy czas ramki w powietrzu
    volatile bool transmissionClenedUp = true;
    volatile bool ackReceived = false;
    bool waitingForAck = false;
    bool ackFramePendingTx = false; // ramka z zadaniem ACK zakolejkowana, ale jeszcze nie nadana
    unsigned long waitForAckStartTime = 0;
    unsigned long ackTimeout = RADIO_ACK_TIMEOUT_MS; // ms, wyliczone z SF i BW

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
    uint8_t ackQueue[RADIO_ACK_QUEUE_LEN][RADIO_ACK_PAYLOAD_SIZE];
    uint8_t ackQueueDest[RADIO_ACK_QUEUE_LEN];
    uint8_t ackQueueHead = 0;        // najstarsze czekajace potwierdzenie
    uint8_t ackQueueCount = 0;       // 0 = nic nie czeka

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
    int lastRssi = 0; // RSSI ostatnio odebranej ramki (dBm, z poprawka na SNR < 0), lapany przy odczycie z FIFO

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

    // Dioda aktywnosci radia (-1 = brak). Sterowana wylacznie z petli glownej:
    // przerwania TxDone/RxDone tylko ustawiaja flagi, a zgaszenie i ponowne
    // zapalenie dzieje sie w startSending, receiveLoop i activityLedLoop.
    int8_t activityLedPin = -1;
    bool activityLedOff = false;
    unsigned long activityLedOffAt = 0;
    void setActivityLed(int8_t pin);     // zapala diode na stale i wlacza mruganie
    void activityBlink();                // gasi diode na RADIO_ACTIVITY_LED_OFF_MS
    void activityLedLoop();              // zapala ja z powrotem, gdy czas minie

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
