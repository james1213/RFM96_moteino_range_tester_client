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



///////////////////////////////Serial
#define PROGRAMMERID_DEFAULT     255
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


// ==================== ROZMIAR PAKIETU OTA ====================
// Ile bajtow firmware'u niesie jeden pakiet. MUSI byc zgodne z SINGLE_PACKET_SIZE
// w MainController.java po stronie PC. Limit dlugosci linii na serialu i rozmiar
// bufora wyliczaja sie z tej jednej liczby, wiec nie da sie ich rozjechac.
//
// Wieksze pakiety = szybszy transfer, bo staly narzut na pakiet (ramka odpowiedzi
// ~72 ms + potwierdzenie ~46 ms + obsluga) nie zalezy od ilosci danych. Dla obrazu
// 18 KB, przy 2 dBm / SF7 / BW 125 kHz:
//   B/pakiet | linia serial | pakiety | czas transferu
//   ---------+--------------+---------+---------------
//         16 |      49      |  1130   | ~4,7 min   (obecnie)
//         32 |      69      |   565   | ~2,5 min
//         64 |     113      |   283   | ~1,8 min   <- zalecane: ostatni rozmiar,
//            |              |         |               ktory miesci sie w buforach
//         96 |     153      |   189   | ~1,4 min
// Powyzej 16 B trzeba tez dodac -DSERIAL_RX_BUFFER_SIZE=128 w platformio.ini
// (kompilator sam o tym przypomni ostrzezeniem ponizej) - jest juz dodane.
#define OTA_PACKET_SIZE_BYTES 64

// Najdluzsza linia "FLX?HEX?<numer>?<base64>?<crc32>", jaka moze przyjsc z PC:
// 8 znakow prefiksu + do 5 cyfr numeru pakietu + '?' + base64 + '?' + do 10 cyfr CRC32.
// Podloga 64 zostawia zapas na krotkie komendy konfiguracyjne (PROGRAMMERID: itp.).
#define OTA_BASE64_LENGTH(dataBytes) (((dataBytes) + 2) / 3 * 4)
#define OTA_SERIAL_LINE_NEEDED (8 + 5 + 1 + OTA_BASE64_LENGTH(OTA_PACKET_SIZE_BYTES) + 1 + 10)
#define OTA_SERIAL_LINE_MAX ((OTA_SERIAL_LINE_NEEDED) > 64 ? (OTA_SERIAL_LINE_NEEDED) : 64)

#if OTA_SERIAL_LINE_MAX > 255
#error "OTA_SERIAL_LINE_MAX > 255 - readSerialLine przekazuje dlugosc jako uint8_t"
#endif
#if OTA_SERIAL_LINE_MAX > SERIAL_RX_BUFFER_SIZE
#warning "Linia OTA dluzsza niz bufor RX UART: dodaj build_flags = -DSERIAL_RX_BUFFER_SIZE=128 w platformio.ini"
#endif



class RadioOta {
private:
    RadioManager *manager;

    // Byla tu tablica uint8_t source[6][10] z zaszytymi danymi testowymi - 60 bajtow
    // sterty na obiekt, do ktorego nic nigdy nie siegalo. Usunieta; w razie potrzeby
    // do odtworzenia z historii gita.

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
        uint16_t PROGRAMMER_ID;
    } CONFIG;

    char _input[OTA_SERIAL_LINE_MAX + 1]; // +1 na koncowe zero dopisywane przez readSerialLine
    char c = 0;
    uint16_t targetID=0;


    // Byl tu String serialReceivedBuffer - kopia linii, ktora i tak lezy juz w _input.
    // Usunieta: ~106 B sterty w najciasniejszym momencie wysylki.
    long currentHexPacketNumber = -1; // numer pakietu z ostatniej ramki FLX?DAT? wyslanej w eter

    bool isResponseForCurrentHexPacket(const String &str, uint8_t prefixLength);
    void noteStaleHexResponse();

    uint8_t readSerialLine(char* input, char endOfLineChar=10, uint8_t maxLength=OTA_SERIAL_LINE_MAX, uint16_t timeout=1000);
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
    bool isOtaInProgress();
//    void radioSendHex();
    bool radioSendHandshake();
    bool radioSendEof();
    void radioOtaDataReceived(String &str, uint8_t senderId);

    ///////////////////////////////////////////////

    void serialSendHandshakeResponse(uint8_t *input, uint8_t inputLen, uint16_t targetID, uint16_t timeout, uint16_t ackTimeout,
                                     uint8_t debug);

    bool isHexEofMessage(String &str);

    bool radioSendHexFromSerial();
};


#endif //RFM96_MOTEINO_RANGE_TESTER_CLIENT_RADIOOTA_H
