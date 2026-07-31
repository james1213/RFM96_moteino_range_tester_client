//#define digitalPinToInterrupt(p)  ((p) == 10 ? 0 : ((p) == 11 ? 1 : ((p) == 2 ? 2 : NOT_AN_INTERRUPT)))
#include <Arduino.h>
#include <radiomanager/RadioManager.h>
#include <SPIFlash.h>
//#include "arduino_base64.hpp"
#include "ota/RadioOta.h"
//#include <CRC32.h>

#define NODE_ID 0x01

// ==================== MOC NADAJNIKA ====================
// Zakres 2..20 dBm (wyjscie PA_BOOST - jedyne podlaczone do anteny w modulach RFM95/96).
//
//   dBm | pobor radia | zasieg | uwagi
//   ----+-------------+--------+---------------------------------------------------
//     2 |   ~25 mA    |  maly  | testy na biurku, bezpieczne na pinie 3V3 FTDI
//    10 |   ~40 mA    |   ok   | maksimum jakie warto probowac na zasilaniu z FTDI
//    17 |   ~90 mA    |  duzy  | wymaga baterii albo 5V przez regulator Moteino
//    20 |  ~130 mA    |  MAKS  | tryb PA_DAC; mocne zasilanie + ~100 uF przy radiu,
//       |             |        | Semtech zaleca duty cycle <= 1%
//
// Za wysoka wartosc przy slabym zasilaniu = reset przy kazdym nadawaniu; widac to
// w logu jako "[BOOT] Reset cause: BROWN_OUT". Moc mozna tez zmieniac w trakcie
// pracy: manager->setTxPower(dbm) - obowiazuje od nastepnej transmisji.
#define TX_POWER_DBM 2

#if TX_POWER_DBM < TX_POWER_MIN_DBM || TX_POWER_DBM > TX_POWER_MAX_DBM
#error "TX_POWER_DBM poza zakresem - dozwolone 2..20 dBm (PA_BOOST)"
#endif


int count = 0;

// Optiboot przed skokiem do aplikacji zeruje MCUSR, ale oryginal zostawia w r2.
uint8_t resetFlags __attribute__((section(".noinit")));
void resetFlagsInit(void) __attribute__((naked)) __attribute__((used)) __attribute__((section(".init0")));
void resetFlagsInit(void) {
    __asm__ __volatile__("sts %0, r2\n" : "=m"(resetFlags) :);
}

void printResetCause() {
    uint8_t flags = resetFlags | MCUSR;
    MCUSR = 0;
    Serial.print(F("[BOOT] Reset cause:"));
    if (flags & _BV(PORF)) Serial.print(F(" POWER_ON"));
    if (flags & _BV(EXTRF)) Serial.print(F(" EXTERNAL"));
    if (flags & _BV(BORF)) Serial.print(F(" BROWN_OUT"));
    if (flags & _BV(WDRF)) Serial.print(F(" WATCHDOG"));
    if (flags == 0) Serial.print(F(" NONE (crash programu / skok do 0)"));
    Serial.println();
}

SPIFlash flash(SS_FLASHMEM, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

RadioManager *manager = new RadioManager();
RadioOta *radioOta = new RadioOta(manager);

boolean runEvery(unsigned long interval);


void setupSerial();

void setupRadio();

void setupFlash();

void dataReceived(String &str, uint8_t senderId);

void setup() {
    setupSerial();
    Serial.println(F("ver. 1.1"));
    printResetCause();
    setupRadio();
    setupFlash();
    delay(5000);
}

void setupRadio() {
    manager->onDataReceived(dataReceived);
//    manager->onOtaDataReceived(radioOtaDataReceived);
    manager->onOtaDataReceived([](String &str, uint8_t senderId){
        radioOta->radioOtaDataReceived(str, senderId);
    });

    manager->onDataSent([]() {
//        Serial.println(F("MAIN | data sent"));
    });

    manager->setupRadio(433E6, 10, 7, 2, NODE_ID,
                        [](int packetSize) {
                            manager->receiveDone(packetSize);
                        },
                        []() {
                            manager->txDone();
                        });

    manager->setTxPower(TX_POWER_DBM);
    manager->printTxPower();

    manager->dumpRegisters();
}

void setupFlash() {
    Serial.println(F("CLIENT"));
    Serial.println(F("[FLASH] Setup started"));
    if (flash.initialize()) {
        Serial.println(F("[FLASH] SPI Flash Init OK"));
        Serial.print(F("[FLASH] UniqueID (MAC): "));
        flash.readUniqueId();
        for (byte i = 0; i < 8; i++) {
            Serial.print(flash.UNIQUEID[i], HEX);
            Serial.print(':');
        }
        Serial.println();

        char flashBuff[50];
        sprintf(flashBuff, "[FLASH] DeviceID: 0x%X", flash.readDeviceId());
        Serial.println(flashBuff);
        Serial.println(F("[FLASH] Setup finished"));
    } else {
        Serial.println(F("[FLASH] SPI Flash MEM not found (is chip soldered?)..."));
    }
}

void setupSerial() {
    Serial.begin(115200);
    while (!Serial);
}


void loop() {
    manager->loop();

    // Ruch testowy: wstrzymany, gdy trwa transfer OTA (isOtaInProgress), a gdy radio
    // jest chwilowo zajete (send() zwraca false), wiadomosc jest po prostu pomijana -
    // send() NIE nadpisuje juz po cichu zakolejkowanej ramki.
    // if (!radioOta->isOtaInProgress() && runEvery(2000)) {
    //     Serial.println();
    //
    //     String str;
    //     str.reserve(80);
    //     str += F("Hello World [#");
    //     str += count++;
    //     str += F("] with ACK | test string 1234567890ABCDEFGHIJKLMNOP");
    //     Serial.print(F("Sending payload: \""));
    //     Serial.print(str);
    //     Serial.println(F("\""));
    //     bool queued = manager->send(str, 2,
    //                   []() {
    //                       Serial.println(F("MAIN | OK"));
    //                   },
    //                   [](String &payload) {
    //                       Serial.print(F("MAIN | NOT OK, payload = "));
    //                       Serial.println(payload);
    //                   });
    //     if (!queued) Serial.println(F("MAIN | radio busy, skipped"));
    // }

    radioOta->loop();
}

void dataReceived(String &str, uint8_t senderId) {
    // Serial.print(F("MAIN | Received data: \""));
    // Serial.print(str);
    // Serial.print(F("\" from senderId: "));
    // Serial.println(senderId);
}


boolean runEvery(unsigned long interval) {
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        return true;
    }
    return false;
}

