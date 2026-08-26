//#define digitalPinToInterrupt(p)  ((p) == 10 ? 0 : ((p) == 11 ? 1 : ((p) == 2 ? 2 : NOT_AN_INTERRUPT)))
#include <Arduino.h>
#include <radiomanager/RadioManager.h>
#include <SPIFlash.h>
//#include "arduino_base64.hpp"
#include "ota/RadioOta.h"
//#include <CRC32.h>

// Wyswietlacz OLED 0,91" 128x32 (SSD1306) na I2C (A4=SDA, A5=SCL), adres 0x3C.
// SSD1306Ascii celowo: nie trzyma framebuffera (128x32/8 = 512 B, czwarta czesc
// calego RAM), pisze znaki wprost do pamieci wyswietlacza.
#include <SSD1306AsciiAvrI2c.h>

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
//
// Przy wlaczonym APC (RadioManager.h) to tylko moc STARTOWA - dalej moc reguluje
// sie sama wedlug RSSI raportowanego w ACK-ach przez druga strone.
#define TX_POWER_DBM 20

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

#define OLED_I2C_ADDRESS 0x3C
SSD1306AsciiAvrI2c oled;
bool oledPresent = false;

boolean runEvery(unsigned long interval);


void setupSerial();

void setupRadio();

void setupFlash();

void setupDisplay();

void displayCountAndRssi(const String &str);

void dataReceived(String &str, uint8_t senderId);

void setup() {
    setupSerial();
    Serial.println(F("ver. 1.1"));
    printResetCause();
    setupRadio();
    setupFlash();
    setupDisplay();
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

void setupDisplay() {
    // Slabe wewnetrzne pull-upy na SDA/SCL: bez podlaczonego wyswietlacza magistrala
    // nie plywa i sonda dostaje czysty NACK zamiast serii timeoutow TWI. Z modulem
    // OLED dominuja jego wlasne rezystory 4,7k-10k, wiec nic nie psuja.
    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);
    AvrI2c probe;
    probe.begin(false); // 100 kHz wystarczy do sondowania
    oledPresent = probe.start((OLED_I2C_ADDRESS << 1) | 0); // ACK = wyswietlacz jest
    probe.stop();
    if (!oledPresent) {
        Serial.println(F("[OLED] brak wyswietlacza pod 0x3C - wyswietlanie wylaczone"));
        return;
    }
    oled.begin(&Adafruit128x32, OLED_I2C_ADDRESS);
    oled.setFont(System5x7);
    oled.clear();
    oled.set2X();
    oled.println(F("KLIENT")); // wiersze 0-1 (2X); wiersz 2 zostaje zawsze pusty
    oled.set1X();
    oled.setCursor(0, 3);      // wiersz 3 (1X) - ten sam uklad co pozniejszy status
    oled.print(F("czekam..."));
    Serial.println(F("[OLED] 128x32 pod 0x3C gotowy"));
}

// Wyswietla: duzo (2X) numer z "[#N]" i RSSI ramki; malym fontem (wiersz 3) moc,
// z jaka MY nadajemy, oraz moc, z jaka nadawca wyslal te wiadomosc (znacznik
// "[PX]" doklejany do tresci testowej). Te moce NIE musza byc rowne - kazdy
// kierunek reguluje sie niezaleznie. Wiadomosci bez "[#" nie ruszaja ekranu.
void displayCountAndRssi(const String &str) {
    // Podczas transferu OTA nie dotykamy wyswietlacza: jeden zapis I2C to kilka ms
    // blokady petli glownej, a o niezawodnosc OTA walczylismy zbyt dlugo.
    if (!oledPresent || radioOta->isOtaInProgress()) return;
    const char *payload = str.c_str(); // String uniewazniony przez OOM ma bufor NULL
    if (payload == nullptr) return;
    const char *counter = strstr(payload, "[#"); // strstr zamiast String::indexOf - zero alokacji
    if (counter == nullptr) return;
    unsigned long updateStart = millis();
    oled.set2X();
    oled.setCursor(0, 0);
    oled.print('#');
    oled.print(atol(counter + 2));
    oled.print(' ');
    oled.print(manager->getLastRssi()); // bez "dBm" - 2X miesci 10 znakow w linii
    oled.clearToEOL();
    oled.set1X();
    oled.setCursor(0, 3);
    oled.print(F("ja:"));
    oled.print((int) manager->getTxPower()); // int8_t bez rzutu trafilby w print(char)
    oled.print(F("dBm on:"));
    const char *peerPower = strstr(payload, "[P"); // moc nadawcy doslana w tresci
    if (peerPower != nullptr) {
        oled.print(atol(peerPower + 2));
        oled.print(F("dBm"));
    } else {
        oled.print('?'); // wiadomosc ze starszego firmware - bez znacznika [P]
    }
    oled.clearToEOL();
    // Normalna aktualizacja to ~20-25 ms; brak wyswietlacza to szybkie NACK-i.
    // Ale SDA/SCL zwarte do masy (realne w terenie) = kazdy bajt czeka na timeout
    // TWI ~37 ms, czyli ~30 s blokady petli na kazda ramke. Wykrywamy i wylaczamy;
    // powrot wyswietlacza dopiero po resecie.
    if (millis() - updateStart > 100) {
        oledPresent = false;
        Serial.println(F("[OLED] magistrala I2C nie odpowiada - wyswietlanie wylaczone"));
    }
}


void loop() {
    manager->loop();

    // APC: na czas transferu OTA moc przypieta do sufitu, regulator zamrozony
    // (wykrywanie zbocza w srodku - wolanie co obieg jest tanie).
    manager->setApcFrozen(radioOta->isOtaInProgress());

    // Ruch testowy: wstrzymany, gdy trwa transfer OTA (isOtaInProgress), a gdy radio
    // jest chwilowo zajete (send() zwraca false), wiadomosc jest po prostu pomijana -
    // send() NIE nadpisuje juz po cichu zakolejkowanej ramki.
    if (!radioOta->isOtaInProgress() && runEvery(1000)) {
        Serial.println();

        String str;
        // 38 (prefiks) + do 6 cyfr licznika + "][P" + 2 cyfry mocy + 51 (sufiks)
        // = do 100 znakow. Rezerwa musi pokrywac calosc, inaczej realokacja przy
        // kazdej wiadomosci tylko podnosi szczyt zuzycia sterty.
        str.reserve(100);
        str += F("Hello World from sender to receiver [#");
        str += count++;
        str += F("][P"); // znacznik APC: moc, z jaka ta wiadomosc jest nadawana
        str += (int) manager->getTxPower(); // int8_t bez rzutu trafilby w concat(char)
        str += F("] with ACK | test string 1234567890ABCDEFGHIJKLMNOP");
        Serial.print(F("Sending payload: \""));
        Serial.print(str);
        Serial.println(F("\""));
        bool queued = manager->send(str, 2,
                      []() {
                          Serial.println(F("MAIN | OK"));
                      },
                      [](String &payload) {
                          Serial.print(F("MAIN | NOT OK, payload = "));
                          Serial.println(payload);
                      });
        if (!queued) Serial.println(F("MAIN | radio busy, skipped"));
    }

    radioOta->loop();
}

void dataReceived(String &str, uint8_t senderId) {
    Serial.print(F("MAIN | Received data: \""));
    Serial.print(str);
    Serial.print(F("\" from senderId: "));
    Serial.println(senderId);
    displayCountAndRssi(str);
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

