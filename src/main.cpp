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

#include <mesh/MeshRouter.h>

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
// Konfiguracja LAWKOWA (plytki blisko siebie, zasilanie z USB/FTDI). Przy 20 dBm
// z odleglosci kilkudziesieciu cm odbiornik ulega saturacji: ramki gina wlasnie
// dlatego, ze sygnal jest ZA MOCNY, a eskalacja APC "brak ACK -> pelna moc"
// pogarsza sprawe (widziane na sprzecie jako spirala strat przy [P18-P20]).
// W terenie: TX_POWER_DBM 20 i APC_CEILING_DBM 20.
#define TX_POWER_DBM 2

// Sufit automatycznej eskalacji APC (skok przy stratach ACK i na czas transferu OTA).
// UWAGA: przy zasilaniu z pinu 3V3 FTDI ustaw najwyzej TX_POWER_FTDI_SAFE_DBM (10) -
// inaczej automat sam, bez udzialu TX_POWER_DBM, wpedzi plytke w petle brown-outow.
#define APC_CEILING_DBM 10

#if TX_POWER_DBM < TX_POWER_MIN_DBM || TX_POWER_DBM > TX_POWER_MAX_DBM
#error "TX_POWER_DBM poza zakresem - dozwolone 2..20 dBm (PA_BOOST)"
#endif


static uint16_t count = 0;
static uint8_t actualDest = 2;
static uint8_t maxNodes = 3;

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
MeshRouter *mesh = new MeshRouter(manager);

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
    manager->onDataReceived(dataReceived); // stare <DAT> wprost - zgodnosc wstecz
//    manager->onOtaDataReceived(radioOtaDataReceived);
    manager->onOtaDataReceived([](String &str, uint8_t senderId){
        radioOta->radioOtaDataReceived(str, senderId);
    });
    manager->onMeshDataReceived([](String &str, uint8_t senderId){
        mesh->radioMeshDataReceived(str, senderId);
    });
    manager->onAnyFrameReceived([](uint8_t senderId){
        mesh->noteFrameFrom(senderId); // kazda ramka = dowod zycia sasiada
    });
    // Dane dostarczone przez mesh laduja w tym samym handlerze co bezposrednie -
    // drugi parametr to wtedy WEZEL ZRODLOWY, nie nadawca ostatniego skoku.
    mesh->onDataReceived(dataReceived);

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
    manager->setApcMaxPower(APC_CEILING_DBM);
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

        // Bez sprintf: rodzina printf (vfprintf + pomocnicy) kosztowala ~1,1 KB flasha.
        Serial.print(F("[FLASH] DeviceID: 0x"));
        Serial.println(flash.readDeviceId(), HEX);
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
//
// Rysowanie jest ROZNICOWE: gorna linia trzyma poprzednia klatke i przepisuje
// wylacznie zmienione komorki znakowe (zwykle 1-2 cyfry, ~1-2 ms), a linia mocy
// przerysowuje sie tylko przy faktycznej zmianie ktorejs z mocy. Przepisywanie
// calej linii co sekunde bylo widoczne jako mruganie ekranu.
void displayCountAndRssi(const String &str) {
    static char prevTopLine[11] = "";  // poprzednia klatka gornej linii (10 komorek 2X po 12 px)
    static int8_t prevOwnPower = -1;   // -1 = jeszcze nic nie narysowano
    static int prevPeerPower = -128;   // -128 = znacznika [P] nie bylo ("?")

    // Podczas transferu OTA nie dotykamy wyswietlacza: jeden zapis I2C to kilka ms
    // blokady petli glownej, a o niezawodnosc OTA walczylismy zbyt dlugo.
    if (!oledPresent || radioOta->isOtaInProgress()) return;
    const char *payload = str.c_str(); // String uniewazniony przez OOM ma bufor NULL
    if (payload == nullptr) return;
    const char *counter = strstr(payload, "[#"); // strstr zamiast String::indexOf - zero alokacji
    if (counter == nullptr) return;
    unsigned long updateStart = millis();

    // Gorna linia (2X): "#<licznik> <rssi>". Komorki za koncem tekstu dopelniamy
    // spacjami - to zastepuje clearToEOL i przy okazji przykrywa napis startowy.
    // Skladanie reczne zamiast snprintf: rodzina printf to ~1,1 KB flasha.
    char topLine[11];
    char num[12];
    uint8_t pos = 0;
    topLine[pos++] = '#';
    ltoa(atol(counter + 2), num, 10);
    for (char *c = num; *c != '\0' && pos < 10; c++) topLine[pos++] = *c;
    if (pos < 10) topLine[pos++] = ' ';
    itoa(manager->getLastRssi(), num, 10);
    for (char *c = num; *c != '\0' && pos < 10; c++) topLine[pos++] = *c;
    topLine[pos] = '\0';
    oled.set2X();
    bool textEnded = false;
    for (uint8_t i = 0; i < 10; i++) {
        if (topLine[i] == '\0') textEnded = true;
        char c = textEnded ? ' ' : topLine[i];
        if (c != prevTopLine[i]) {
            oled.setCursor(i * 12, 0);
            oled.write(c);
            prevTopLine[i] = c;
        }
    }

    // Linia mocy (1X, wiersz 3): przerysowanie tylko przy zmianie wartosci.
    int8_t ownPower = manager->getTxPower();
    int peerPower = -128;
    const char *peerMarker = strstr(payload, "[P"); // moc nadawcy doslana w tresci
    if (peerMarker != nullptr) peerPower = (int) atol(peerMarker + 2);
    if (ownPower != prevOwnPower || peerPower != prevPeerPower) {
        prevOwnPower = ownPower;
        prevPeerPower = peerPower;
        oled.set1X();
        oled.setCursor(0, 3);
        oled.print(F("ja:"));
        oled.print((int) ownPower); // int8_t bez rzutu trafilby w print(char)
        oled.print(F("dBm on:"));
        if (peerPower == -128) {
            oled.print('?'); // wiadomosc ze starszego firmware - bez znacznika [P]
        } else {
            oled.print(peerPower);
            oled.print(F("dBm"));
        }
        oled.clearToEOL(); // domiata ogon "czekam..." i dluzsze poprzednie wartosci
    }
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
    // Mesh: na czas OTA bez beaconow i forwardingu (RAM i airtime dla transferu).
    mesh->setFrozen(radioOta->isOtaInProgress());
    mesh->loop();

    // Czarna skrzynka: stan radia i mesh co 10 s - do diagnozy epizodow gluchoty.
    static unsigned long lastDiagMillis = 0;
    if (millis() - lastDiagMillis >= 10000) {
        lastDiagMillis = millis();
        manager->printRadioDiag();
        mesh->printState();
    }

    // Ruch testowy: wstrzymany, gdy trwa transfer OTA (isOtaInProgress), a gdy radio
    // jest chwilowo zajete (send() zwraca false), wiadomosc jest po prostu pomijana -
    // send() NIE nadpisuje juz po cichu zakolejkowanej ramki.
    if (!radioOta->isOtaInProgress() && runEvery(1000 + (micros() & 0xFF))) {
        // Jitter 0-255 ms: oba wezly nadaja "co sekunde", wiec bez niego potrafia
        // zsynchronizowac sie tak, ze ich ACK-i (wyzwalane odbiorem) leca rownoczesnie
        // i zderzaja sie w eterze co cykl - kwarce dryfuja zbyt wolno, by fazy same
        // sie rozeszly. Widziane na sprzecie: kazda wymiana konczyla sie timeoutem.
        Serial.println();

        String str;
        // 38 (prefiks) + do 6 cyfr licznika + "][P" + 2 cyfry mocy + 51 (sufiks)
        // = do 100 znakow. Rezerwa musi pokrywac calosc, inaczej realokacja przy
        // kazdej wiadomosci tylko podnosi szczyt zuzycia sterty.
        str.reserve(100);
        str += F("Hello World from sender to receiver [#");
        str += count++;
        str += F("][P"); // znacznik APC: moc, z jaka ta wiadomosc jest nadawana
        str += (int) manager->getEffectiveTxPower(); // moc FAKTYCZNEGO nadania; int8_t bez rzutu trafilby w concat(char)
        str += F("] with ACK | test string 1234567890ABCDEFGHIJKLMNOP");
        Serial.print(F("Sending payload: \""));
        Serial.print(str);
        Serial.println(F("\""));
        // Ruch testowy idzie przez mesh: trasa (takze wieloskokowa) wybierana
        // automatycznie z tablicy tras budowanej z beaconow.
        bool queued = mesh->send(actualDest, str,
                      []() {
                          Serial.println(F("MAIN | OK"));
                      },
                      [](String &payload) {
                          Serial.print(F("MAIN | NOT OK, payload = "));
                          Serial.println(payload);
                      });
        if (!queued) Serial.println(F("MAIN | send pominiety (brak trasy albo radio zajete)"));

        actualDest++;
        if (actualDest == NODE_ID) actualDest++;
        if (actualDest > maxNodes) {
            actualDest = 1;
            if (actualDest == NODE_ID) actualDest++;
        }

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

