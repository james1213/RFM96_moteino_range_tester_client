//FSK
/*
  RadioLib SX127x Transmit with Interrupts Example

  This example transmits LoRa packets with one second delays
  between them. Each packet contains up to 255 bytes
  of data, in the form of:
  - Arduino String
  - null-terminated char array (C-string)
  - arbitrary binary data (byte array)

  Other modules from SX127x/RFM9x family can also be used.

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/

// include the library
#include <Arduino.h>
#include <RadioLib.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"


#define NODE_ID 0x01
#define NODE_ID_BROADCAST 0xFF
#define NODE_ID_TO_SEND 0x02

// RF96 has the following connections:
// CS pin:    10
// DIO0 pin:  2
// RESET pin: 3 - ja mam gdzie indziej podłączony
// TODO - sprawdzić gdzie mam podłączone piny od SPI, czy domyślnie
RFM96 radio = new Module(10, 2, 3);



// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1278 radio = RadioShield.ModuleA;

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

String sendBuffer = "";

unsigned long sendingTime = 0;


void setTransmittedFlag(void);

void send(String &str, uint8_t address);
void bufferedSend(String &str);

void setupRadio();

void sendLoop();

void setup() {
    Serial.begin(115200);
    setupRadio();

    // start transmitting the first packet
    Serial.print(F("[RFM96] Sending first packet ... "));

    String helloString = "Hello World";
    bufferedSend(helloString);
}

// flag to indicate that a packet was sent
volatile bool transmissionFinished = true;
volatile bool transmissionClenedUp = true;

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setTransmittedFlag(void) {
    Serial.println("SENDING TIME = " + String(micros() - sendingTime) + " us");
    // we sent a packet, set the flag
    transmissionFinished = true;
}

// counter to keep track of transmitted packets
int count = 0;

void loop() {
    sendLoop();

    String str = "Hello World [#" + String(count++) + "]";
    bufferedSend(str);
    delay(1000);
}

void sendLoop() {
// check if the previous transmission finished
    if (transmissionFinished) {
        if (!transmissionClenedUp) {
            transmissionClenedUp = true;

            if (transmissionState == RADIOLIB_ERR_NONE) {
                // packet was successfully sent
                Serial.println(F("transmission finished!"));

                // NOTE: when using interrupt-driven transmit method,
                //       it is not possible to automatically measure
                //       transmission data rate using getDataRate()

            } else {
                Serial.print(F("failed, code "));
                Serial.println(transmissionState);

            }

            // clean up after transmission is finished
            // this will ensure transmitter is disabled,
            // RF switch is powered down etc.
            radio.finishTransmit();
        } else if (!sendBuffer.equals("")) {
            Serial.print(F("[RFM96] Sending another packet ... "));
            transmissionFinished = false;
            transmissionClenedUp = false;
            send(sendBuffer, NODE_ID_TO_SEND);
            sendBuffer = "";
        }
    }
}

void setupRadio() {// initialize SX1278 with default settings
    Serial.print(F("[RFM96] Initializing ... "));

    int state = radio.beginFSK();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
        Serial.print(F("Chip version: ")); Serial.println(radio.getChipVersion());
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        Serial.print(F("Chip version: ")); Serial.println(radio.getChipVersion());
        while (true);
    }

    // set the function that will be called when packet transmission is finished
    radio.setPacketSentAction(setTransmittedFlag);

    // set node address to 0x02
    state = radio.setNodeAddress(NODE_ID);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.println(F("[RFM96] Unable to set node address, code "));
        Serial.println(state);
    }

    // set broadcast address to 0xFF
    state = radio.setBroadcastAddress(NODE_ID_BROADCAST);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.println(F("[RFM96] Unable to set broadcast address, code "));
        Serial.println(state);
    }

    uint8_t syncWord[] = {0x01, 0x23, 0x45, 0x67,
                          0x89, 0xAB, 0xCD, 0xEF};
    state = radio.setSyncWord(syncWord, 8);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("Unable to set configuration, code "));
        Serial.println(state);
        while (true);
    }

    state = radio.fixedPacketLengthMode(64);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("Unable to change fixedPacketLengthMode, code "));
        Serial.println(state);
        while (true);
    }
}

void bufferedSend(String &str) {
    sendBuffer = str;
    sendLoop();
}

void send(String &str, uint8_t address) {
    Serial.println("Sending: [" + str + "]");;
    unsigned long startTime = micros();
    str = str + "`";
    sendingTime = micros();
    transmissionState = radio.startTransmit(str, address);
    Serial.println("radio.startTransmit() time: " + String(micros() - startTime) + " us");
}
