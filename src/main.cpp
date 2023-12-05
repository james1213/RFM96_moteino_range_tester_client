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


void setTransmittedFlag(void);

void setup() {
    Serial.begin(115200);

    // initialize SX1278 with default settings
    Serial.print(F("[SX1278] Initializing ... "));
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

    // set the function that will be called
    // when packet transmission is finished
    radio.setPacketSentAction(setTransmittedFlag);





    // set node address to 0x02
    state = radio.setNodeAddress(0x01);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.println(F("[SX1278] Unable to set node address, code "));
        Serial.println(state);
    }
    // set broadcast address to 0xFF
    state = radio.setBroadcastAddress(0xFF);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.println(F("[SX1278] Unable to set broadcast address, code "));
        Serial.println(state);
    }




   ////////////////////////

    uint8_t syncWord[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
    state = radio.setSyncWord(syncWord, 8);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("Unable to set configuration, code "));
        Serial.println(state);
        while (true);
    }
//
//    state = radio.setOOK(true);
//    if (state != RADIOLIB_ERR_NONE) {
//        Serial.print(F("Unable to change modulation OOK, code "));
//        Serial.println(state);
//        while (true);
//    }
//    state = radio.setDataShapingOOK(1);
//    if (state != RADIOLIB_ERR_NONE) {
//        Serial.print(F("Unable to change data shaping, code "));
//        Serial.println(state);
//        while (true);
//    }

    state = radio.fixedPacketLengthMode(64);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("Unable to change fixedPacketLengthMode, code "));
        Serial.println(state);
        while (true);
    }

/////////////////////////////////////




    // start transmitting the first packet
    Serial.print(F("[SX1278] Sending first packet ... "));

    // you can transmit C-string or Arduino string up to
    // 255 characters long
    transmissionState = radio.startTransmit("Hello World`", 0x02);

    // you can also transmit byte array up to 255 bytes long
    /*
      byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
      transmissionState = radio.startTransmit(byteArr, 8);
    */
}

// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setTransmittedFlag(void) {
    // we sent a packet, set the flag
    transmittedFlag = true;
}

// counter to keep track of transmitted packets
int count = 0;

void loop() {
    // check if the previous transmission finished
    if(transmittedFlag) {
        // reset flag
        transmittedFlag = false;

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

        // wait a second before transmitting again
        delay(1000);

        // send another one
        Serial.print(F("[SX1278] Sending another packet ... "));

        // you can transmit C-string or Arduino string up to
        // 255 characters long
        String str = "Hello World [#" + String(count++) + "]`";


        unsigned long startTime = micros();
        transmissionState = radio.startTransmit(str, 0x02);
        Serial.println("radio.startTransmit() time: " + String(micros() - startTime) + " us");

        // you can also transmit byte array up to 255 bytes long
        /*
          byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                            0x89, 0xAB, 0xCD, 0xEF};
          transmissionState = radio.startTransmit(byteArr, 8);
        */
    }
}