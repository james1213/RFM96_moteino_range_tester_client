///*
//  RadioLib SX127x Transmit with Interrupts Example
//
//  This example transmits LoRa packets with one second delays
//  between them. Each packet contains up to 255 bytes
//  of data, in the form of:
//  - Arduino String
//  - null-terminated char array (C-string)
//  - arbitrary binary data (byte array)
//
//  Other modules from SX127x/RFM9x family can also be used.
//
//  For default module settings, see the wiki page
//  https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem
//
//  For full API reference, see the GitHub Pages
//  https://jgromes.github.io/RadioLib/
//*/
//
//// include the library
//#include <Arduino.h>
//#include <RadioLib.h>
//#include "SSD1306Ascii.h"
//#include "SSD1306AsciiWire.h"
//
//// RF96 has the following connections:
//// CS pin:    10
//// DIO0 pin:  2
//// RESET pin: 3 - ja mam gdzie indziej podłączony
//// TODO - sprawdzić gdzie mam podłączone piny od SPI, czy domyślnie
//RFM96 radio = new Module(10, 2, 3);
//
//
//
//// or using RadioShield
//// https://github.com/jgromes/RadioShield
////SX1278 radio = RadioShield.ModuleA;
//
//// save transmission state between loops
//int transmissionState = RADIOLIB_ERR_NONE;
//
//
//void setTransmittedFlag(void);
//
//void setup() {
//    Serial.begin(115200);
//
//    // initialize SX1278 with default settings
//    Serial.print(F("[SX1278] Initializing ... "));
//    int state = radio.begin();
//    if (state == RADIOLIB_ERR_NONE) {
//        Serial.println(F("success!"));
//        Serial.print(F("Chip version: ")); Serial.println(radio.getChipVersion());
//    } else {
//        Serial.print(F("failed, code "));
//        Serial.println(state);
//        Serial.print(F("Chip version: ")); Serial.println(radio.getChipVersion());
//        while (true);
//    }
//
//    // set the function that will be called
//    // when packet transmission is finished
//    radio.setPacketSentAction(setTransmittedFlag);
//
//    // start transmitting the first packet
//    Serial.print(F("[SX1278] Sending first packet ... "));
//
//    // you can transmit C-string or Arduino string up to
//    // 255 characters long
//    transmissionState = radio.startTransmit("Hello World!");
//
//    // you can also transmit byte array up to 255 bytes long
//    /*
//      byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
//                        0x89, 0xAB, 0xCD, 0xEF};
//      transmissionState = radio.startTransmit(byteArr, 8);
//    */
//}
//
//// flag to indicate that a packet was sent
//volatile bool transmittedFlag = false;
//
//// this function is called when a complete packet
//// is transmitted by the module
//// IMPORTANT: this function MUST be 'void' type
////            and MUST NOT have any arguments!
//#if defined(ESP8266) || defined(ESP32)
//ICACHE_RAM_ATTR
//#endif
//void setTransmittedFlag(void) {
//    // we sent a packet, set the flag
//    transmittedFlag = true;
//}
//
//// counter to keep track of transmitted packets
//int count = 0;
//
//void loop() {
//    // check if the previous transmission finished
//    if(transmittedFlag) {
//        // reset flag
//        transmittedFlag = false;
//
//        if (transmissionState == RADIOLIB_ERR_NONE) {
//            // packet was successfully sent
//            Serial.println(F("transmission finished!"));
//
//            // NOTE: when using interrupt-driven transmit method,
//            //       it is not possible to automatically measure
//            //       transmission data rate using getDataRate()
//
//        } else {
//            Serial.print(F("failed, code "));
//            Serial.println(transmissionState);
//
//        }
//
//        // clean up after transmission is finished
//        // this will ensure transmitter is disabled,
//        // RF switch is powered down etc.
//        radio.finishTransmit();
//
//        // wait a second before transmitting again
//        delay(1000);
//
//        // send another one
//        Serial.print(F("[SX1278] Sending another packet ... "));
//
//        // you can transmit C-string or Arduino string up to
//        // 255 characters long
//        String str = "Hello World! #" + String(count++);
//        unsigned long startTime = micros();
//        transmissionState = radio.startTransmit(str);
//        Serial.println("radio.startTransmit() time: " + String(micros() - startTime) + " us");
//
//        // you can also transmit byte array up to 255 bytes long
//        /*
//          byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
//                            0x89, 0xAB, 0xCD, 0xEF};
//          transmissionState = radio.startTransmit(byteArr, 8);
//        */
//    }
//}



////With ACK
///*
//  RadioLib SX127x Transmit with Interrupts Example
//
//  This example transmits LoRa packets with one second delays
//  between them. Each packet contains up to 255 bytes
//  of data, in the form of:
//  - Arduino String
//  - null-terminated char array (C-string)
//  - arbitrary binary data (byte array)
//
//  Other modules from SX127x/RFM9x family can also be used.
//
//  For default module settings, see the wiki page
//  https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem
//
//  For full API reference, see the GitHub Pages
//  https://jgromes.github.io/RadioLib/
//*/
//
//// include the library
//#include <Arduino.h>
//#include <RadioLib.h>
//#include "SSD1306Ascii.h"
//#include "SSD1306AsciiWire.h"
//#include "RFM96Wrapper.h"
//
//// RF96 has the following connections:
//// CS pin:    10
//// DIO0 pin:  2
//// RESET pin: 3 - ja mam gdzie indziej podłączony
//// TODO - sprawdzić gdzie mam podłączone piny od SPI, czy domyślnie
////RFM96 radio = new Module(10, 2, 3);
//RFM96 radio = new Module(10, 2, 3);
//
//
//
//// or using RadioShield
//// https://github.com/jgromes/RadioShield
////SX1278 radio = RadioShield.ModuleA;
//
//// save transmission state between loops
//int transmissionState = RADIOLIB_ERR_NONE;
//
//
//void setTransmittedFlag(void);
//void setReceivedFlag(void);
//
//void setup() {
//    Serial.begin(115200);
//
//    // initialize SX1278 with default settings
//    Serial.print(F("[SX1278] Initializing ... "));
//    int state = radio.begin();
//    if (state == RADIOLIB_ERR_NONE) {
//        Serial.println(F("success!"));
//        Serial.print(F("Chip version: ")); Serial.println(radio.getChipVersion());
//    } else {
//        Serial.print(F("failed, code "));
//        Serial.println(state);
//        Serial.print(F("Chip version: ")); Serial.println(radio.getChipVersion());
//        while (true);
//    }
//
//    // set the function that will be called
//    // when packet transmission is finished
//    radio.setPacketSentAction(setTransmittedFlag);
//
//    // set the function that will be called
//    // when new packet is received
//    radio.setPacketReceivedAction(setReceivedFlag);
//
//
//
//
//    // start listening for LoRa packets
//    Serial.print(F("[SX1278] Starting to listen ... "));
//    state = radio.startReceive();
//    if (state == RADIOLIB_ERR_NONE) {
//        Serial.println(F("success!"));
//    } else {
//        Serial.print(F("failed, code "));
//        Serial.println(state);
//        while (true);
//    }
//
//
//
//
//
//    // start transmitting the first packet
//    Serial.print(F("[SX1278] Sending first packet ... "));
//
//    // you can transmit C-string or Arduino string up to
//    // 255 characters long
//    transmissionState = radio.startTransmit("Hello World!");
//
//    // you can also transmit byte array up to 255 bytes long
//    /*
//      byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
//                        0x89, 0xAB, 0xCD, 0xEF};
//      transmissionState = radio.startTransmit(byteArr, 8);
//    */
//}
//
//// flag to indicate that a packet was sent
//volatile bool transmittedFlag = false;
//volatile bool receivedFlag = false;
//
//// this function is called when a complete packet
//// is transmitted by the module
//// IMPORTANT: this function MUST be 'void' type
////            and MUST NOT have any arguments!
//#if defined(ESP8266) || defined(ESP32)
//ICACHE_RAM_ATTR
//#endif
//void setTransmittedFlag(void) {
//    // we sent a packet, set the flag
//    transmittedFlag = true;
//}
//
//void setReceivedFlag(void) {
//    // we sent a packet, set the flag
//    receivedFlag = true;
//}
//
//// counter to keep track of transmitted packets
//int count = 0;
//
//void loop() {
//    // check if the previous transmission finished
//    if(transmittedFlag) {
//        // reset flag
//        transmittedFlag = false;
//
//        if (transmissionState == RADIOLIB_ERR_NONE) {
//            // packet was successfully sent
//            Serial.println(F("transmission finished!"));
//
//            // NOTE: when using interrupt-driven transmit method,
//            //       it is not possible to automatically measure
//            //       transmission data rate using getDataRate()
//
//        } else {
//            Serial.print(F("failed, code "));
//            Serial.println(transmissionState);
//
//        }
//
//        // clean up after transmission is finished
//        // this will ensure transmitter is disabled,
//        // RF switch is powered down etc.
//        radio.finishTransmit();
//
//        // wait a second before transmitting again
//        delay(1000);
//
//        // send another one
//        Serial.print(F("[SX1278] Sending another packet ... "));
//
//        // you can transmit C-string or Arduino string up to
//        // 255 characters long
//        String str = "Hello World! #" + String(count++);
//        unsigned long startTime = micros();
//        transmissionState = radio.startTransmit(str);
//        Serial.println("radio.startTransmit() time: " + String(micros() - startTime) + " us");
//
//        // you can also transmit byte array up to 255 bytes long
//        /*
//          byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
//                            0x89, 0xAB, 0xCD, 0xEF};
//          transmissionState = radio.startTransmit(byteArr, 8);
//        */
//    }
//
//
//    if(receivedFlag) {
//        // reset flag
//        receivedFlag = false;
//
//        // you can read received data as an Arduino String
//        String str;
//        int state = radio.readData(str);
//
//        // you can also read received data as byte array
//        /*
//          byte byteArr[8];
//          int numBytes = radio.getPacketLength();
//          int state = radio.readData(byteArr, numBytes);
//        */
//
//        if (state == RADIOLIB_ERR_NONE) {
//            // packet was successfully received
//            Serial.println(F("[SX1278] Received packet!"));
//
//            // print data of the packet
//            Serial.print(F("[SX1278] Data:\t\t"));
//            Serial.println(str);
//
//            // print RSSI (Received Signal Strength Indicator)
//            Serial.print(F("[SX1278] RSSI:\t\t"));
//            Serial.print(radio.getRSSI());
//            Serial.println(F(" dBm"));
//
//            // print SNR (Signal-to-Noise Ratio)
//            Serial.print(F("[SX1278] SNR:\t\t"));
//            Serial.print(radio.getSNR());
//            Serial.println(F(" dB"));
//
//            // print frequency error
//            Serial.print(F("[SX1278] Frequency error:\t"));
//            Serial.print(radio.getFrequencyError());
//            Serial.println(F(" Hz"));
//
//        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
//            // packet was received, but is malformed
//            Serial.println(F("[SX1278] CRC error!"));
//
//        } else {
//            // some other error occurred
//            Serial.print(F("[SX1278] Failed, code "));
//            Serial.println(state);
//
//        }
//    }
//}




















////With adressing and FSK
///*
//  RadioLib SX127x Transmit with Interrupts Example
//
//  This example transmits LoRa packets with one second delays
//  between them. Each packet contains up to 255 bytes
//  of data, in the form of:
//  - Arduino String
//  - null-terminated char array (C-string)
//  - arbitrary binary data (byte array)
//
//  Other modules from SX127x/RFM9x family can also be used.
//
//  For default module settings, see the wiki page
//  https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem
//
//  For full API reference, see the GitHub Pages
//  https://jgromes.github.io/RadioLib/
//*/
//
//// include the library
//#include <Arduino.h>
//#include <RadioLib.h>
//#include "SSD1306Ascii.h"
//#include "SSD1306AsciiWire.h"
//#include "RFM96Wrapper.h"
//
//// RF96 has the following connections:
//// CS pin:    10
//// DIO0 pin:  2
//// RESET pin: 3 - ja mam gdzie indziej podłączony
//// TODO - sprawdzić gdzie mam podłączone piny od SPI, czy domyślnie
////RFM96 radio = new Module(10, 2, 3);
//RFM96 radio = new Module(10, 2, 3);
//
//
//
//// or using RadioShield
//// https://github.com/jgromes/RadioShield
////SX1278 radio = RadioShield.ModuleA;
//
//// save transmission state between loops
//int transmissionState = RADIOLIB_ERR_NONE;
//#define NODE_ID_TO_SEND 2
//#define NODE_ID 1
//
//
//void setTransmittedFlag(void);
////void setReceivedFlag(void);
//
//void setup() {
//    Serial.begin(115200);
//
//    // initialize SX1278 with default settings
//    Serial.print(F("[SX1278] Initializing ... "));
////    int state = radio.begin();
//    int state = radio.beginFSK();
//    if (state == RADIOLIB_ERR_NONE) {
//        Serial.println(F("success!"));
//        Serial.print(F("Chip version: ")); Serial.println(radio.getChipVersion());
//    } else {
//        Serial.print(F("failed, code "));
//        Serial.println(state);
//        Serial.print(F("Chip version: ")); Serial.println(radio.getChipVersion());
//        while (true);
//    }
//
//    // set node address to 0x02
//    state = radio.setNodeAddress(NODE_ID);
//    // set broadcast address to 0xFF
//    state = radio.setBroadcastAddress(0xFF);
//    if (state != RADIOLIB_ERR_NONE) {
//        Serial.println(F("[SX1278] Unable to set address filter, code "));
//        Serial.println(state);
//    }
//
//
//
//
//
//
////    // the following settings can also
////    // be modified at run-time
////    state = radio.setFrequency(433.5);
////    state = radio.setBitRate(100.0);
////    state = radio.setFrequencyDeviation(10.0);
////    state = radio.setRxBandwidth(250.0);
////    state = radio.setOutputPower(10.0);
////    state = radio.setCurrentLimit(100);
////    state = radio.setDataShaping(RADIOLIB_SHAPING_0_5);
////    uint8_t syncWord[] = {0x01, 0x23, 0x45, 0x67,
////                          0x89, 0xAB, 0xCD, 0xEF};
////    state = radio.setSyncWord(syncWord, 8);
////    if (state != RADIOLIB_ERR_NONE) {
////        Serial.print(F("Unable to set configuration, code "));
////        Serial.println(state);
////        while (true);
////    }
//
//
//
//
//    // set the function that will be called
//    // when packet transmission is finished
//    radio.setPacketSentAction(setTransmittedFlag);
//
//    // set the function that will be called
//    // when new packet is received
////    radio.setPacketReceivedAction(setReceivedFlag);
//
//
//
//
////    // start listening for LoRa packets
////    Serial.print(F("[SX1278] Starting to listen ... "));
////    state = radio.startReceive();
////    if (state == RADIOLIB_ERR_NONE) {
////        Serial.println(F("success!"));
////    } else {
////        Serial.print(F("failed, code "));
////        Serial.println(state);
////        while (true);
////    }
//
//
//
//
//
//    // start transmitting the first packet
//    Serial.print(F("[SX1278] Sending first packet ... "));
//
//    // you can transmit C-string or Arduino string up to
//    // 255 characters long
//    transmissionState = radio.startTransmit("Hello World!", NODE_ID_TO_SEND);
//
//    // you can also transmit byte array up to 255 bytes long
//    /*
//      byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
//                        0x89, 0xAB, 0xCD, 0xEF};
//      transmissionState = radio.startTransmit(byteArr, 8);
//    */
//}
//
//// flag to indicate that a packet was sent
//volatile bool transmittedFlag = false;
////volatile bool receivedFlag = false;
//
//// this function is called when a complete packet
//// is transmitted by the module
//// IMPORTANT: this function MUST be 'void' type
////            and MUST NOT have any arguments!
//#if defined(ESP8266) || defined(ESP32)
//ICACHE_RAM_ATTR
//#endif
//void setTransmittedFlag(void) {
//    // we sent a packet, set the flag
//    transmittedFlag = true;
//}
//
////void setReceivedFlag(void) {
////    // we sent a packet, set the flag
////    receivedFlag = true;
////}
//
//// counter to keep track of transmitted packets
//int count = 0;
//
//void loop() {
//    // check if the previous transmission finished
//    if(transmittedFlag) {
//        // reset flag
//        transmittedFlag = false;
//
//        if (transmissionState == RADIOLIB_ERR_NONE) {
//            // packet was successfully sent
//            Serial.println(F("transmission finished!"));
//
//            // NOTE: when using interrupt-driven transmit method,
//            //       it is not possible to automatically measure
//            //       transmission data rate using getDataRate()
//
//        } else {
//            Serial.print(F("failed, code "));
//            Serial.println(transmissionState);
//
//        }
//
//        // clean up after transmission is finished
//        // this will ensure transmitter is disabled,
//        // RF switch is powered down etc.
//        radio.finishTransmit();
//
//        // wait a second before transmitting again
//        delay(1000);
//
//        // send another one
//        Serial.print(F("[SX1278] Sending another packet ... "));
//
//        // you can transmit C-string or Arduino string up to
//        // 255 characters long
//        String str = "Hello! #[" + String(count++) + "]";
//        Serial.print(F("["));
//        Serial.print(str);
//        Serial.print(F("]"));
//        unsigned long startTime = micros();
//        transmissionState = radio.startTransmit(str, NODE_ID_TO_SEND);
//        Serial.println("radio.startTransmit() time: " + String(micros() - startTime) + " us");
//
//        // you can also transmit byte array up to 255 bytes long
//        /*
//          byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
//                            0x89, 0xAB, 0xCD, 0xEF};
//          transmissionState = radio.startTransmit(byteArr, 8);
//        */
//    }
//
//
////    if(receivedFlag) {
////        // reset flag
////        receivedFlag = false;
////
////        // you can read received data as an Arduino String
////        String str;
////        int state = radio.readData(str);
////
////        // you can also read received data as byte array
////        /*
////          byte byteArr[8];
////          int numBytes = radio.getPacketLength();
////          int state = radio.readData(byteArr, numBytes);
////        */
////
////        if (state == RADIOLIB_ERR_NONE) {
////            // packet was successfully received
////            Serial.println(F("[SX1278] Received packet!"));
////
////            // print data of the packet
////            Serial.print(F("[SX1278] Data:\t\t"));
////            Serial.println(str);
////
////            // print RSSI (Received Signal Strength Indicator)
////            Serial.print(F("[SX1278] RSSI:\t\t"));
////            Serial.print(radio.getRSSI());
////            Serial.println(F(" dBm"));
////
////            // print SNR (Signal-to-Noise Ratio)
////            Serial.print(F("[SX1278] SNR:\t\t"));
////            Serial.print(radio.getSNR());
////            Serial.println(F(" dB"));
////
////            // print frequency error
////            Serial.print(F("[SX1278] Frequency error:\t"));
////            Serial.print(radio.getFrequencyError());
////            Serial.println(F(" Hz"));
////
////        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
////            // packet was received, but is malformed
////            Serial.println(F("[SX1278] CRC error!"));
////
////        } else {
////            // some other error occurred
////            Serial.print(F("[SX1278] Failed, code "));
////            Serial.println(state);
////
////        }
////    }
//}













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