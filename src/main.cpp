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


void setInterruptionFlag(void);
void send(String &str, uint8_t address);
void bufferedSend(String &str);

void setupRadio();

void sendLoop();

String readReceivedData();
void startListening();

void receiveLoop();

void dataReceived(const String &str);

//void waitForAckLoop();

void setup() {
    Serial.begin(115200);
    setupRadio();
    startListening();

    // start transmitting the first packet
//    Serial.print(F("[RFM96] Sending first packet ... "));

//    String helloString = "Hello World";
//    bufferedSend(helloString);
}

// flag to indicate that a packet was sent
volatile bool transmissionFinished = true;
volatile bool transmissionClenedUp = true;
volatile bool receivedFlag = false;
volatile bool ackReceived = false;
//bool waitingForAck = false;
//unsigned long waitForAckStartTime = 0;
//unsigned long ackTimeout = 3000; //ms

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setInterruptionFlag(void) {
    if (!transmissionFinished) {
        Serial.println("SENDING TIME = " + String(micros() - sendingTime) + " us");
        // we sent a packet, set the flag
        transmissionFinished = true;
    } else {
        // we got a packet, set the flag
        receivedFlag = true;
    }
}

// counter to keep track of transmitted packets
int count = 0;

void loop() {
    sendLoop();

    if (millis() % 1000 == 0) {
        Serial.println();
        String str = "Hello World [#" + String(count++) + "]";
        bufferedSend(str);
    }
    receiveLoop();

//    delay(1000);
}

void receiveLoop() {// check if the flag is set
    if (receivedFlag) {
        // reset flag
        receivedFlag = false;
        String str = readReceivedData();
        startListening();
        if (str.equals("!")) {
            Serial.println(F("Received ACK"));
            Serial.println("RECEIVED ACK TIME = " + String(micros() - sendingTime) + " us");
            ackReceived = true;
        } else {
            dataReceived(str);
            //TODO wysłanie ack
            String ackString = "!";
            bufferedSend(ackString);
        }
    }
}

void dataReceived(const String &str) { Serial.println("Received data: \"" + str + "\""); }

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
    // or
    // set the function that will be called when new packet is received
    radio.setChannelScanAction(setInterruptionFlag);


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
            startListening();
        } else if (!sendBuffer.equals("")) {
//            if (ackReceived) {
                Serial.print(F("[RFM96] Sending another packet ... "));
                transmissionFinished = false;
                transmissionClenedUp = false;
                send(sendBuffer, NODE_ID_TO_SEND);
                sendBuffer = "";
//            } else {
//                Serial.println("Can not send next data until ACK from previous data is received");
//            }
        }
    }
}

void send(String &str, uint8_t address) {
    Serial.println("Sending: [" + str + "]");;
    unsigned long startTime = micros();
    str = str + "`";
    sendingTime = micros();
    transmissionState = radio.startTransmit(str, address);
    Serial.println("radio.startTransmit() time: " + String(micros() - startTime) + " us");
}

String readReceivedData() {// you can read received data as an Arduino String
    String str;
    int state = radio.readData(str);

//        str.remove(str.lastIndexOf("`"));
    str.remove(str.indexOf("`"));

    // you can also read received data as byte array
/*
  byte byteArr[8];
  int numBytes = radio.getPacketLength();
  int state = radio.readData(byteArr, numBytes);
*/

    if (state == RADIOLIB_ERR_NONE) {
        // packet was successfully received
        Serial.println();
        Serial.println(F("[RFM96] Received packet!"));

        // print data of the packet
        Serial.print(F("[RFM96] Data:\t\t"));
        Serial.println(str);

        // print RSSI (Received Signal Strength Indicator)
        Serial.print(F("[RFM96] RSSI:\t\t"));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));

        // print SNR (Signal-to-Noise Ratio)
        Serial.print(F("[RFM96] SNR:\t\t"));
        Serial.print(radio.getSNR());
        Serial.println(F(" dB"));

        // print frequency error
        Serial.print(F("[RFM96] Frequency error:\t"));
        Serial.print(radio.getFrequencyError());
        Serial.println(F(" Hz"));

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        // packet was received, but is malformed
        str = "";
        Serial.println(F("[RFM96] CRC error!"));
    } else {
        // some other error occurred
        str = "";
        Serial.print(F("[RFM96] Failed, code "));
        Serial.println(state);
    }
    return str;
}

void startListening() {// start listening for LoRa packets
    Serial.print(F("[RFM96] Starting to listen ... "));
    int state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }
}
