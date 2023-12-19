/*
  LoRa Simple Gateway/Node Exemple

  This code uses InvertIQ function to create a simple Gateway/Node logic.

  Gateway - Sends messages with enableInvertIQ()
          - Receives messages with disableInvertIQ()

  Node    - Sends messages with disableInvertIQ()
          - Receives messages with enableInvertIQ()

  With this arrangement a Gateway never receive messages from another Gateway
  and a Node never receive message from another Node.
  Only Gateway to Node and vice versa.

  This code receives messages and sends a message every second.

  InvertIQ function basically invert the LoRa I and Q signals.

  See the Semtech datasheet, http://www.semtech.com/images/datasheet/sx1276.pdf
  for more on InvertIQ register 0x33.

  created 05 August 2018
  by Luiz H. Cassettari
*/

#include <Arduino.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>
//#include "RadioManager/RadioManager.h"

#define NODE_ID 0x01
#define NODE_ID_BROADCAST 0xFF
#define NODE_ID_TO_SEND 0x02

volatile bool transmissionFinished = true;
volatile bool transmissionClenedUp = true;
volatile bool receivedFlag = false;
volatile bool ackReceived = false;
bool waitingForAck = false;
unsigned long waitForAckStartTime = 0;
unsigned long ackTimeout = 1000; //ms

String ackCallback_paylod = "";
void (*ackNotReceivedCallback)(String &payload);
void (*ackReceivedCallback)();

//int transmissionState = RADIOLIB_ERR_NONE;

String sendBuffer = "";
uint8_t messageId = 0;
uint8_t destinationAddress = 0;
uint8_t destinationIdOfLastMessage = 0;
uint8_t senderIdOfLastMessage = 0;
uint8_t receivedMessageIdOfLastMessage = 0;

unsigned long sendingTime = 0;

// counter to keep track of transmitted packets
int count = 0;

void LoRa_rxMode();
void LoRa_txMode();
void LoRa_sendMessage(String message);
void onReceive(int packetSize);
void onTxDone();
boolean runEvery(unsigned long interval);
void setupRadio();
void setupSerial();
void radioLoop();
void sendLoop();
void receiveLoop();
void waitForAckTimeoutLoop();

void bufferedSendAndWaitForAck(String &str, uint8_t address, void (*_ackReceivedCallback)(),
                               void (*_ackNotReceivedCallback)(String &payload));
void bufferedSend(String &str, uint8_t address);
void send(String &str, uint8_t address);
void extractMessageIdAndSenderIdAndDestinationIdFromReceivedData(String &str);
bool isAckPayloadAndValidMessageId(String str);
void dataReceived(const String &str);
bool isAckPayload(String str);
int splitString(String &text, String *texts, char ch);

void setup() {
    setupSerial();
    setupRadio();
}

void setupSerial() {
    Serial.begin(115200);                   // initialize serial
    while (!Serial);
}

void setupRadio() {
//    resetRadio();
//    LoRa.setPins(10, 3, 2);
    LoRa.setPins(10, 7, 2);
    if (!LoRa.begin(433E6)) {
        Serial.println("LoRa init failed. Check your connections.");
        while (true);                       // if failed, do nothing
    }

    Serial.println("LoRa init succeeded.");
    Serial.println();
    Serial.println("LoRa Simple Node");
    Serial.println("Only receive messages from gateways");
    Serial.println("Tx: invertIQ disable");
    Serial.println("Rx: invertIQ enable");
    Serial.println();

    LoRa.onReceive(onReceive);
    LoRa.onTxDone(onTxDone);
    LoRa_rxMode();
}

//void resetRadio() {// Hard Reset the RFM module
//    Serial.println(F("[RADIO] resetting radio"));
//    pinMode(RFM69_RST, OUTPUT);
//    digitalWrite(RFM69_RST, HIGH);
//    delay(100);
//    digitalWrite(RFM69_RST, LOW);
//    delay(100);
//    digitalWrite(RFM69_RST, HIGH);
//    delay(100);
//    Serial.println(F("[RADIO] radio resetted"));
//}

void loop() {
    radioLoop();
    if (runEvery(2000)) { // repeat every 1000 millis


        Serial.println();
//        String str = "Hello World [#" + String(count++) + "] without ACK";
//        bufferedSend(str);

        String str = "Hello World [#" + String(count++) + "] with ACK";
        bufferedSendAndWaitForAck(str, NODE_ID_TO_SEND,
//        bufferedSendAndWaitForAckWithWaitingUntilPreviousAckIsReceived(str,
                                  []() {
                                      Serial.println(F("OK"));
                                  },
                                  [](String &payload) {
                                      Serial.print(F("NOT OK, payload = "));
                                      Serial.println(payload);
                                  });






//        String message = "HeLoRa World! ";
//        message += "I'm a Node! ";
//        message += millis();
//
//        LoRa_sendMessage(message); // send a message
//
//        Serial.println("Send Message!");
    }
}

void radioLoop() {
    sendLoop();
    receiveLoop();
    waitForAckTimeoutLoop();
}

void sendLoop() {
// check if the previous transmission finished
    if (transmissionFinished) {
        if (!transmissionClenedUp) {
            transmissionClenedUp = true;

//            if (transmissionState == RADIOLIB_ERR_NONE) {
                // packet was successfully sent
                Serial.println(F("transmission finished!"));

                // NOTE: when using interrupt-driven transmit method,
                //       it is not possible to automatically measure
                //       transmission data rate using getDataRate()

//            } else {
//                Serial.print(F("failed, code "));
//                Serial.println(transmissionState);
//
//            }

            // clean up after transmission is finished
            // this will ensure transmitter is disabled,
            // RF switch is powered down etc.

            //zamiast tego w przerwaniu mamy przełączenie na nasłuchiwanie
//            radio.finishTransmit();
//            startListening();
        } else if (!sendBuffer.equals("")) {
//            if (ackReceived) {
            Serial.print(F("[RFM96] Sending another packet ... "));
            transmissionFinished = false;
            transmissionClenedUp = false;
            send(sendBuffer, destinationAddress);
            sendBuffer = "";
//            } else {
//                Serial.println("Can not send next data until ACK from previous data is received");
//            }
        }
    }
}

void bufferedSendAndWaitForAck(String &str, uint8_t address, void (*_ackReceivedCallback)(),
                               void (*_ackNotReceivedCallback)(String &payload)) {
    ackReceivedCallback = _ackReceivedCallback;
    ackNotReceivedCallback = _ackNotReceivedCallback;
    ackCallback_paylod = str;
    waitingForAck = true;
    ackReceived = false;
    waitForAckStartTime = millis();
    bufferedSend(str, address);
}

void bufferedSend(String &str, uint8_t address) {
    sendBuffer = str;
    messageId++;
    destinationAddress = address;
    sendLoop();
}

void send(String &str, uint8_t address) {
//    Serial.println("Sending: [" + str + "] to " + address);
    Serial.print(F("Sending: ["));
    Serial.print(str);
    Serial.print(F("] to "));
    Serial.println(address);
    unsigned long startTime = micros();
    str = String(address) + "@" + String(NODE_ID) + "@" + String(messageId) + "@" + str + "`";
//    Serial.println("Transmitting str: [" + str + "]");
    Serial.print(F("Transmitting str: ["));
    Serial.print(str);
    Serial.println(F("]"));
    sendingTime = micros();
//    transmissionState = radio.startTransmit(str, address);
//    transmissionState = radio.startTransmit(str);
    LoRa_sendMessage(str);
//    Serial.println("radio.startTransmit() time: " + String(micros() - startTime) + " us");
    Serial.print(F("radio.startTransmit() time: "));
    Serial.print(String(micros() - startTime));
    Serial.println(F(" us"));
}

String readReceivedData() {// you can read received data as an Arduino String
    String str = "";
        while (LoRa.available()) {
            str += (char)LoRa.read();
    }

    str.remove(str.indexOf("`"));


        Serial.println();
        Serial.println(F("[RFM96] Received packet!"));

        // print data of the packet
        Serial.print(F("[RFM96] Data:\t\t"));
        Serial.println(str);

//        // print senderIdOfLastMessage
//        Serial.print(F("[RFM96] SendeId:\t\t"));
//        Serial.println(radio.getSenderId());

        // print RSSI (Received Signal Strength Indicator)
        Serial.print(F("[RFM96] RSSI:\t\t"));
        Serial.print(LoRa.packetRssi());
        Serial.println(F(" dBm"));

        // print SNR (Signal-to-Noise Ratio)
        Serial.print(F("[RFM96] SNR:\t\t"));
        Serial.print(LoRa.packetSnr());
        Serial.println(F(" dB"));

        // print frequency error
        Serial.print(F("[RFM96] Frequency error:\t"));
        Serial.print(LoRa.packetFrequencyError());
        Serial.println(F(" Hz"));

    return str;
}

void receiveLoop() {// check if the flag is set
    if (receivedFlag) {
        // reset flag
        receivedFlag = false;
        String str = readReceivedData();


        Serial.print(F("[ACK] | before extractMessageIdAndSenderIdAndDestinationIdFromReceivedData = "));
        Serial.println(str);
        extractMessageIdAndSenderIdAndDestinationIdFromReceivedData(str);
        Serial.print(F("[ACK] | extracted senderIdOfLastMessage = "));
        Serial.println(senderIdOfLastMessage);
        Serial.print(F("[ACK] | extracted receivedMessageIdOfLastMessage = "));
        Serial.println(receivedMessageIdOfLastMessage);
        Serial.print(F("[ACK] | after extractMessageIdAndSenderIdAndDestinationIdFromReceivedData = "));
        Serial.println(str);

//        startListening(); // tutaj chyba nie ma to sendu bo podcas odbioru zawsze słucha

        if (destinationIdOfLastMessage == NODE_ID) {
            Serial.println(F("This is a destination address"));
        } else {
            Serial.println(F("This is not a destination address, ignoring message"));
            return;
        }

//        if (str.equals("!")) {
        if (isAckPayloadAndValidMessageId(str) && waitingForAck) { //TODO powinno byc jeszce sprawdzanie messageID isAckPayloadAndValidMessageId(str)
            Serial.println(F("Received ACK"));
//            Serial.println("RECEIVED ACK TIME = " + String(micros() - sendingTime) + " us");
            Serial.print(F("RECEIVED ACK TIME = "));
            Serial.print(String(micros() - sendingTime));
            Serial.println(F(" us"));
            ackReceived = true;
            waitingForAck = false;
            if (ackReceivedCallback) {
                ackReceivedCallback();
            }
        } else {
            dataReceived(str);
            if (receivedMessageIdOfLastMessage != 0) {
                if (!isAckPayload(str)) {
                    Serial.print(F("Sending ACK to address: "));
                    Serial.println(senderIdOfLastMessage);
                    String ackString = "!";
                    ackString.concat(receivedMessageIdOfLastMessage);
                    bufferedSend(ackString, senderIdOfLastMessage);
                }
            } else {
                Serial.println(F("[ACK] | Can not extract message id from received message"));
            }
        }
    }
}

void extractMessageIdAndSenderIdAndDestinationIdFromReceivedData(String &str) { //TODO od razu powinno usuwać z tego stringa te dane
    //TODO czy jak sie tutaj usunię to wszędzie czy tutaj to będzie jednak kopia
    String splittedStr[4];
    int length = splitString(str, splittedStr, '@');
    if (length == 4) {
        destinationIdOfLastMessage = (uint8_t) splittedStr[0].toInt();
        senderIdOfLastMessage = (uint8_t) splittedStr[1].toInt();
        receivedMessageIdOfLastMessage = (uint8_t) splittedStr[2].toInt();
    }
}

// Example:
// String splittedData[4];
// int length = splitString(receivedText, splittedData, ',');
int splitString(String &text, String *texts, char ch) { // Split the string into substrings
    int stringCount = 0;
    while (text.length() > 0) {
        int index = text.indexOf(ch);
        if (index == -1) { // No space found
            texts[stringCount++] = text;
            break;
        } else {
            texts[stringCount++] = text.substring(0, index);
            text = text.substring(index + 1);
        }
    }
    return stringCount;
}

bool isAckPayload(String str) {
    Serial.print(F("isAckPayload, str = "));
    return str.charAt(0) == '!';
}

bool isAckPayloadAndValidMessageId(String str) {
    Serial.print(F("isAckPayloadAndValidMessageId, str = "));
    Serial.println(str);
    if (str.charAt(0) == '!') {
        str.remove(0,1);
        Serial.print(F("isAckPayloadAndValidMessageId, messageId = "));
        Serial.println(messageId);
        Serial.print(F("isAckPayloadAndValidMessageId, ((uint8_t) str.toInt()) = "));
        Serial.println(((uint8_t) str.toInt()));

        if (messageId == ((uint8_t) str.toInt())) {
            Serial.println(F("isAckPayloadAndValidMessageId, return true"));
            return true;
        }
    }
    Serial.println(F("isAckPayloadAndValidMessageId, return false"));
    return false;
}

void dataReceived(const String &str) {
//    Serial.println("Received data: \"" + str + "\"");
    Serial.print(F("Received data: \""));
    Serial.print(str);
    Serial.println(F("\""));
}

void waitForAckTimeoutLoop() {
    if (waitingForAck && !ackReceived) {
        if (millis() - waitForAckStartTime >= ackTimeout) {
            waitingForAck = false;
            Serial.println(F("ACK NOT RECEIVED - TIMEOUT"));
            if (ackNotReceivedCallback) {
                ackNotReceivedCallback(ackCallback_paylod);
            }
        }
    }
}

void LoRa_rxMode(){
//    LoRa.enableInvertIQ();                // node
    LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
    LoRa.idle();                          // set standby mode
//    LoRa.disableInvertIQ();               // node
}

void LoRa_sendMessage(String message) {
    LoRa_txMode();                        // set tx mode
    LoRa.beginPacket();                   // start packet
    LoRa.print(message);                  // add payload
    LoRa.endPacket(true);                 // finish packet and send it
}

void onReceive(int packetSize) {
    Serial.println(F("Received interrupt"));
    receivedFlag = true;


//    String message = "";
//
//    while (LoRa.available()) {
//        message += (char)LoRa.read();
//    }
//
//    Serial.print("Node Receive: ");
//    Serial.println(message);
}

void onTxDone() {
    Serial.println(F("Send interrupt"));
    Serial.print(F("SENDING TIME = "));
    Serial.print(String(micros() - sendingTime));
    Serial.println(F(" us"));
    transmissionFinished = true;
    Serial.println("TxDone");
    LoRa_rxMode();
}

boolean runEvery(unsigned long interval)
{
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis;
        return true;
    }
    return false;
}

