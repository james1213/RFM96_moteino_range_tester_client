//
// Created by LukaszLibront on 19.12.2023.
//

#include "RadioManager.h"

bool logActive = false;
volatile bool radioManager_transmissionFinished = true;
volatile bool radioManager_receivedFlag = false;
unsigned long radioManager_sendingTime = 0;
volatile bool transmissionClenedUp = true;
volatile bool ackReceived = false;
bool waitingForAck = false;
unsigned long waitForAckStartTime = 0;
unsigned long ackTimeout = 1000; //ms
String ackCallback_paylod = "";

void (*ackNotReceivedCallback)(String &payload);
void (*ackReceivedCallback)();
void (*dataReceivedCallback)(String &receivedText, uint8_t senderId);
void (*dataSentCallback)();

String sendBuffer = "";
uint8_t messageId = 0;
uint8_t destinationAddress = 0;
uint8_t destinationIdOfLastMessage = 0;
uint8_t senderIdOfLastMessage = 0;
uint8_t receivedMessageIdOfLastMessage = 0;
int receivedPacketSize = 0;


void radioManager_setupRadio() {
    LoRa.setPins(10, 7, 2);
    if (!LoRa.begin(433E6)) {
        radioManager_logln("LoRa init failed. Check your connections.");
        while (true);                       // if failed, do nothing
    }

    radioManager_logln("LoRa init succeeded.");
    radioManager_logln();
    radioManager_logln("LoRa Simple Node");
    radioManager_logln("Only receive messages from gateways");
    radioManager_logln("Tx: invertIQ disable");
    radioManager_logln("Rx: invertIQ enable");
    radioManager_logln();

    LoRa.onReceive(radioManager_onReceiveDone);
    LoRa.onTxDone(radioManager_onTxDone);
    radioManager_LoRa_rxMode();
}

void radioManager_onDataReceived(void(*callback)(String &receivedText, uint8_t senderId)) {
    dataReceivedCallback = callback;
}

void radioManager_onDataSent(void(*callback)()) {
    dataSentCallback = callback;
}

int getReceivedPacketSize() {
    return receivedPacketSize;
}

void radioManager_radioLoop() {
    radioManager_sendLoop();
    radioManager_receiveLoop();
    waitForAckTimeoutLoop();
}

void radioManager_sendLoop() {
    if (radioManager_transmissionFinished) {
        if (!transmissionClenedUp) {
            transmissionClenedUp = true;
            radioManager_logln(F("transmission finished!"));
        } else if (!sendBuffer.equals("")) {
            radioManager_log(F("[RFM96] Sending another packet ... "));
            radioManager_transmissionFinished = false;
            transmissionClenedUp = false;
            send(sendBuffer, destinationAddress);
            sendBuffer = "";
        }
    }
}

void radioManager_receiveLoop() {
    if (radioManager_receivedFlag) {
        radioManager_receivedFlag = false;
        String str = radioManager_readReceivedData();

        radioManager_log(F("[ACK] | before extractMessageIdAndSenderIdAndDestinationIdFromReceivedData = "));
        radioManager_logln(str);
        radioManager_extractMessageIdAndSenderIdAndDestinationIdFromReceivedData(str);
        radioManager_log(F("[ACK] | extracted senderIdOfLastMessage = "));
        radioManager_logln(senderIdOfLastMessage);
        radioManager_log(F("[ACK] | extracted receivedMessageIdOfLastMessage = "));
        radioManager_logln(receivedMessageIdOfLastMessage);
        radioManager_log(F("[ACK] | after extractMessageIdAndSenderIdAndDestinationIdFromReceivedData = "));
        radioManager_logln(str);

        if (destinationIdOfLastMessage == NODE_ID) {
            radioManager_logln(F("This is a destination address"));
        } else {
            radioManager_logln(F("This is not a destination address, ignoring message"));
            return;
        }

        if (isAckPayloadAndValidMessageId(str) && waitingForAck) {
            radioManager_logln(F("Received ACK"));
            radioManager_log(F("RECEIVED ACK TIME = "));
            radioManager_log(String(micros() - radioManager_sendingTime));
            radioManager_logln(F(" us"));
            ackReceived = true;
            waitingForAck = false;
            if (ackReceivedCallback) {
                ackReceivedCallback();
            }
        } else {
//            dataReceived(str);
            if (dataReceivedCallback) {
                dataReceivedCallback(str, senderIdOfLastMessage);
            }
            if (receivedMessageIdOfLastMessage != 0) {
                if (!isAckPayload(str)) {
                    radioManager_log(F("Sending ACK to address: "));
                    radioManager_logln(senderIdOfLastMessage);
                    String ackString = "!";
                    ackString.concat(receivedMessageIdOfLastMessage);
                    bufferedSend(ackString, senderIdOfLastMessage);
                }
            } else {
                radioManager_logln(F("[ACK] | Can not extract message id from received message"));
            }
        }
    }
}

String radioManager_readReceivedData() {
    String str = "";
    while (LoRa.available()) {
        str += (char) LoRa.read();
    }

    str.remove(str.indexOf("`"));


    radioManager_logln();
    radioManager_logln(F("[RFM96] Received packet!"));

    // print data of the packet
    radioManager_log(F("[RFM96] Data:\t\t"));
    radioManager_logln(str);

//        // print senderIdOfLastMessage
//        radioManager_log(F("[RFM96] SendeId:\t\t"));
//        radioManager_logln(radio.getSenderId());

    // print RSSI (Received Signal Strength Indicator)
    radioManager_log(F("[RFM96] RSSI:\t\t"));
    radioManager_log(LoRa.packetRssi());
    radioManager_logln(F(" dBm"));

    // print SNR (Signal-to-Noise Ratio)
    radioManager_log(F("[RFM96] SNR:\t\t"));
    radioManager_log(LoRa.packetSnr());
    radioManager_logln(F(" dB"));

    // print frequency error
    radioManager_log(F("[RFM96] Frequency error:\t"));
    radioManager_log(LoRa.packetFrequencyError());
    radioManager_logln(F(" Hz"));

    return str;
}

void radioManager_extractMessageIdAndSenderIdAndDestinationIdFromReceivedData(
        String &str) { //TODO od razu powinno usuwać z tego stringa te dane
    //TODO czy jak sie tutaj usunię to wszędzie czy tutaj to będzie jednak kopia
    String splittedStr[4];
    int length = splitString(str, splittedStr, '@');
    if (length == 4) {
        destinationIdOfLastMessage = (uint8_t) splittedStr[0].toInt();
        senderIdOfLastMessage = (uint8_t) splittedStr[1].toInt();
        receivedMessageIdOfLastMessage = (uint8_t) splittedStr[2].toInt();
    }
}

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
    radioManager_log(F("isAckPayload, str = "));
    return str.charAt(0) == '!';
}

bool isAckPayloadAndValidMessageId(String str) {
    radioManager_log(F("isAckPayloadAndValidMessageId, str = "));
    radioManager_logln(str);
    if (str.charAt(0) == '!') {
        str.remove(0, 1);
        radioManager_log(F("isAckPayloadAndValidMessageId, messageId = "));
        radioManager_logln(messageId);
        radioManager_log(F("isAckPayloadAndValidMessageId, ((uint8_t) str.toInt()) = "));
        radioManager_logln(((uint8_t) str.toInt()));

        if (messageId == ((uint8_t) str.toInt())) {
            radioManager_logln(F("isAckPayloadAndValidMessageId, return true"));
            return true;
        }
    }
    radioManager_logln(F("isAckPayloadAndValidMessageId, return false"));
    return false;
}

//void dataReceived(const String &str) {
//    radioManager_log(F("Received data: \""));
//    radioManager_log(str);
//    radioManager_logln(F("\""));
//}

void waitForAckTimeoutLoop() {
    if (waitingForAck && !ackReceived) {
        if (millis() - waitForAckStartTime >= ackTimeout) {
            waitingForAck = false;
            radioManager_logln(F("ACK NOT RECEIVED - TIMEOUT"));
            if (ackNotReceivedCallback) {
                ackNotReceivedCallback(ackCallback_paylod);
            }
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
    radioManager_sendLoop();
}

void send(String &str, uint8_t address) {
    radioManager_log(F("Sending: ["));
    radioManager_log(str);
    radioManager_log(F("] to "));
    radioManager_logln(address);
    unsigned long startTime = micros();
    str = String(address) + "@" + String(NODE_ID) + "@" + String(messageId) + "@" + str + "`";
    radioManager_log(F("Transmitting str: ["));
    radioManager_log(str);
    radioManager_logln(F("]"));
    radioManager_sendingTime = micros();
    LoRa_sendMessage(str);
    radioManager_log(F("radio.startTransmit() time: "));
    radioManager_log(String(micros() - startTime));
    radioManager_logln(F(" us"));
}

void LoRa_sendMessage(String message) {
    LoRa_txMode();                        // set tx mode
    LoRa.beginPacket();                   // start packet
    LoRa.print(message);                  // add payload
    LoRa.endPacket(true);                 // finish packet and send it
}

void LoRa_txMode() {
    LoRa.idle();                          // set standby mode
//    LoRa.disableInvertIQ();               // node
}

void radioManager_onReceiveDone(int packetSize) {
    if (packetSize > 0) {
        radioManager_receivedFlag = true;
        receivedPacketSize = packetSize;
        radioManager_logln(F("Received interrupt"));
    } else {
        radioManager_logln(F("ERROR: Received 0 lenght packet!!!"));
    }
}

void radioManager_onTxDone() {
    radioManager_LoRa_rxMode();
    radioManager_transmissionFinished = true;
    radioManager_logln(F("Send interrupt"));
    radioManager_log(F("SENDING TIME = "));
    radioManager_log(String(micros() - radioManager_sendingTime));
    radioManager_logln(F(" us"));
    radioManager_logln(F("TxDone"));

    if (dataSentCallback) {
        dataSentCallback();
    }
}

void radioManager_LoRa_rxMode() {
//    LoRa.enableInvertIQ();                // node
    LoRa.receive();                       // set receive mode
}


void radioManager_logln(const __FlashStringHelper *ifsh) {
    if (logActive) Serial.println(ifsh);
}

void radioManager_log(const __FlashStringHelper *ifsh) {
    if (logActive) Serial.print(ifsh);
}

void radioManager_logln(const String &s) {
    if (logActive) Serial.println(s);
}

void radioManager_log(const String &s) {
    if (logActive) Serial.print(s);
}

void radioManager_logln(unsigned char b, int base) {
    if (logActive) Serial.println(b, base);
}

void radioManager_log(unsigned char b, int base) {
    if (logActive) Serial.print(b, base);
}

void radioManager_logln() {
    if (logActive) Serial.println();
}

void radioManager_logln(int n, int base) {
    if (logActive) Serial.println(n, base);
}

void radioManager_log(int n, int base) {
    if (logActive) Serial.print(n, base);
}

void radioManager_logln(double n, int digits) {
    if (logActive) Serial.println(n, digits);
}

void radioManager_log(double n, int digits) {
    if (logActive) Serial.print(n, digits);
}

void radioManager_logln(long n, int base) {
    if (logActive) Serial.println(n, base);
}

void radioManager_log(long n, int base) {
    if (logActive) Serial.print(n, base);
}