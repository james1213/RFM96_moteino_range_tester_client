//
// Created by LukaszLibront on 19.12.2023.
//

#include "RadioManager.h"

//uint8_t nodeId = 0;
//bool logActive = true;
//volatile bool transmissionFinished = true;
//volatile bool receivedFlag = false;
//unsigned long sendingTime = 0;
//volatile bool transmissionClenedUp = true;
//volatile bool ackReceived = false;
//bool waitingForAck = false;
//unsigned long waitForAckStartTime = 0;
//unsigned long ackTimeout = 1000; //ms
//String ackCallback_paylod = "";
//
//void (*ackNotReceivedCallback)(String &payload);
//void (*ackReceivedCallback)();
//void (*dataReceivedCallback)(String &receivedText, uint8_t senderId);
//void (*dataSentCallback)();
//
//String sendBuffer = "";
//uint8_t messageId = 0;
//uint8_t destinationAddress = 0;
//uint8_t destinationIdOfLastMessage = 0;
//uint8_t senderIdOfLastMessage = 0;
//uint8_t receivedMessageIdOfLastMessage = 0;
//String lastReceivedData;
//int receivedPacketSize = 0;
//
//bool _ackRequested = false;
//bool needToSendAckToSender = false;
//bool sendAckAutomaticly = true;
//volatile bool _haveData;


void RadioManager::setupRadio(uint8_t _nodeId, void(*onReceiveDoneCallback)(int), void(*onTxDoneCallback)()) {
    nodeId = _nodeId;
    LoRa.enableCrc();
    LoRa.setPins(10, 7, 2);
    if (!LoRa.begin(433E6)) {
        DEBUGlogln("LoRa init failed. Check your connections.");
        while (true);                       // if failed, do nothing
    }

    DEBUGlogln("LoRa init succeeded.");
    DEBUGlogln();
    DEBUGlogln("LoRa Simple Node");
    DEBUGlogln("Only receive messages from gateways");
    DEBUGlogln("Tx: invertIQ disable");
    DEBUGlogln("Rx: invertIQ enable");
    DEBUGlogln();

    LoRa.onReceive(onReceiveDoneCallback);
    LoRa.onTxDone(onTxDoneCallback);
    LoRa_rxMode();
}

void cStyleWrapper_onReceiveDone(int packetSize) {
    //TODO
}

void RadioManager::onDataReceived(void(*callback)(String &receivedText, uint8_t senderId)) {
    dataReceivedCallback = callback;
}

void RadioManager::onDataSent(void(*callback)()) {
    dataSentCallback = callback;
}

int RadioManager::getReceivedPacketSize() {
    return receivedPacketSize;
}

void RadioManager::setSendAckAutomaticly(bool value) {
    sendAckAutomaticly = value;
}

bool RadioManager::isAckReceived() {
    return ackReceived;
}

uint8_t RadioManager::getSenderIdOfLastMessage() {
    return senderIdOfLastMessage;
}

String RadioManager::getLastReceivedData() {
    return lastReceivedData;
}

bool RadioManager::isHaveDate() {
    return _haveData;
}

bool RadioManager::setHaveData(bool value) {
    _haveData = value;
}

bool RadioManager::isTransmissionFinished() {
    return transmissionFinished;
}

bool RadioManager::isNeedToSendAckToSender(){
    return needToSendAckToSender;
}

void RadioManager::radioLoop() {
    sendLoop();
    receiveLoop();
    waitForAckTimeoutLoop();
}

void RadioManager::sendLoop() {
    if (transmissionFinished) {
        if (!transmissionClenedUp) {
            transmissionClenedUp = true;
            DEBUGlogln(F("transmission finished!"));
        } else if (!sendBuffer.equals("") || !ackSendBuffer.equals("")) {
            DEBUGlog(F("[RFM96] Sending another packet ... "));
            transmissionFinished = false;
            transmissionClenedUp = false;
            if (!ackSendBuffer.equals("")) {
                DEBUGlog(F("[RFM96] Sending ACK packet ... "));
                startSending(ackSendBuffer, destinationAddress);
                ackSendBuffer = "";
            } else if (!sendBuffer.equals("")) {
                DEBUGlog(F("[RFM96] Sending normal packet ... "));
                startSending(sendBuffer, destinationAddress);
                sendBuffer = "";
            }
        }
    }
}

void RadioManager::receiveLoop() {
    if (receivedFlag) {
        receivedFlag = false;
        String str = readReceivedData();

        DEBUGlog(F("[ACK] | before extractMessageIdAndSenderIdAndDestinationIdFromReceivedData = "));
        DEBUGlogln(str);
        extractMessageIdAndSenderIdAndDestinationIdFromReceivedData(str);
        DEBUGlog(F("[ACK] | extracted senderIdOfLastMessage = "));
        DEBUGlogln(senderIdOfLastMessage);
        DEBUGlog(F("[ACK] | extracted receivedMessageIdOfLastMessage = "));
        DEBUGlogln(receivedMessageIdOfLastMessage);
        DEBUGlog(F("[ACK] | extracted needToSendAckToSender = "));
        DEBUGlogln(needToSendAckToSender);
        DEBUGlog(F("[ACK] | after extractMessageIdAndSenderIdAndDestinationIdFromReceivedData = "));
        DEBUGlogln(str);

        if (destinationIdOfLastMessage == nodeId) {
            DEBUGlogln(F("This is a destination address"));
        } else {
            DEBUGlogln(F("This is not a destination address, ignoring message"));
            return;
        }

        if (isAckPayloadAndValidMessageId(str) && waitingForAck) {
            DEBUGlogln(F("Received ACK"));
            DEBUGlog(F("RECEIVED ACK TIME = "));
            DEBUGlog(String(micros() - sendingTime));
            DEBUGlogln(F(" us"));
            ackReceived = true;
            waitingForAck = false;
            if (ackReceivedCallback) {
                ackReceivedCallback();
            }
        } else {
//            Serial.println(F("RadioManager | _haveData = true"));
            _haveData = true;
            lastReceivedData = str;


            if (receivedMessageIdOfLastMessage != 0) {
                DEBUGlogln(F("RadioManager | checking if it is need to send ACK"));
                DEBUGlog(F("RadioManager | !isAckPayload(str) = "));
                DEBUGlogln(!isAckPayload(str));
                DEBUGlog(F("RadioManager | needToSendAckToSender = "));
                DEBUGlogln(needToSendAckToSender);
                DEBUGlog(F("RadioManager | sendAckAutomaticly = "));
                DEBUGlogln(sendAckAutomaticly);
                if (!isAckPayload(str) && needToSendAckToSender && sendAckAutomaticly) {
//                    Serial.println(F("RadioManager | inside: !isAckPayload(str) && needToSendAckToSender && sendAckAutomaticly"));
                    sendAck();
                }
            } else {
                DEBUGlogln(F("[ACK] | Can not extract message id from received message"));
            }


            if (isOtaPayload(str)) {
                DEBUGlogln(F("Received OTA message"));
                str = str.substring(5);
                if (otaDataReceivedCallback) {
                    otaDataReceivedCallback(str, senderIdOfLastMessage);
                }
            } else {
                if (dataReceivedCallback) {
                    dataReceivedCallback(str, senderIdOfLastMessage);
                }
            }





        }
    }
}

void RadioManager::sendAck() {
    DEBUGlog(F("Sending ACK to address: "));
//    Serial.print(F("Sending ACK to address: "));
    DEBUGlogln(senderIdOfLastMessage);
//    Serial.println(senderIdOfLastMessage);
    String ackString = "!";
    ackString.concat(receivedMessageIdOfLastMessage);
    send(ackString, senderIdOfLastMessage, false, true, nullptr, nullptr, true);
}

String RadioManager::readReceivedData() {
    String str = "";
    while (LoRa.available()) {
        str += (char) LoRa.read();
    }

//    DEBUGlog(F("BEFORE remove: "));
//    DEBUGlogln(str);
//    str.remove(str.indexOf("`")); //TODO usunięcie tylko ostatniego takiego znaku

    str.remove(str.lastIndexOf("`")); //TODO usunięcie tylko ostatniego takiego znaku
//    str.remove(str.length() - 1);
//    DEBUGlog(F("AFTER remove: "));
//    DEBUGlogln(str);

    DEBUGlogln();
    DEBUGlogln(F("[RFM96] Received packet!"));

    // print data of the packet
    DEBUGlog(F("[RFM96] Data:\t\t"));
    DEBUGlogln(str);

//        // print senderIdOfLastMessage
//        DEBUGlog(F("[RFM96] SendeId:\t\t"));
//        DEBUGlogln(radio.getSenderId());

    // print RSSI (Received Signal Strength Indicator)
    DEBUGlog(F("[RFM96] RSSI:\t\t"));
    DEBUGlog(LoRa.packetRssi());
    DEBUGlogln(F(" dBm"));

    // print SNR (Signal-to-Noise Ratio)
    DEBUGlog(F("[RFM96] SNR:\t\t"));
    DEBUGlog(LoRa.packetSnr());
    DEBUGlogln(F(" dB"));

    // print frequency error
    DEBUGlog(F("[RFM96] Frequency error:\t"));
    DEBUGlog(LoRa.packetFrequencyError());
    DEBUGlogln(F(" Hz"));

    return str;
}

//TODO przerobić tę metodę
void RadioManager::extractMessageIdAndSenderIdAndDestinationIdFromReceivedData(
        String &str) { //TODO od razu powinno usuwać z tego stringa te dane
    //TODO czy jak sie tutaj usunię to wszędzie czy tutaj to będzie jednak kopia
    String splittedStr[4];
    int length = splitString(str, splittedStr, '@', 4);
    if (length == 4) {
        destinationIdOfLastMessage = (uint8_t) splittedStr[0].toInt();
        senderIdOfLastMessage = (uint8_t) splittedStr[1].toInt();
        receivedMessageIdOfLastMessage = (uint8_t) splittedStr[2].toInt();
        needToSendAckToSender = (bool) splittedStr[3].toInt();
    }
//    for (int i = 0; i < length; i++) {
//        DEBUGlog(F("splittedStr["));
//        DEBUGlog(i);
//        DEBUGlog(F("]="));
//        DEBUGlogln(splittedStr[i]);
//    }
}

int RadioManager::splitString(String &text, String *texts, char ch, int maxArrayLength) { // Split the string into substrings
//    unsigned int arrayLength = texts->length();
    unsigned int arrayLength = maxArrayLength;
    int arrayIndex = 0;
    int stringCount = 0;
    while (text.length() > 0 && arrayIndex < arrayLength) {
        arrayIndex++;
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

bool RadioManager::isAckPayload(String str) {
    DEBUGlog(F("isAckPayload, str = "));
    DEBUGlogln(str);
    return str.charAt(0) == '!';
}

bool RadioManager::isAckPayloadAndValidMessageId(String str) {
    DEBUGlog(F("isAckPayloadAndValidMessageId, str = "));
    DEBUGlogln(str);
    if (str.charAt(0) == '!') {
        str.remove(0, 1);
        DEBUGlog(F("isAckPayloadAndValidMessageId, messageId = "));
        DEBUGlogln(messageId);
        DEBUGlog(F("isAckPayloadAndValidMessageId, ((uint8_t) str.toInt()) = "));
        DEBUGlogln(((uint8_t) str.toInt()));

        if (messageId == ((uint8_t) str.toInt())) {
            DEBUGlogln(F("isAckPayloadAndValidMessageId, return true"));
            return true;
        }
    }
    DEBUGlogln(F("isAckPayloadAndValidMessageId, return false"));
    return false;
}

//void RadioManager::dataReceived(const String &str) {
//    DEBUGlog(F("Received data: \""));
//    DEBUGlog(str);
//    DEBUGlogln(F("\""));
//}

void RadioManager::waitForAckTimeoutLoop() {
    if (waitingForAck && !ackReceived) {
        if (millis() - waitForAckStartTime >= ackTimeout) {
            waitingForAck = false;
            DEBUGlogln(F("ACK NOT RECEIVED - TIMEOUT"));
            if (ackNotReceivedCallback) {
                ackNotReceivedCallback(ackCallback_paylod);
            }
        }
    }
}

//void RadioManager::bufferedSendAndWaitForAck(String &str, uint8_t address, void (*_ackReceivedCallback)(), void (*_ackNotReceivedCallback)(String &payload)) {
//    ackReceivedCallback = _ackReceivedCallback;
//    ackNotReceivedCallback = _ackNotReceivedCallback;
////    ackCallback_paylod = str;
////    waitingForAck = true;
////    ackReceived = false;
////    waitForAckStartTime = millis();
//    send(str, address, true);
//}

void RadioManager::send(String &str, uint8_t address, bool ackRequested, bool _sendAckAutomaticly, void (*_ackReceivedCallback)(), void (*_ackNotReceivedCallback)(String &payload), bool useAckBuffer) {
    sendAckAutomaticly = _sendAckAutomaticly;
    if (ackRequested) {
        ackReceivedCallback = _ackReceivedCallback;
        ackNotReceivedCallback = _ackNotReceivedCallback;
        ackCallback_paylod = str;
        waitingForAck = true;
        ackReceived = false;
        waitForAckStartTime = millis();
    }
    if (useAckBuffer) {
        ackSendBuffer = str;
//        ackMessageId++;
    } else {
        sendBuffer = str;
//        messageId++;
    }
    if (messageId == 0) messageId = 1;
    destinationAddress = address;
    _ackRequested = ackRequested;
    sendLoop();
}

void RadioManager::startSending(String &str, uint8_t address) {
    DEBUGlog(F("Sending: ["));
    DEBUGlog(str);
    DEBUGlog(F("] to "));
    DEBUGlogln(address);
    unsigned long startTime = micros();
    messageId++;
    if (messageId == 0) messageId = 1;
    str = String(address) + "@" + String(nodeId) + "@" + String(messageId) + "@" + (_ackRequested ? "1" : "0") + "@" + str + "`";
    DEBUGlog(F("Transmitting str: ["));
    DEBUGlog(str);
    DEBUGlogln(F("]"));
    sendingTime = micros();
    LoRa_sendMessage(str);
    DEBUGlog(F("radio.startTransmit() time: "));
    DEBUGlog(String(micros() - startTime));
    DEBUGlogln(F(" us"));
}

void RadioManager::LoRa_sendMessage(String message) {
    LoRa_txMode();                        // set tx mode
    LoRa.beginPacket();                   // start packet
    LoRa.print(message);                  // add payload
    LoRa.endPacket(true);                 // finish packet and startSending it
}

void RadioManager::LoRa_txMode() {
    LoRa.idle();                          // set standby mode
//    LoRa.disableInvertIQ();               // node
}

void RadioManager::onReceiveDone(int packetSize) {
    if (packetSize > 0) {
        receivedFlag = true;
        receivedPacketSize = packetSize;
        DEBUGlogln(F("Received interrupt"));
    } else {
        DEBUGlogln(F("ERROR: Received 0 lenght packet!!!"));
    }
}

void RadioManager::onTxDone() {
    LoRa_rxMode();
    transmissionFinished = true;
    DEBUGlogln(F("Send interrupt"));
    DEBUGlog(F("SENDING TIME = "));
    DEBUGlog(String(micros() - sendingTime));
    DEBUGlogln(F(" us"));
    DEBUGlogln(F("TxDone"));

    if (dataSentCallback) {
        dataSentCallback();
    }
}

void RadioManager::LoRa_rxMode() {
//    LoRa.enableInvertIQ();                // node
    LoRa.receive();                       // set receive mode
}


void RadioManager::DEBUGlogln(const __FlashStringHelper *ifsh) {
    if (logActive) Serial.println(ifsh);
}

void RadioManager::DEBUGlog(const __FlashStringHelper *ifsh) {
    if (logActive) Serial.print(ifsh);
}

void RadioManager::DEBUGlogln(const String &s) {
    if (logActive) Serial.println(s);
}

void RadioManager::DEBUGlog(const String &s) {
    if (logActive) Serial.print(s);
}

void RadioManager::DEBUGlogln(unsigned char b, int base) {
    if (logActive) Serial.println(b, base);
}

void RadioManager::DEBUGlog(unsigned char b, int base) {
    if (logActive) Serial.print(b, base);
}

void RadioManager::DEBUGlogln() {
    if (logActive) Serial.println();
}

void RadioManager::DEBUGlogln(int n, int base) {
    if (logActive) Serial.println(n, base);
}

void RadioManager::DEBUGlog(int n, int base) {
    if (logActive) Serial.print(n, base);
}

void RadioManager::DEBUGlogln(double n, int digits) {
    if (logActive) Serial.println(n, digits);
}

void RadioManager::DEBUGlog(double n, int digits) {
    if (logActive) Serial.print(n, digits);
}

void RadioManager::DEBUGlogln(long n, int base) {
    if (logActive) Serial.println(n, base);
}

void RadioManager::DEBUGlog(long n, int base) {
    if (logActive) Serial.print(n, base);
}

void RadioManager::dumpRegisters() {
    LoRa.dumpRegisters(Serial);
}

void RadioManager::onOtaDataReceived(void (*callback)(String &, uint8_t)) {
    otaDataReceivedCallback = callback;
}

bool RadioManager::isOtaPayload(String str) {
    DEBUGlog(F("isOtaPayload, str = "));
    DEBUGlogln(str);
    const char* data = str.c_str();


//    DEBUGlog(F("isOtaPayload, data[0] = "));
//    DEBUGlogln(data[0]);
//    DEBUGlog(F("isOtaPayload, data[1] = "));
//    DEBUGlogln(data[1]);
//    DEBUGlog(F("isOtaPayload, data[2] = "));
//    DEBUGlogln(data[2]);


    bool returnValue = data[0] == '<' && data[1] == 'O' && data[2] == 'T' && data[3] == 'A' && data[4] == '>';

    DEBUGlog(F("isOtaPayload, returnValue = "));
    DEBUGlogln(returnValue);
    return returnValue;
}
