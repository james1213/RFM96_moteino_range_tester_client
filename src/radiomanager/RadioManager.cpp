//
// Created by LukaszLibront on 19.12.2023.
//

#include "RadioManager.h"


void RadioManager::setupRadio(long frequency, int ss, int reset, int dio0, uint8_t _nodeId, void(*receiveDoneCallback)(int), void(*txDoneCallback)()) {
    nodeId = _nodeId;
//    LoRa.setPins(10, 7, 2);
    LoRa.setPins(ss, reset, dio0);
    if (!LoRa.begin(frequency)) {
        DEBUGlogln(F("LoRa init failed. Check your connections."));
        while (true);                       // if failed, do nothing
    }
    // enableCrc() musi byc PO begin() - begin() resetuje modul i czysci ten bit
    LoRa.enableCrc();
    LoRa.onReceive(receiveDoneCallback);
    LoRa.onTxDone(txDoneCallback);

    DEBUGlogln(F("LoRa init succeeded."));

    LoRa_rxMode();
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

void RadioManager::setHaveData(bool value) {
    _haveData = value;
}

bool RadioManager::isTransmissionFinished() {
    return transmissionFinished;
}

bool RadioManager::isNeedToSendAckToSender(){
    return needToSendAckToSender;
}

void RadioManager::loop() {
    sendLoop();
    receiveLoop();
    waitForAckTimeoutLoop();
}

void RadioManager::sendLoop() {
    if (transmissionFinished) {
        if (!transmissionClenedUp) {
            transmissionClenedUp = true;
            DEBUGlogln(F("transmission finished!"));
            if (LOG_ACTIVE) {
                DEBUGlog(F("SENDING TIME = "));
                DEBUGlog(String(txDoneTime - sendingTime));
                DEBUGlogln(F(" us"));
            }
            if (dataSentCallback) {
                dataSentCallback();
            }
        } else if (!sendBuffer.equals("") || !ackSendBuffer.equals("")) {
            DEBUGlogln(F("[RFM96] Sending another packet ... "));
            transmissionFinished = false;
            transmissionClenedUp = false;
            if (!ackSendBuffer.equals("")) {
                DEBUGlogln(F("[RFM96] Sending ACK packet ... "));
                startSending(ackSendBuffer, ackSendBufferDest, false);
                ackSendBuffer = "";
            } else if (!sendBuffer.equals("")) {
                DEBUGlogln(F("[RFM96] Sending normal packet ... "));
                startSending(sendBuffer, sendBufferDest, sendBufferAckReq);
                sendBuffer = "";
            }
        }
    }
}

void RadioManager::receiveLoop() {
    if (zeroLengthPacketReceived) {
        zeroLengthPacketReceived = false;
        DEBUGlogln(F("ERROR: Received 0 lenght packet!!!"));
    }
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
            if (LOG_ACTIVE) {
                DEBUGlog(F("RECEIVED ACK TIME = "));
                DEBUGlog(String(micros() - sendingTime));
                DEBUGlogln(F(" us"));
            }
            ackReceived = true;
            waitingForAck = false;
            pendingAckMessageId = 0;
            if (ackReceivedCallback) {
                ackReceivedCallback();
            }
        } else {
            if (isAckPayload(str)) {
                // spozniony/niedopasowany ACK - nie publikuj go jako danych
                DEBUGlogln(F("RadioManager | stale/unmatched ACK - ignoring"));
                return;
            }
            DEBUGlogln(F("RadioManager | _haveData = true"));
            _haveData = true;
            lastReceivedData = str;


            if (receivedMessageIdOfLastMessage != 0) {
                DEBUGlogln(F("RadioManager | checking if it is need to send ACK"));
                DEBUGlog(F("RadioManager | !isAckPayload(str) = "));
                DEBUGlogln(!isAckPayload(str));
                DEBUGlog(F("RadioManager | needToSendAckToSender = "));
                DEBUGlogln(needToSendAckToSender);
                if (!isAckPayload(str) && needToSendAckToSender) {
                    DEBUGlogln(F("RadioManager | inside: !isAckPayload(str) && needToSendAckToSender && sendAckAutomaticly"));
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
            } else if (isDataPayload(str)){
                DEBUGlogln(F("Received DATA message"));
                str = str.substring(5);
                if (dataReceivedCallback) {
                    dataReceivedCallback(str, senderIdOfLastMessage);
                }
            } else {
                DEBUGlog(F("Wrong message format, message: "));
                DEBUGlogln(str);
            }
        }
    }
}

void RadioManager::sendAck() {
    DEBUGlog(F("Sending ACK to address: "));
    DEBUGlogln(senderIdOfLastMessage);
    String ackString = "!";
    ackString.concat(receivedMessageIdOfLastMessage);
    sendDirectly(ackString, senderIdOfLastMessage, false, nullptr, nullptr, true);
}

String RadioManager::readReceivedData() {
    String str = "";
    str.reserve(receivedPacketSize + 1); // jedna alokacja zamiast realokacji przy kazdym znaku
//    for(int & receivedByte : receivedBytes) {
//        receivedByte = 0;
//    }
//    int index = 0;
    while (LoRa.available()) {
//        receivedBytes[index] = LoRa.read();
        str += (char) LoRa.read();
//        str += (char) receivedBytes[index];
//        index++;
    }
//    Serial.print(F("readReceivedData, str.length()")); Serial.println(str.length());

//    int receivedBytes[256];
//    int index = 0;
//    while (LoRa.available()) {
//        receivedBytes[index++] = LoRa.read();
//    }
//    Serial.print(F("readReceivedData, str.length()")); Serial.println(str.length());


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
}

int RadioManager::splitString(String &text, String *texts, char ch, int maxArrayLength) { // Split the string into substrings
    int arrayLength = maxArrayLength;
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

bool RadioManager::isAckPayload(const String &str) {
//    DEBUGlog(F("isAckPayload, str = "));
//    DEBUGlogln(str);
    return str.charAt(0) == '!';
}

bool RadioManager::isAckPayloadAndValidMessageId(String str) {
    DEBUGlog(F("isAckPayloadAndValidMessageId, str = "));
    DEBUGlogln(str);
    if (str.charAt(0) == '!') {
        str.remove(0, 1);
        DEBUGlog(F("isAckPayloadAndValidMessageId, pendingAckMessageId = "));
        DEBUGlogln(pendingAckMessageId);
        DEBUGlog(F("isAckPayloadAndValidMessageId, ((uint8_t) str.toInt()) = "));
        DEBUGlogln(((uint8_t) str.toInt()));

        // porownujemy z id ramki DAT czekajacej na ACK, a nie z zywym licznikiem
        // messageId - ten podbija tez kazda wyslana ramka ACK
        if (pendingAckMessageId != 0 && pendingAckMessageId == ((uint8_t) str.toInt())) {
            DEBUGlogln(F("isAckPayloadAndValidMessageId, return true"));
            return true;
        }
    }
    DEBUGlogln(F("isAckPayloadAndValidMessageId, return false"));
    return false;
}

void RadioManager::waitForAckTimeoutLoop() {
    if (waitingForAck && !ackReceived) {
        if (millis() - waitForAckStartTime >= ackTimeout) {
            waitingForAck = false;
            pendingAckMessageId = 0;
            DEBUGlogln(F("ACK NOT RECEIVED - TIMEOUT"));
            if (ackNotReceivedCallback) {
                ackNotReceivedCallback(ackCallback_paylod);
            }
        }
    }
}

void RadioManager::sendOta(String &str, uint8_t address, void (*_ackReceivedCallback)(), void (*_ackNotReceivedCallback)(String &payload)) {
    String strOta = "<OTA>" + str;
    sendDirectly(strOta, address, _ackReceivedCallback || _ackNotReceivedCallback, _ackReceivedCallback, _ackNotReceivedCallback, false);
}

void RadioManager::send(String &str, uint8_t address, void (*_ackReceivedCallback)(), void (*_ackNotReceivedCallback)(String &payload)) {
    String strData = "<DAT>" + str;
    sendDirectly(strData, address, _ackReceivedCallback || _ackNotReceivedCallback, _ackReceivedCallback, _ackNotReceivedCallback, false);
}

void RadioManager::sendDirectly(String &str, uint8_t address, bool ackRequested, void (*_ackReceivedCallback)(), void (*_ackNotReceivedCallback)(String &payload), bool useAckBuffer) {
    if (ackRequested) {
        if (waitingForAck && ackNotReceivedCallback) {
            // poprzednia wiadomosc wciaz czekala na ACK - powiadom aplikacje, ze przepada
            waitingForAck = false;
            ackNotReceivedCallback(ackCallback_paylod);
        }
        ackReceivedCallback = _ackReceivedCallback;
        ackNotReceivedCallback = _ackNotReceivedCallback;
        ackCallback_paylod = str;
        waitingForAck = true;
        ackReceived = false;
        waitForAckStartTime = millis();
    }
    if (useAckBuffer) {
        ackSendBuffer = str;
        ackSendBufferDest = address;
    } else {
        sendBuffer = str;
        sendBufferDest = address;
        sendBufferAckReq = ackRequested;
    }
    if (messageId == 0) messageId = 1;
    sendLoop();
}

void RadioManager::startSending(String &str, uint8_t address, bool ackRequested) {
    DEBUGlog(F("Sending: ["));
    DEBUGlog(str);
    DEBUGlog(F("] to "));
    DEBUGlogln(address);
    unsigned long startTime = micros();
    messageId++;
    if (messageId == 0) messageId = 1;
    if (ackRequested) pendingAckMessageId = messageId; // to id trafia do ramki i wroci w "!id"
    String frame;
    frame.reserve(str.length() + 16); // naglowek "adr@nadawca@id@ack@" + '`'
    frame += String(address);
    frame += '@';
    frame += String(nodeId);
    frame += '@';
    frame += String(messageId);
    frame += '@';
    frame += (ackRequested ? '1' : '0');
    frame += '@';
    frame += str;
    frame += '`';
    DEBUGlog(F("Transmitting str: ["));
    DEBUGlog(frame);
    DEBUGlogln(F("]"));
    sendingTime = micros();
    LoRa_sendMessage(frame);
    if (LOG_ACTIVE) {
        DEBUGlog(F("radio.startTransmit() time: "));
        DEBUGlog(String(micros() - startTime));
        DEBUGlogln(F(" us"));
    }
}

void RadioManager::LoRa_sendMessage(const String &message) {
    LoRa_txMode();                        // set tx mode
    LoRa.beginPacket();                   // start packet
    LoRa.print(message);                  // add payload
    LoRa.endPacket(true);                 // finish packet and startSending it
}

void RadioManager::LoRa_txMode() {
    LoRa.idle();                          // set standby mode
//    LoRa.disableInvertIQ();               // node
}

// Kontekst przerwania (ISR)! Zadnych Serial.print ani String (malloc nie jest
// reentrantny - alokacja w ISR w trakcie alokacji w petli glownej psuje sterte).
void RadioManager::receiveDone(int packetSize) {
    if (packetSize > 0) {
        receivedFlag = true;
        receivedPacketSize = packetSize;
    } else {
        zeroLengthPacketReceived = true;
    }
}

// Kontekst przerwania (ISR) - jak wyzej. Logi i dataSentCallback z sendLoop().
void RadioManager::txDone() {
    LoRa_rxMode();
    txDoneTime = micros();
    transmissionFinished = true;
}

void RadioManager::LoRa_rxMode() {
//    LoRa.enableInvertIQ();                // node
    LoRa.receive();                       // set receive mode
}


void RadioManager::DEBUGlogln(const __FlashStringHelper *ifsh) {
    if (LOG_ACTIVE) Serial.println(ifsh);
}

void RadioManager::DEBUGlog(const __FlashStringHelper *ifsh) {
    if (LOG_ACTIVE) Serial.print(ifsh);
}

void RadioManager::DEBUGlogln(const String &s) {
    if (LOG_ACTIVE) Serial.println(s);
}

void RadioManager::DEBUGlog(const String &s) {
    if (LOG_ACTIVE) Serial.print(s);
}

void RadioManager::DEBUGlogln(unsigned char b, int base) {
    if (LOG_ACTIVE) Serial.println(b, base);
}

void RadioManager::DEBUGlog(unsigned char b, int base) {
    if (LOG_ACTIVE) Serial.print(b, base);
}

void RadioManager::DEBUGlogln() {
    if (LOG_ACTIVE) Serial.println();
}

void RadioManager::DEBUGlogln(int n, int base) {
    if (LOG_ACTIVE) Serial.println(n, base);
}

void RadioManager::DEBUGlog(int n, int base) {
    if (LOG_ACTIVE) Serial.print(n, base);
}

void RadioManager::DEBUGlogln(double n, int digits) {
    if (LOG_ACTIVE) Serial.println(n, digits);
}

void RadioManager::DEBUGlog(double n, int digits) {
    if (LOG_ACTIVE) Serial.print(n, digits);
}

void RadioManager::DEBUGlogln(long n, int base) {
    if (LOG_ACTIVE) Serial.println(n, base);
}

void RadioManager::DEBUGlog(long n, int base) {
    if (LOG_ACTIVE) Serial.print(n, base);
}

void RadioManager::dumpRegisters() {
    LoRa.dumpRegisters(Serial);
}

void RadioManager::setTxPower(int dbm) {
    LoRa.setTxPower(dbm);
}

void RadioManager::onOtaDataReceived(void (*callback)(String &, uint8_t)) {
    otaDataReceivedCallback = callback;
}

bool RadioManager::isOtaPayload(String &str) {
    DEBUGlog(F("isOtaPayload, str = "));
    DEBUGlogln(str);

    const char* data = str.c_str();
    bool returnValue = data[0] == '<' && data[1] == 'O' && data[2] == 'T' && data[3] == 'A' && data[4] == '>';
//    bool returnValue = str.startsWith("<OTA>");

    DEBUGlog(F("isOtaPayload, returnValue = "));
    DEBUGlogln(returnValue);
    return returnValue;
}

bool RadioManager::isDataPayload(String &str) {
    DEBUGlog(F("isDataPayload, str = "));
    DEBUGlogln(str);
    const char* data = str.c_str();
    bool returnValue = data[0] == '<' && data[1] == 'D' && data[2] == 'A' && data[3] == 'T' && data[4] == '>';
//    bool returnValue = str.startsWith("<DAT>");

    DEBUGlog(F("isDataPayload, returnValue = "));
    DEBUGlogln(returnValue);
    return returnValue;
}

bool RadioManager::isDataSent() {
    return transmissionFinished;
}
