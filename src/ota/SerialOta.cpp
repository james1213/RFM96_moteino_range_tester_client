//
// Created by LukaszLibront on 29.12.2023.
//

#include "SerialOta.h"



//char input[64]; //serial input buffer

void SerialOta::loop() {
    if (Serial.available()) {
        byte inputLen = readSerialLine(_input, 10, 64, 100);
        if (inputLen > 0) {
//            Serial.print("Received data from serial: ");
//            Serial.println(_input);
            boolean configChanged = false;
            char *colon = strchr(_input, ':');

            if (strstr(_input, "EEPROMRESET") == _input) {
                resetEEPROM();
            } else if (strstr(_input, "SETTINGS?") == _input) {
                printSettings();
            } else if (strstr(_input, "NETWORKID:") == _input && strlen(colon + 1) > 0) {
                uint8_t newNetId = atoi(++colon); //extract ID from message
                if (newNetId <= 255) {
                    CONFIG.NETWORKID = newNetId;
                    configChanged = true;
                } else {
                    Serial << F("Invalid networkId:") << newNetId << endl;
                }
            } else if (strstr(_input, "NODEID:") == _input && strlen(colon + 1) > 0) {
                uint16_t newId = atoi(++colon); //extract ID from message
                if (newId <= 1023) {
                    CONFIG.NODEID = newId;
                    configChanged = true;
                } else {
                    Serial << F("Invalid nodeId:") << newId << endl;
                }
            } else if (strstr(_input, "FREQUENCY:") == _input && strlen(colon + 1) > 0) {
                uint32_t newFreq = atol(++colon); //extract ID from message
                if (VALID_FREQUENCY(newFreq)) {
                    CONFIG.FREQUENCY = newFreq;
                    configChanged = true;
                } else {
                    Serial << F("Invalid frequency:") << newFreq << endl;
                }
            } else if (strstr(_input, "ENCRYPTKEY:") == _input) {
                if (strlen(colon + 1) == 16) {
                    strcpy(CONFIG.ENCRYPTKEY, colon + 1);
                    configChanged = true;
                } else if (strlen(colon + 1) == 0) {
                    strcpy(CONFIG.ENCRYPTKEY, "");
                    configChanged = true;
                } else
                    Serial << F("Invalid encryptkey length:") << colon + 1 << "(" << strlen(colon + 1)
                           << F("expected:16)") << endl;
            } else if (strstr(_input, "BR300KBPS:") == _input && strlen(colon + 1) > 0) {
                uint8_t newBR = atoi(++colon); //extract ID from message
                if (newBR == 0 || newBR == 1) {
                    CONFIG.BR300KBPS = newBR;
                    configChanged = true;
                } else {
                    Serial << F("Invalid BR300KBPS:") << newBR << endl;
                }
            } else if (inputLen == 4 && strstr(_input, "FLX?") == _input) {
                if (targetID == 0)
                    Serial.println("TO?");
                else
                    checkForSerialHandshake((byte *) _input, inputLen, targetID, TIMEOUT, ACK_TIME, DEBUG_MODE);
            } else if (strstr(_input, "TO:") == _input && strlen(colon + 1) > 0) {
                uint16_t newTarget = atoi(++colon);
                if (newTarget > 0 && newTarget <= 1023) {
                    targetID = newTarget;
                    Serial << F("TO:") << targetID << F(":OK") << endl;
                } else Serial << _input << F(":INV") << endl;
            } else Serial << F("UNKNOWN_CMD: ") << _input << endl; //echo back un

            if (configChanged) {
                EEPROM.writeBlock(0, CONFIG); //save changes to EEPROM
                printSettings();
//                initRadio();
            }
        }
    }


    if (radioOta->getState() == OtaState(WAITING_FOR_HEX_DATA_FROM_SERIAL)) {
        handleHexData();
    } else if (radioOta->getState() == OtaState(WAITING_FOR_START)) {
        Serial.println(F("FLX?NOK"));
    } else if (radioOta->getState() == OtaState(HANDLING_SERIAL_HEX_DATA)) {
        handleSerialHexData();
    } else if (radioOta->getState() == OtaState(TRANSMISSION_SUCESS)) {
        Serial.println(F("FLX?OK")); //signal EOF serial handshake back to host script
        if (debug) Serial.println(F("FLASH IMG TRANSMISSION SUCCESS"));
    } else if (radioOta->getState() == OtaState(TRANSMISSION_FAILED)) {
        if (debug) Serial.println(F("FLASH IMG TRANSMISSION FAIL"));
    }

}


//===================================================================================================================
// readSerialLine() - reads a line feed (\n) terminated line from the serial stream
// returns # of bytes read, up to 254
// timeout in ms, will timeout and return after so long
// this is called at the OTA programmer side
//===================================================================================================================
uint8_t SerialOta::readSerialLine(char *input, char endOfLineChar, uint8_t maxLength, uint16_t timeout) {
    uint8_t inputLen = 0;
    Serial.setTimeout(timeout);
    inputLen = Serial.readBytesUntil(endOfLineChar, input, maxLength);
    input[inputLen] = 0;//null-terminate it
    Serial.setTimeout(0);
    return inputLen;
}

SerialOta::SerialOta(RadioManager *manager, RadioOta *radioOta) {
    this->manager = manager;
    this->radioOta = radioOta;
}

boolean SerialOta::resetEEPROMCondition() {
    //conditions for resetting EEPROM:
    return CONFIG.NETWORKID > 255 ||
           CONFIG.NODEID > 1023 ||
           !VALID_FREQUENCY(CONFIG.FREQUENCY);
}

void SerialOta::resetEEPROM() {
    Serial.println("Resetting EEPROM to default values...");
    CONFIG.NETWORKID = NETWORKID_DEFAULT;
    CONFIG.NODEID = NODEID_DEFAULT;
    CONFIG.FREQUENCY = FREQUENCY_DEFAULT;
    CONFIG.BR300KBPS = false;
    strcpy(CONFIG.ENCRYPTKEY, ENCRYPTKEY_DEFAULT);
    EEPROM.writeBlock(0, CONFIG);
}

void SerialOta::printSettings() {
    Serial << endl << F("NETWORKID:") << CONFIG.NETWORKID << endl;
    Serial << F("NODEID:") << CONFIG.NODEID << endl;
    Serial << F("FREQUENCY:") << CONFIG.FREQUENCY << endl;
    Serial << F("BR300KBPS:") << CONFIG.BR300KBPS << endl;
    Serial << F("ENCRYPTKEY:") << CONFIG.ENCRYPTKEY << endl;
}

void SerialOta::Blink(int DELAY_MS) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(LED_BUILTIN, LOW);
}

//===================================================================================================================
// checkForSerialHandshake() - returns TRUE if a HEX file transmission was detected and it was actually transmitted successfully
// this is called at the OTA programmer side
//===================================================================================================================
void
SerialOta::checkForSerialHandshake(uint8_t *input, uint8_t inputLen, uint16_t targetID, uint16_t timeout, uint16_t ackTimeout,
                                   uint8_t debug) {
    if (inputLen == 4 && input[0] == 'F' && input[1] == 'L' && input[2] == 'X' && input[3] == '?') {
        radioOta->setState(OtaState(SENDING_HANDSHAKE));
    }
}

//HandleSerialHandshake
void SerialOta::handleHexData(uint8_t *input, uint8_t inputLen, uint16_t targetID, uint16_t timeout, uint16_t ackTimeout,
                              uint8_t debug) {
    Serial.println(F("\nFLX?OK")); //signal serial handshake back to host script
    radioOta->setState(OtaState(HANDLING_SERIAL_HEX_DATA));
}

void SerialOta::handleSerialHexData() {
#ifdef SHIFTCHANNEL
    if (handleSerialHEXDataWrapper(targetID, timeout, ackTimeout, debug))
#else
        if (handleSerialHEXData(radio, targetID,timeout, ackTimeout, debug))
#endif
    {
        Serial.println(F("FLX?OK")); //signal EOF serial handshake back to host script
        if (debug) Serial.println(F("FLASH IMG TRANSMISSION SUCCESS"));
    }
    if (debug) Serial.println(F("FLASH IMG TRANSMISSION FAIL"));
}


//===================================================================================================================
// handleSerialHEXDataWrapper() - wrapper for handleSerialHEXData(), also shifts the channel if SHIFTCHANNEL is defined
//===================================================================================================================
#ifdef SHIFTCHANNEL

uint8_t SerialOta::handleSerialHEXDataWrapper(uint16_t targetID, uint16_t timeout,
                                              uint16_t ACKTIMEOUT, uint8_t
                                              DEBUG) {
//    Serial.print(F("   #inside handleSerialHEXDataWrapper()"));
//    radioOta.setFrequency(radio.getFrequency() + SHIFTCHANNEL); //shift center freq by SHIFTCHANNEL amount
    uint8_t result = handleSerialHEXData(targetID, timeout, ACKTIMEOUT, DEBUG);
//    radio.setFrequency(radio.getFrequency() - SHIFTCHANNEL); //shift center freq by SHIFTCHANNEL amount
    return result;
}

#endif


//===================================================================================================================
// handleSerialHEXData() - handles the transmission of the HEX image from the serial port to the node being OTA programmed
// this is called at the OTA programmer side
//===================================================================================================================
uint8_t SerialOta::handleSerialHEXData(uint16_t targetID, uint16_t timeout, uint16_t ACKTIMEOUT,
                                       uint8_t debug) {
//    Serial.print(F("   #inside handleSerialHEXData()"));
    long now = millis();
    uint16_t seq = 0, tmp = 0, inputLen;
    uint16_t remoteID = manager->getSenderIdOfLastMessage(); //save the remoteID as soon as possible
    uint8_t sendBuf[57];
    char input[115];
//a FLASH record should not be more than 64 bytes: FLX:9999:10042000FF4FA591B4912FB7F894662321F48C91D6

    while (1) {
        inputLen = readSerialLine(input);
        if (inputLen == 0) goto timeoutcheck;
        tmp = 0;

        if (inputLen >= 6) { //FLX:9:
            if (input[0] == 'F' && input[1] == 'L' && input[2] == 'X') {
                if (input[3] == ':') {
                    uint8_t index = 3;
                    for (uint8_t i = 4; i < 8; i++) //up to 4 characters for seq number
                    {
                        if (input[i] >= 48 && input[i] <= 57)
                            tmp = tmp * 10 + input[i] - 48;
                        else if (input[i] == ':') {
                            if (i == 4)
                                return false;
                            else break;
                        }
                        index++;
                    }
//Serial.print(F("input[index] = "));Serial.print(F("["));Serial.print(index);Serial.print(F("]="));Serial.println(input[index]);
                    if (input[++index] != ':') return false;
                    now = millis(); //got good packet
                    index++;
                    uint8_t hexDataLen = validateHEXData(input + index, inputLen - index);

                    if (hexDataLen > 0 && hexDataLen < 253) {
                        if (tmp == seq) //only read data when packet number is the next expected SEQ number
                        {
                            uint8_t sendBufLen = prepareSendBuffer(input + index + 8, sendBuf, hexDataLen,
                                                                   seq); //extract HEX data from input to BYTE data into sendBuf (go from 2 HEX bytes to 1 byte), +8 jumps over the header to the HEX raw data
//Serial.print(F("PREP "));Serial.print(sendBufLen); Serial.print(F(" > ")); PrintHex83(sendBuf, sendBufLen);

//SEND RADIO DATA
//                            Serial.print(F("   #before sendHEXPacket"));
                            if (sendHEXPacket(remoteID, sendBuf, sendBufLen, seq, timeout, ACKTIMEOUT, debug)) {
                                sprintf((char *) sendBuf, "FLX:%u:OK", seq);
                                Serial.println((char *) sendBuf); //response to host
                                seq++;
                            } else return false;
                        }
                    }
//else Serial.print(F("FLX:INV"));
                    else {
                        Serial.print(F("FLX:INV:"));
                        Serial.println(hexDataLen);
                    }
                }
                if (inputLen == 7 && input[3] == '?' && input[4] == 'E' && input[5] == 'O' && input[6] == 'F') {
                    //TODO SEND RADIO EOF
                    return HandleSerialHandshake(targetID, true, timeout, ACKTIMEOUT, debug);
                }
            }
        }

//abort FLASH sequence if no valid packet received for a long time
        timeoutcheck:
        if (millis() - now > timeout) {
            Serial.print(F("Timeout getting FLASH image from SERIAL, aborting.."));
//send abort msg or just let node timeout as well?
            return false;
        }
    }
    return true;
}

//===================================================================================================================
// validateHEXData() - returns length of HEX data bytes if everything is valid
//returns 0 if any validation failed
//===================================================================================================================
uint8_t SerialOta::validateHEXData(void* data, uint8_t length)
{
    //assuming 1 byte record length, 2 bytes address, 1 byte record type, N data bytes, 1 CRC byte
    char* input = (char*)data;
    if (length <12 || length%2!=0) return 0; //shortest possible intel data HEX record is 12 bytes
    //Serial.print(F("VAL > ")); Serial.println((char*)input);

    uint8_t checksum=0;
    //check valid HEX data and CRC
    for (uint8_t i=0; i<length;i++)
    {
        if (!((input[i] >=48 && input[i]<=57) || (input[i] >=65 && input[i]<=70))) //0-9,A-F
            return 255;
        if (i%2 && i<length-2) checksum+=BYTEfromHEX(input[i-1], input[i]);
    }
    checksum=(checksum^0xFF)+1;

    //TODO : CHECK for address continuity (intel HEX addresses are big endian)

    //Serial.print(F("final CRC:"));Serial.println((uint8_t)checksum, HEX);
    //Serial.print(F("CRC byte:"));Serial.println(BYTEfromHEX(input[length-2], input[length-1]), HEX);

    //check CHECKSUM byte
    if (((uint8_t)checksum) != BYTEfromHEX(input[length-2], input[length-1]))
        return 254;

    uint8_t dataLength = BYTEfromHEX(input[0], input[1]); //length of actual HEX flash data (usually 16bytes)
    //calculate record length
    if (length != dataLength*2 + 10) //add headers and checksum bytes (a total of 10 combined)
        return 253;

    return dataLength; //all validation OK!
}

//===================================================================================================================
// prepareSendBuffer() - returns the final size of the buf
//===================================================================================================================
uint8_t SerialOta::prepareSendBuffer(char* hexdata, uint8_t*buf, uint8_t length, uint16_t seq)
{
    uint8_t seqLen = sprintf(((char*)buf), "FLX:%u:", seq);
    for (uint8_t i=0; i<length;i++)
        buf[seqLen+i] = BYTEfromHEX(hexdata[i*2], hexdata[i*2+1]);
    return seqLen+length;
}

//===================================================================================================================
// BYTEfromHEX() - converts from ASCII HEX to byte, assume A and B are valid HEX chars [0-9A-F]
//===================================================================================================================
uint8_t SerialOta::BYTEfromHEX(char MSB, char LSB)
{
    return (MSB>=65?MSB-55:MSB-48)*16 + (LSB>=65?LSB-55:LSB-48);
}

