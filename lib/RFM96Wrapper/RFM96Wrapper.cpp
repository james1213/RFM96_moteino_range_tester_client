////
//// Created by ŁukaszLibront on 01.12.2023.
////
//
//#include "RFM96Wrapper.h"
//
//void basePacketReceivedAction();
//void basePacketSentAction();
//
//void (*packetReceivedAction)(void);
//void (*packetSentAction)(void);
//
//void initWrapper(Module *mod) {
//    setPacketReceivedAction(basePacketReceivedAction);
//    setPacketSentAction(basePacketSentAction);
//}
//
//
//bool ackReceived() {
//    return false;
//}
//
//bool ackTimeouted() {
//    return false;
//}
//
//void setPacketReceivedAction(void (*func)(void)) {
//    packetReceivedAction = func;
//}
//void setPacketSentAction(void (*func)(void)) {
//    packetSentAction = func;
//}
//
//void basePacketReceivedAction() {
//    String str;
//    int state = readData(str);
//
//
//    if (packetReceivedAction) {
//        packetReceivedAction();
//    }
//
//}
//void basePacketSentAction() {
//    if (packetSentAction) {
//        packetSentAction();
//    }
//}
