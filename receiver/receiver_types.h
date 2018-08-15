//Took from: https://github.com/jpiat/arduino/blob/master/LiFiReceiver/receiver_types.h

enum receiver_state {
  IDLE, //waiting for sync
  SYNC, //synced, waiting for STX
  START, //STX received
  DATA //receiving DATA
};
