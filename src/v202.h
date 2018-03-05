#ifndef V202_H_
#define V202_H_

#include "Arduino.h"
#include "nRF24.h"

typedef struct __attribute__((__packed__)) {
  int8_t throttle;
  int8_t steering;
  int8_t trim_throttle;
  int8_t trim_steering;
  uint8_t flags;
  uint8_t crc;
} rx_values_t;

enum mode
{
  RX = 0,
  TX
};

enum rxState
{
  NO_BIND = 0,
  WAIT_FIRST_SYNCHRO,
  BOUND,
  SIGNAL_LOST
};


enum rxReturn
{
  BOUND_NEW_VALUES = 0,   // Bound state, frame received with new TX values
  BOUND_NO_VALUES,        // Bound state, no new frame received
  NOT_BOUND,              // Not bound, initial state
  BIND_IN_PROGRESS,       // Bind in progress, first frame has been received with TX id, wait no bind frame
  ERROR_SIGNAL_LOST,      // Signal lost
  UNKNOWN                 // ???, not used for moment
};

class v202
{
public:
  v202();
  ~v202();

  void init(nRF24 *wireless, uint8_t mode);
  uint8_t run(rx_values_t *rx_value );
  void setTXId(uint8_t txid[3]);
  void command(uint8_t throttle, int8_t yaw, int8_t pitch, int8_t roll, uint8_t flags);
  
protected:
  void retrieveFrequency();
  bool checkCRC();
  uint8_t getCRC();
  inline bool checkTXaddr() { return ( mTxid[0] == mFrame[7] && mTxid[1] == mFrame[8] && mTxid[2] == mFrame[9]);}

  nRF24 *mWireless;
  uint8_t mTxid[3];
  uint8_t mRfChannels[16];
  uint8_t mRfChNum;
  uint8_t mFrame[16];
  uint8_t mState;
  uint8_t mMode;
  uint8_t mErrorTimeoutCode;
  uint16_t mTimeout;
  bool packet_sent;
  unsigned long mLastSignalTime;
};
#endif /* V202_H_ */
