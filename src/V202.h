#ifndef V202_H_
#define V202_H_

#include "Arduino.h"
#include "CustomNRF24.h"

#define TIME_TX_EMITS_ALL_FREQ 128

const uint16_t NORMAL_TIMEOUT = 11;
const uint8_t ERROR_JUMP_FREQ = 1;
const uint8_t ERROR_WAIT_PREV_FREQ = 2;
const uint8_t ERROR_WAIT_ONE_FREQ = 3;
const uint8_t NO_ERROR = 0;

static uint8_t freq_hopping[][16] = {
        { 0x27, 0x1B, 0x39, 0x28, 0x24, 0x22, 0x2E, 0x36,
                0x19, 0x21, 0x29, 0x14, 0x1E, 0x12, 0x2D, 0x18 }, //  00
        { 0x2E, 0x33, 0x25, 0x38, 0x19, 0x12, 0x18, 0x16,
                0x2A, 0x1C, 0x1F, 0x37, 0x2F, 0x23, 0x34, 0x10 }, //  01
        { 0x11, 0x1A, 0x35, 0x24, 0x28, 0x18, 0x25, 0x2A,
                0x32, 0x2C, 0x14, 0x27, 0x36, 0x34, 0x1C, 0x17 }, //  02
        { 0x22, 0x27, 0x17, 0x39, 0x34, 0x28, 0x2B, 0x1D,
                0x18, 0x2A, 0x21, 0x38, 0x10, 0x26, 0x20, 0x1F }  //  03
};

static uint8_t freq_test[8];

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

enum txType {
    AETR = 0,
    TAER
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

class V202
{
public:
  V202();
  ~V202();

  void init(CustomNRF24 *wireless, uint8_t mode, uint8_t txType);
  uint8_t run(rx_values_t *rx_value );
  void setTXId(uint8_t txid[3]);
  void command(uint8_t throttle, int8_t yaw, int8_t pitch, int8_t roll, uint8_t flags);
  
protected:
  void retrieveFrequency();
  bool checkCRC();
  bool isBindingFrame();
  uint8_t initBinding(uint8_t bindingState);
  uint8_t getCRC();
  inline bool checkTXaddr() { return ( mTxid[0] == mFrame[7] && mTxid[1] == mFrame[8] && mTxid[2] == mFrame[9]);}

  CustomNRF24 *mWireless;
  uint8_t mTxid[3];
  uint8_t mRfChannels[16];
  uint8_t mRfChNum;
  uint8_t mFrame[16];
  uint8_t mState;
  uint8_t mMode;
  uint8_t mTxType;
  uint8_t mErrorTimeoutCode;
  uint16_t mTimeout;
  bool packet_sent;
  unsigned long mLastSignalTime;
};
#endif /* V202_H_ */
