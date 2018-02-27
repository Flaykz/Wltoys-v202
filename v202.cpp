#include "v202.h"

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

v202::v202()
{
  mTxid[0]=0;
  mTxid[1]=0;
  mTxid[2]=0;
  mTimeout = 9999;
  mErrorTimeoutCode = NO_ERROR;
  
  for(uint8_t i = 0; i < 8; ++i)
  {
    uint8_t freq = 0x18 + (i << 3);
    freq_test[i] = (freq & 0x0F) ? freq : freq - 3;
  }
  
  mState = NO_BIND;
  mLastSignalTime = 0;
  mRfChNum = 0;
}

v202::~v202()
{
  /*EMPTY*/
}

void v202::init(nRF24 *wireless, uint8_t mode)
{
  mMode = mode
  mWireless = wireless;
  mWireless->init(16);
  delayMicroseconds(100);
  if (mMode == RX)
    mWireless->rxMode(freq_test[0]);
  else
    mWireless->txMode(freq_test[0]);
    mRfChNum = 0;
  mLastSignalTime = millis();
}

void v202::command(uint8_t throttle, int8_t yaw, int8_t pitch, int8_t roll, uint8_t flags)
{
  uint8_t buf[16];
  if (flags == 0xc0) {
    // binding
    buf[0] = 0x00;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = 0x00;
  } else {
    // regular packet
    buf[0] = throttle;
    buf[1] = (uint8_t) yaw;
    buf[2] = (uint8_t) pitch;
    buf[3] = (uint8_t) roll;
    // Trims, middle is 0x40
    buf[4] = 0x40; // yaw
    buf[5] = 0x40; // pitch
    buf[6] = 0x40; // roll
  }
  // TX id
  buf[7] = txid[0];
  buf[8] = txid[1];
  buf[9] = txid[2];
  // empty
  buf[10] = 0x00;
  buf[11] = 0x00;
  buf[12] = 0x00;
  buf[13] = 0x00;
  //
  buf[14] = flags;
  uint8_t sum = 0;
  uint8_t i;
  for (i = 0; i < 15;  ++i) sum += buf[i];
  buf[15] = getCRC(buf);
  if (packet_sent) {
    bool report_done = false;
    while (!(mWireless.read_register(STATUS) & _BV(TX_DS))) ;
    mWireless.write_register(STATUS, _BV(TX_DS));
  }
  packet_sent = true;
  uint8_t rf_ch = mRfChannels[mRfChNum >> 1];
  mRfChNum++; if (mRfChNum >= 32) mRfChNum = 0;
  mWireless.write_register(RF_CH, rf_ch);
  mWireless.flush_tx();
  mWireless.write_payload(buf, 16);
  delayMicroseconds(15);
}

void v202::setTXId(uint8_t txid[3])
{

  mTxid[0] = txid[0];
  mTxid[1] = txid[1];
  mTxid[2] = txid[2];
  retrieveFrequency();
}

void v202::retrieveFrequency()
{
  uint8_t sum;
  sum = mTxid[0] + mTxid[1] + mTxid[2];
  // Base row is defined by lowest 2 bits
  uint8_t (&fh_row)[16] = freq_hopping[sum & 0x03];
  // Higher 3 bits define increment to corresponding row
  uint8_t increment = (sum & 0x1e) >> 2;
  for (int i = 0; i < 16; ++i) {
    uint8_t val = fh_row[i] + increment;
    // Strange avoidance of channels divisible by 16
    mRfChannels[i] = (val & 0x0f) ? val : val - 3;
  }
}

bool v202::checkCRC()
{  
  return (getCRC(mFrame) == mFrame[15]);
}

uint8_t v202::getCRC(uint8_t buf[16])
{  
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 15; ++i)
    sum+= buf[i];
  return sum;
}

// loop function, can be factorized (for later)
uint8_t v202::run( rx_values_t *rx_value )
{
  uint8_t returnValue = UNKNOWN;
  switch(mState)
  {
    case BOUND:
    {
      bool incrementChannel = false;
      returnValue = BOUND_NO_VALUES;
      unsigned long newTime = millis();
      if( mWireless->rxFlag() )
      {
        mWireless->resetRxFlag();
        
	      uint8_t lastCRC = 0;
        while ( !mWireless->rxEmpty() )
        {
          mWireless->readPayload(mFrame, 16);

          if( checkCRC() && checkTXaddr() )
          {
	          if(incrementChannel && lastCRC == mFrame[15])
			        continue;

        		if(!incrementChannel)
        		{
        		  mRfChNum++;
        		  if( mRfChNum > 15) 
        		    mRfChNum = 0;
        		  mWireless->switchFreq(mRfChannels[mRfChNum]);
        		 incrementChannel = true;
        		 mTimeout = NORMAL_TIMEOUT;
        		 mErrorTimeoutCode = NO_ERROR;
        		}

		        lastCRC = mFrame[15];

    		    mLastSignalTime = newTime;
    
    		    // a valid frame has been received
    		    //incrementChannel = true;
    		    // Discard bind frame
    		    if( mFrame[14] != 0xc0 )
    		    {
    		      // Extract values futuba:AETR spektrum:TAER
    		      returnValue = BOUND_NEW_VALUES;
    		      //mFrame[0]                           A
    		      rx_value->steering = mFrame[1];   //  E
    		      rx_value->throttle = mFrame[2];   //  T
    		      //mFrame[3]                       //  R
    		      rx_value->trim_steering = mFrame[4];//  
              rx_value->trim_throttle = mFrame[5];
    		      rx_value->flags = mFrame[14];
    		      rx_value->crc = mFrame[15];
    		      rx_value->frame = mFrame;
    		    }
          }

        }
      }

      
      if(incrementChannel == false && uint16_t(newTime - mLastSignalTime) > mTimeout)
      {
      	mErrorTimeoutCode++;

      	mLastSignalTime = millis();
      	uint8_t freq_jump =0;
      	if(mErrorTimeoutCode == ERROR_JUMP_FREQ)
      	{
      		freq_jump  = uint16_t(newTime - mLastSignalTime) / 8 + 1;
      		mTimeout = freq_jump * 8 + 6;
      	}
      	else if(mErrorTimeoutCode == ERROR_WAIT_PREV_FREQ)
      	{
      		freq_jump  = 10;
      		mTimeout = 120;
      	}
      	else
      	{
      		freq_jump = random(1, 15);
      		mTimeout = 250;
      	}
        
         mRfChNum+=freq_jump;
          if( mRfChNum > 15) 
            mRfChNum = mRfChNum % 16;
          mWireless->switchFreq(mRfChannels[mRfChNum]);
      }

      if(mErrorTimeoutCode > 0)
        returnValue = ERROR_SIGNAL_LOST;
    }
    break;
    // Initial state
    case NO_BIND:
    {
      returnValue = NOT_BOUND;
      unsigned long newTime = millis();
 
      if( !mWireless->rxFlag() )
      {
        // Wait 128ms before switching the frequency
        // 128ms is th time to a TX to emits on all this frequency
        if((newTime - mLastSignalTime) > 128)
        {
          mRfChNum++;
          if( mRfChNum > 7) 
            mRfChNum = 0;
          mWireless->switchFreq(freq_test[mRfChNum]);
          mLastSignalTime = newTime;
        }
      }
      else 
      {
        mWireless->resetRxFlag();
        bool bFrameOk = false;
        while ( !mWireless->rxEmpty() )
        {
          mWireless->readPayload(mFrame, 16);
                           
          if( checkCRC() && mFrame[14] == 0xC0 )
          {
            // Bind frame is OK
            mTxid[0] = mFrame[7];
            mTxid[1] = mFrame[8];
            mTxid[2] = mFrame[9];
            // Create TX frequency array
            retrieveFrequency();
            mRfChNum = 0;
            mWireless->switchFreq(mRfChannels[mRfChNum]);
            mLastSignalTime = newTime;  
            mState = WAIT_FIRST_SYNCHRO;
            mWireless->flushRx();
            returnValue = BIND_IN_PROGRESS;
            break;
          }
        }
      }
    }
    break;
    
    // Wait on the first frequency of TX
    case WAIT_FIRST_SYNCHRO:
      returnValue = BIND_IN_PROGRESS;
      if( mWireless->rxFlag() )
      {
        mWireless->resetRxFlag();
        bool incrementChannel = false;
        while ( !mWireless->rxEmpty() )
        {
          mWireless->readPayload(mFrame, 16);
          if( checkCRC() && mFrame[14] != 0xc0 && checkTXaddr())
          {
            incrementChannel = true;
            mState = BOUND;
          }
        }
        
        if(incrementChannel)
        {
          // switch channel
          mRfChNum++;
          if( mRfChNum > 15) 
            mRfChNum = 0;
          mWireless->switchFreq(mRfChannels[mRfChNum]);
        }
      }
    break;
    // Not implement for the moment
    case SIGNAL_LOST:
      returnValue = ERROR_SIGNAL_LOST;
    break;
    
    default:
    break;
  }
  
  return returnValue;
}