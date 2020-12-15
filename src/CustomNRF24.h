#ifndef NRF24_H_
#define NRF24_H_

#include "Arduino.h"
#include <SPI.h> 	//must be included in the ino file (where the main is)
#include "CustomNRF24_Register.h"

class CustomNRF24
{
  private:
    bool dynamic_payloads_enabled;
  public:
    CustomNRF24();
    ~CustomNRF24();
    void setPins(uint8_t cePin, uint8_t csPin);
    void init(uint8_t payloadSize);
    inline void setCeLow() {digitalWrite(mCePin,LOW);} 
    inline void setCeHigh() {digitalWrite(mCePin,HIGH);} 
    inline void setCsLow() {digitalWrite(mCsPin,LOW);} 
    inline void setCsHigh() {digitalWrite(mCsPin,HIGH);}
    inline void switchFreq(uint8_t freq){writeRegister(RF_CH, freq);}
    inline bool rxFlag(){return (readRegister(STATUS) & _BV(RX_DR));} 
    inline void resetRxFlag(){writeRegister(STATUS, _BV(RX_DR));}
    inline bool rxEmpty(){return (readRegister(FIFO_STATUS) & _BV(RX_EMPTY));} 
    uint8_t readPayload(void* buf, uint8_t len);
    uint8_t writePayload(const void* buf, uint8_t len);
    uint8_t rxMode(uint8_t initialFreq);
    uint8_t txMode(uint8_t initialFreq);
    uint8_t flushRx();
    uint8_t flushTx();
    void activate(uint8_t code);
    uint8_t readRegister(uint8_t reg, uint8_t* buf, uint8_t len);
    uint8_t readRegister(uint8_t reg);
    uint8_t writeRegister(uint8_t reg, const uint8_t* buf, uint8_t len);
    uint8_t writeRegister(uint8_t reg, uint8_t value);
  protected:  
    uint8_t mCePin;
    uint8_t mCsPin;
    uint8_t mPayloadSize;
};

#endif /* NRF24_H_ */
