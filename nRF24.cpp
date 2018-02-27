#include "nRF24.h"

//#include <SPI.h>
//#include "nRF24L01.h"

nRF24::nRF24()
{
  /* EMPTY */
}

nRF24::~nRF24()
{
  /* EMPTY */
}

void nRF24::setPins(uint8_t cePin, uint8_t csPin)
{
  mCePin = cePin;
  mCsPin = csPin;
  pinMode(mCePin,OUTPUT);
  pinMode(mCsPin,OUTPUT);
  setCeLow();
  setCsHigh();
}

void nRF24::init(uint8_t payloadSize)
{
  // Initialize SPI
  SPI.begin();
  mPayloadSize = payloadSize;
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
}

uint8_t nRF24::rxMode(uint8_t initialFreq)
{
  setCeLow();
  writeRegister(CONFIG, _BV(EN_CRC) | _BV(CRCO));	// Enable CRC (2bytes)
  delayMicroseconds(100);
  writeRegister(EN_AA, 0x00);		// Disable auto acknowledgment
  writeRegister(EN_RXADDR, 0x01);	// Enable all data pipe (PO to P5)
  writeRegister(SETUP_AW, 0x03);	// 5 bytes address
  writeRegister(SETUP_RETR, 0xFF);	// 15 retransmit, 4000us pause
  writeRegister(RF_CH, initialFreq);	// channel 8
  writeRegister(RF_SETUP, 0x05); 	// 1Mbps, -6dBm power (0x06 => 1Mbps, 0dBm)
  writeRegister(STATUS, 0x70);		// Clear status register
  writeRegister(RX_PW_P0, mPayloadSize);	// RX payload of 16 bytes
  writeRegister(FIFO_STATUS, 0x00);	// Nothing useful for write command
  const uint8_t* rx_tx_addr = reinterpret_cast<const uint8_t *>("\x66\x88\x68\x68\x68");
  writeRegister(RX_ADDR_P0, rx_tx_addr, 5);
  delay(50);
  flushTx();
  flushRx();
  delayMicroseconds(100);
  writeRegister(CONFIG, _BV(EN_CRC) | _BV(CRCO) | _BV(PWR_UP)  );
  delayMicroseconds(100);
  writeRegister(CONFIG, _BV(EN_CRC) | _BV(CRCO) | _BV(PWR_UP) | _BV(PRIM_RX) );
  delayMicroseconds(100);
  setCeHigh();
  delayMicroseconds(100);
}

uint8_t nRF24::txMode(uint8_t initialFreq)
{
  setCeLow();
  writeRegister(CONFIG, _BV(EN_CRC) | _BV(CRCO));	// Enable CRC (2bytes)
  //delayMicroseconds(100);
  writeRegister(EN_AA, 0x00);		// Disable auto acknowledgment
  writeRegister(EN_RXADDR, 0x3F);
  writeRegister(SETUP_AW, 0x03);	// 5 bytes address
  writeRegister(SETUP_RETR, 0xFF);	// 15 retransmit, 4000us pause
  writeRegister(RF_CH, initialFreq);	// channel 8
  writeRegister(RF_SETUP, 0x05); 	// 1Mbps, -6dBm power (0x06 => 1Mbps, 0dBm)
  writeRegister(STATUS, 0x70);		// Clear status register
  //writeRegister(OBSERVE_TX, 0x00);
  //writeRegister(CD, 0x00);
  writeRegister(RX_ADDR_P2, 0xC3);
  writeRegister(RX_ADDR_P3, 0xC4);
  writeRegister(RX_ADDR_P4, 0xC5);
  writeRegister(RX_ADDR_P5, 0xC6);
  writeRegister(RX_PW_P0, mPayloadSize); // 0x10
  writeRegister(RX_PW_P1, mPayloadSize);
  writeRegister(RX_PW_P2, mPayloadSize);
  writeRegister(RX_PW_P3, mPayloadSize);
  writeRegister(RX_PW_P4, mPayloadSize);
  writeRegister(RX_PW_P5, mPayloadSize);
  writeRegister(FIFO_STATUS, 0x00);	// Nothing useful for write command
  const uint8_t* rx_tx_addr = reinterpret_cast<const uint8_t *>("\x66\x88\x68\x68\x68");
  const uint8_t* rx_p1_addr = reinterpret_cast<const uint8_t *>("\x88\x66\x86\x86\x86");
  writeRegister(RX_ADDR_P0, rx_tx_addr, 5);
  writeRegister(RX_ADDR_P1, rx_p1_addr, 5);
  writeRegister(TX_ADDR, rx_tx_addr, 5);
  activate(0x53); // magic for BK2421 bank switch
  Serial.write("Try to switch banks "); Serial.print(read_register(STATUS)); Serial.write("\n");
  if (read_register(STATUS) & 0x80) {
    Serial.write("BK2421 detected\n");
    long nul = 0;
    writeRegister(0x00, (const uint8_t *) "\x40\x4B\x01\xE2", 4);
    writeRegister(0x01, (const uint8_t *) "\xC0\x4B\x00\x00", 4);
    writeRegister(0x02, (const uint8_t *) "\xD0\xFC\x8C\x02", 4);
    writeRegister(0x03, (const uint8_t *) "\xF9\x00\x39\x21", 4);
    writeRegister(0x04, (const uint8_t *) "\xC1\x96\x9A\x1B", 4);
    writeRegister(0x05, (const uint8_t *) "\x24\x06\x7F\xA6", 4);
    writeRegister(0x06, (const uint8_t *) &nul, 4);
    writeRegister(0x07, (const uint8_t *) &nul, 4);
    writeRegister(0x08, (const uint8_t *) &nul, 4);
    writeRegister(0x09, (const uint8_t *) &nul, 4);
    writeRegister(0x0A, (const uint8_t *) &nul, 4);
    writeRegister(0x0B, (const uint8_t *) &nul, 4);
    writeRegister(0x0C, (const uint8_t *) "\x00\x12\x73\x00", 4);
    writeRegister(0x0D, (const uint8_t *) "\x46\xB4\x80\x00", 4);
    writeRegister(0x0E, (const uint8_t *) "\x41\x10\x04\x82\x20\x08\x08\xF2\x7D\xEF\xFF", 11);
    writeRegister(0x04, (const uint8_t *) "\xC7\x96\x9A\x1B", 4);
    writeRegister(0x04, (const uint8_t *) "\xC1\x96\x9A\x1B", 4);
  } else {
    Serial.write("nRF24L01 detected\n");
  }
  activate(0x53); // switch bank back
  delay(50);
  flushTx();
  delayMicroseconds(100);
  writeRegister(CONFIG, _BV(EN_CRC) | _BV(CRCO) | _BV(PWR_UP)  );
  delayMicroseconds(100);
  setCeHigh();
  delayMicroseconds(100);
}

uint8_t nRF24::readRegister(uint8_t reg, uint8_t* buf, uint8_t len)
{
  setCsLow();
  uint8_t result = SPI.transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- )
    *buf++ = SPI.transfer(0xff);
  setCsHigh();

  return result;
}

uint8_t nRF24::readRegister(uint8_t reg)
{
  setCsLow();
  SPI.transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
  uint8_t result = SPI.transfer(0xff);
  setCsHigh();
  
  return result;
}

uint8_t nRF24::writeRegister(uint8_t reg, const uint8_t* buf, uint8_t len)
{
  setCsLow();
  uint8_t result = SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- )
    SPI.transfer(*buf++);
  setCsHigh();

  return result;
}

uint8_t nRF24::writeRegister(uint8_t reg, uint8_t value)
{
  setCsLow();
  uint8_t result = SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  SPI.transfer(value);
  setCsHigh();

  return result;
}

uint8_t nRF24::writePayload(const void* buf, uint8_t len)
{
  uint8_t result;

  const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

  uint8_t data_len = min(len,mPayloadSize);
  uint8_t blank_len = mPayloadSize - data_len;
  //uint8_t blank_len = dynamic_payloads_enabled ? 0 : mPayloadSize - data_len;
  
  setCsLow();
  result = SPI.transfer( W_TX_PAYLOAD );
  while ( data_len-- )
    SPI.transfer(*current++);
  while ( blank_len-- )
    SPI.transfer(0);
  setCsHigh();

  return result;
}

uint8_t nRF24::readPayload(void* buf, uint8_t len)
{
  uint8_t result;
  uint8_t* current = reinterpret_cast<uint8_t*>(buf);

  uint8_t data_len = min(len,mPayloadSize);
  uint8_t blank_len = mPayloadSize - data_len;
  //uint8_t blank_len = dynamic_payloads_enabled ? 0 : mPayloadSize - data_len;
  
  setCsLow();
  result = SPI.transfer( R_RX_PAYLOAD );
  while ( data_len-- )
    *current++ = SPI.transfer(0xff);
  while ( blank_len-- )
    SPI.transfer(0xff);
  setCsHigh();

  return result;
}

uint8_t nRF24::flushTx(void)
{
  setCsLow();
  uint8_t result = SPI.transfer( FLUSH_TX );
  setCsHigh();

  return result;
}

uint8_t nRF24::flushRx(void)
{
  setCsLow();
  uint8_t result = SPI.transfer( FLUSH_RX );
  setCsHigh();

  return result;
}

void nRF24::activate(uint8_t code)
{
  setCsLow();
  SPI.transfer(ACTIVATE);
  SPI.transfer(code);
  setCsHigh();
}