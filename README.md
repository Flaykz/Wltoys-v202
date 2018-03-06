# Wltoys-v202
## Goal
Send or receive wltoys v202 paquets with NRF24L01 chip.

This arduino library can thanks to a nrf24L01 chip :
* Receive and decodes frames from the v202/v222/v262/v282... transmitter.
* Create and send v202 frames from transmitter to the original Wltoys receiver.

I want to thank execuc, Rivig and deviationTX community for sharing their code :
* https://github.com/execuc/v202-receiver
* https://bitbucket.org/rivig/v202/src
* https://github.com/DeviationTX/deviation/blob/master/src/protocol/v202_nrf24l01.c

This code has not been tested enough and it is not reliable. So don't use it with dangerous rc model as planes, helicopters, cars...

## Hardware
You juste need an arduino uno/pro mini/nano and a nrf24l01+ chip.  Connect SCK, MISO, MOSI on pins D13, D12 and D11.   Then connect CE and CS on digital pins you have chosen in the code (see the `wireless.setPins()` method below).  Finally connect nrf24l01 VCC and GND to arduino 3.3V and GND pins.

## Use
There are two classes : 

 * CustomNRF24 :    handle the spi protocol to communicate with the nrf24l01p chip (rx and tx)
 * V202 :           handle the v2xx protocol
	
You need to initilize these class like that :


    CustomNRF24 nrfChip;
    V202 protocolV202;

The nrf24l01 pins must be defined in arduino setup function. `setPins` method arguments define the nrf24L01 CE (chip enable) and CS (SPI chip select) pins in this order. In my example, I did not use SS arduino pin as CS but D7 but SS pin must be set to output to activate the SPI mode to master.

    void setup() {
      // SS pin must be set as output to set SPI to master !
      pinMode(SS, OUTPUT);
      Serial.begin(115200);
      // Set CS pin to D7 and CE pin to D8
      nrfChip.setPins(8,7);
      protocolV202.init(&nrfChip);
      ...
    }
	
SPI wrapper (CustomNRF24 class) are linked to the protocol in the setup function

    protocolV202.init(&nrfChip);

In the loop function0, v202Protocol Run() method must be called at most every 4ms with & `rx_values_t` structure:

    uint8_t value = protocolV202.run(&rxValues); 
	
This function has several kind of returns from an enum :

    enum rxReturn
    {
       BOUND_NEW_VALUES = 0,   // Bound state, frame received with new TX values
       BOUND_NO_VALUES,        // Bound state, no new frame received
       NOT_BOUND,              // Not bound, initial state
       BIND_IN_PROGRESS,       // Bind in progress, first frame has been received with TX id, wait no bind frame.
       ERROR_SIGNAL_LOST,      // Signal lost
       UNKNOWN                 // Not used for moment
    };
	
When a frame is received (`BOUND_NEW_VALUES`), `rx_values_t` structure can be read :

    typedef struct __attribute__((__packed__)) {
      uint8_t throttle;
      int8_t yaw;
      int8_t pitch;
      int8_t roll;
      int8_t trim_yaw;
      int8_t trim_pitch;
      int8_t trim_roll;
      uint8_t flags;
    } rx_values_t;

Four axis, 3 trims values and flags are available. Last value depends on buttons pushed on the transmitter.
	
## Improvements 
There are many improvements to do :
- [ ] Make more reliable code, first.
- [ ] Handle signal lost : return to initial state for example.
- [ ] Make a better state machine for this protocol with a re-factor of the code and a smaller footprint.
- [ ] Reduce execution time. State machine takes 120us when a frame arrived and 32ms when there are nothing to do.
- [ ] Implement this protocol in the multiwii project, if it useful.
- [ ] Configure AETR or TAER order.