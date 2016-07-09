## Hardware needed:
- [Arduino Uno](https://www.arduino.cc/en/main/arduinoBoardUno)
- [Bluefruit LE UART friend](https://learn.adafruit.com/introducing-the-adafruit-bluefruit-le-uart-friend/introduction)
- [Rotary encoder (com-10982)](https://www.sparkfun.com/products/10982)
- [Blackberry Trackballer](https://www.sparkfun.com/products/13169)

## Setting things up
0. Download and install Arduino IDE (tested with 1.6.8)
0. Download and install Adafruit_BluefruitLE_nRF51 library
0. Download and install PinChangeInterrupt library (v.1.2.4)
0. In `SoftwareSerial.cpp` (`PATH/TO/ARDUINO/hardware/arduino/avr/libraries/SoftwareSerial/src`), comment out the following code:

  ```cpp
  #if defined(PCINT1_vect)
  ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect));
  #endif

  #if defined(PCINT2_vect)
  ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect));
  #endif
  ```

0. Connect to Bluefruit LE UART friend according to:
  https://learn.adafruit.com/introducing-the-adafruit-bluefruit-le-uart-friend/wiring
0. Connect the trackball and encoder according to pin assignments in `silverfish.h`
