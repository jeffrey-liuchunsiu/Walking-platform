#include <Arduino.h>
#include "PinChangeInterrupt.h"
#include "silverfish.h"

/* Entry point */
void setup(void) {
  initializeSerial();
  initializeBluefruit();
  resetCounters();
  setAllThePinsNeeded();
  attachInterruptToPins();
}

/* Main Loop */
void loop(void) {
  noInterrupts();
  float angle = getOrientationFromEncoder();
  float x = getXwithTransformation(angle);
  float y = getYwithTransformation(angle);
  bool pressed = isButtonPressed();
  interrupts();

  sendBluetoothKeys(x, y, pressed);
  resetCounters();
  delay(200);
  sendResetKey(x, y);
}

/* Setup functions */
void initializeSerial(void) {
  Serial.begin(115200);
  Serial.println(F("Silverfish Controller!!"));
  Serial.println(F("-----------------------"));
}

void resetCounters(void) {
  upCount = 0;
  downCount = 0;
  leftCount = 0;
  rightCount = 0;
  btnCount = 0;
}

void setAllThePinsNeeded(void) {
  pinMode(upPin, INPUT_PULLUP);
  pinMode(downPin, INPUT_PULLUP);
  pinMode(leftPin, INPUT_PULLUP);
  pinMode(rightPin, INPUT_PULLUP);
  pinMode(btn, INPUT_PULLUP);
  pinMode(ENC_A, INPUT);
  digitalWrite(ENC_A, HIGH);
  pinMode(ENC_B, INPUT);
  digitalWrite(ENC_B, HIGH);
  pinMode(TB_GND, OUTPUT);
  digitalWrite(TB_GND, LOW);
}

void attachInterruptToPins(void) {
  attachPCINT(digitalPinToPCINT(upPin), upChange, CHANGE);
  attachPCINT(digitalPinToPCINT(downPin), downChange, CHANGE);
  attachPCINT(digitalPinToPCINT(leftPin), leftChange, CHANGE);
  attachPCINT(digitalPinToPCINT(rightPin), rightChange, CHANGE);
  attachPCINT(digitalPinToPCINT(btn), btnChange, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_A), readEncoder, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_B), readEncoder, CHANGE);
}

/* Interrupt handlers */
void upChange(void) {
  upCount = upCount + 1;
}

void downChange(void) {
  downCount = downCount + 1;
}

void leftChange(void) {
  leftCount = leftCount + 1;
}

void rightChange(void) {
  rightCount = rightCount + 1;
}

void btnChange(void) {
  btnCount = btnCount + 1;
}

void readEncoder() {
  static int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  static uint8_t old_AB = 0;

  old_AB <<= 2;                   //remember previous state
  old_AB |= ( ENC_PORT & 0x03 );  //add current state
  encoderCount += ( enc_states[( old_AB & 0x0f )]);
  if (encoderCount >= 180)
    encoderCount = encoderCount - 256 + 90;
  if (encoderCount >= 90)
    encoderCount = encoderCount - 90;
}

/* Data processing */
float getOrientationFromEncoder(void) {
  unsigned long angle = encoderCount;
  angle = angle * 360 / 90;
  return 3.1415 * angle / 180;
}

bool isButtonPressed(void) {
  return btnCount > 0;
}

float getXwithTransformation(float angle) {
  int ry = upCount - downCount;
  int rx = rightCount - leftCount;
  return rx * cos(angle) - ry * sin(angle);
}

float getYwithTransformation(float angle) {
  int ry = upCount - downCount;
  int rx = rightCount - leftCount;
  return rx * sin(angle) + ry * cos(angle);
}

/* Communications */
void sendBluetoothKeys(float x, float y, boolean pressed) {
  if ( pressed )
    ble.println(SELECT);

  if ( abs(y) >= THRESHOLD_Y or abs(x) >= THRESHOLD_X ) {
    ble.print(KEYHEADER);
    if ( y <= -THRESHOLD_Y )
      ble.print(DOWN);
    else if ( y >= THRESHOLD_Y )
      ble.print(UP);

    if ( x <= -THRESHOLD_X )
      ble.print(LEFT);
    else if ( x >= THRESHOLD_X )
      ble.print(RIGHT);

    if ( abs(y) >= THRESHOLD_Y and abs(x) >= THRESHOLD_X )
      ble.println(KEYENDSHORT);
    else
      ble.println(KEYENDLONG);
  }
}

void sendResetKey(float x, float y) {
  if ( abs(y) >= THRESHOLD_Y or abs(x) >= THRESHOLD_X )
    ble.println(RESETKEYS);
}

/* Bluefruit functions */
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void initializeBluefruit() {
  delay(500);
  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE ) {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() )
      error(F("Couldn't factory reset"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'Silverfish Controller': "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=Silverfish Controller" )) )
    error(F("Could not set device name?"));

  /* Enable HID Service */
  Serial.println(F("Enable HID Service (including Keyboard): "));
  if (! ble.sendCommandCheckOK(F( "AT+BleKeyboardEn=On"  )))
    error(F("Could not enable Keyboard"));
  if (! ble.sendCommandCheckOK(F( "AT+BLEHIDEN=On"  )))
    error(F("Failed to enable HID (firmware >=0.6.6?)"));

  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset())
    error(F("Couldn't reset??"));

  Serial.println();
  Serial.println(F("Go to your phone's Bluetooth settings to pair your device"));
  Serial.println();
}
