#include <SoftwareSerial.h>
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE 0

#define upPin     2
#define downPin   3
#define leftPin   4
#define rightPin  5
#define btn       6
#define ENC_A     14
#define ENC_B     15
#define ENC_PORT  PINC
#define TB_GND    7

#define RESETKEYS   "AT+BLEKEYBOARDCODE=00"
#define SELECT      "AT+BleHidControlKey=0x0041"
#define KEYHEADER   "AT+BLEKEYBOARDCODE=00-00-"
#define KEYENDSHORT "00-00"
#define KEYENDLONG  "00-00-00"

#define DOWN  "51-"
#define UP    "52-"
#define LEFT  "50-"
#define RIGHT "4F-"

#define THRESHOLD_X 5
#define THRESHOLD_Y 5

static int upCount;
static int downCount;
static int leftCount;
static int rightCount;
static int btnCount;
static uint8_t encoderCount = 0;

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);
Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                              BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

/* Entry point */
void setup(void);

/* Main Loop */
void loop(void);

/* Setup functions */
void initializeSerial(void);
void resetCounters(void);
void setAllThePinsNeeded(void);
void attachInterruptToPins(void);

/* Interrupt handlers */
void upChange(void);
void downChange(void);
void leftChange(void);
void rightChange(void);
void btnChange(void);
void readEncoder(void);

/* Data processing */
float getOrientationFromEncoder(void);
bool isButtonPressed(void);
float getXwithTransformation(float angle);;
float getYwithTransformation(float angle);

/* Communications */
void sendBluetoothKeys(float x, float y, boolean pressed);
void sendResetKey(float x, float y);

/* Bluefruit functions */
void error(const __FlashStringHelper*err);
void initializeBluefruit(void);
