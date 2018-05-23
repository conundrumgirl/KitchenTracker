#include "ble_config.h"
/*
 * Provides skeleton code to interact with the Android FaceTrackerBLE app
 *
 * Created by Jon Froehlich, May 7, 2018
 *
 * Based on previous code by Liang He, Bjoern Hartmann,
 * Chris Dziemborowicz and the RedBear Team. See:
 * https://github.com/jonfroehlich/CSE590Sp2018/tree/master/A03-BLEAdvanced
 */

#if defined(ARDUINO)
SYSTEM_MODE(SEMI_AUTOMATIC);
#endif

#define RECEIVE_MAX_LEN 5 // TODO: change this based on how much data you are sending from Android
#define SEND_MAX_LEN 5

// Must be an integer between 1 and 9 and and must also be set to len(BLE_SHORT_NAME) + 1
#define BLE_SHORT_NAME_LEN 7

// device from everyone else's device in class.
#define BLE_SHORT_NAME 'A', 'L', 'I', 'N', 'A', '1'

/* Define the pins on the Duo board
 */
#define LED_ANALOG_OUT_PIN A0
#define PIEZO_OUT_PIN A1
#define SERVO_ANALOG_OUT_PIN D2
const int TRIG_PIN = D0;
const int ECHO_PIN = D1;
#define BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN D7

#define MAX_SERVO_ANGLE 180
#define MIN_SERVO_ANGLE 0
#define ALARM_THRESHOLD 50

volatile int _servoAngle;
volatile int age = 0;

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;

// servo
Servo _servo;

// Device connected and disconnected callbacks
void deviceConnectedCallback(BLEStatus_t status, uint16_t handle);
void deviceDisconnectedCallback(uint16_t handle);

// UUID is used to find the device by other BLE-abled devices
static uint8_t service1_uuid[16] = {0x71, 0x3d, 0x00, 0x00, 0x50, 0x3e, 0x4c, 0x75, 0xba, 0x94, 0x31, 0x48, 0xf1, 0x8d, 0x94, 0x1e};
static uint8_t service1_tx_uuid[16] = {0x71, 0x3d, 0x00, 0x03, 0x50, 0x3e, 0x4c, 0x75, 0xba, 0x94, 0x31, 0x48, 0xf1, 0x8d, 0x94, 0x1e};
static uint8_t service1_rx_uuid[16] = {0x71, 0x3d, 0x00, 0x02, 0x50, 0x3e, 0x4c, 0x75, 0xba, 0x94, 0x31, 0x48, 0xf1, 0x8d, 0x94, 0x1e};

// Define the receive and send handlers
static uint16_t receive_handle = 0x0000; // recieve
static uint16_t send_handle = 0x0000;    // send

static uint8_t receive_data[RECEIVE_MAX_LEN] = {0x01};
int bleReceiveDataCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size); // function declaration for receiving data callback
static uint8_t send_data[SEND_MAX_LEN] = {0x00};

// Define the configuration data
static uint8_t adv_data[] = {
    0x02,
    BLE_GAP_AD_TYPE_FLAGS,
    BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE,

    BLE_SHORT_NAME_LEN,
    BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
    BLE_SHORT_NAME,

    0x11,
    BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE,
    0x1e, 0x94, 0x8d, 0xf1, 0x48, 0x31, 0x94, 0xba, 0x75, 0x4c, 0x3e, 0x50, 0x00, 0x00, 0x3d, 0x71};

static btstack_timer_source_t send_characteristic;
static void bleSendDataTimerCallback(btstack_timer_source_t *ts); // function declaration for sending data callback
// 200ms (how often to read the pins and transmit the data to Android)
int _sendDataFrequency = 200;

//smoothing
const int SMOOTHING_WINDOW_SIZE = 5;
volatile int _readings[SMOOTHING_WINDOW_SIZE]; // the readings from the analog input
volatile int _readIndex = 0;                   // the index of the current reading
volatile int _total = 0;                       // the running total
volatile int _average = 0;

void setup()
{
  Serial.begin(115200);

  //sonic
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(LED_ANALOG_OUT_PIN, OUTPUT);

  //blincer
  digitalWrite(LED_ANALOG_OUT_PIN, HIGH);

  delay(5000);
  Serial.println("Kitchen Tracker.");

  // Initialize ble_stack.
  ble.init();

  // Register BLE callback functions
  ble.onConnectedCallback(bleConnectedCallback);
  ble.onDisconnectedCallback(bleDisconnectedCallback);

  //lots of standard initialization hidden in here - see ble_config.cpp
  configureBLE();

  // Set BLE advertising data
  ble.setAdvertisementData(sizeof(adv_data), adv_data);

  // Register BLE callback functions
  ble.onDataWriteCallback(bleReceiveDataCallback);

  // Add user defined service and characteristics
  ble.addService(service1_uuid);
  receive_handle = ble.addCharacteristicDynamic(service1_tx_uuid, ATT_PROPERTY_NOTIFY | ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, receive_data, RECEIVE_MAX_LEN);
  send_handle = ble.addCharacteristicDynamic(service1_rx_uuid, ATT_PROPERTY_NOTIFY, send_data, SEND_MAX_LEN);

  // BLE peripheral starts advertising now.
  ble.startAdvertising();
  Serial.println("BLE start advertising.");

  // Setup pins

  //pinMode(RIGHT_EYE_ANALOG_OUT_PIN, OUTPUT);
  pinMode(BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN, OUTPUT);
  _servo.attach(SERVO_ANALOG_OUT_PIN);

  pinMode(PIEZO_OUT_PIN, OUTPUT);

  _servoAngle = ((int)((MAX_SERVO_ANGLE - MIN_SERVO_ANGLE) / 2.0));
  _servo.write(_servoAngle);
  // Start a task to check status of the pins on your RedBear Duo
  // Works by polling every X milliseconds where X is _sendDataFrequency
  send_characteristic.process = &bleSendDataTimerCallback;
  ble.setTimer(&send_characteristic, _sendDataFrequency);
  ble.addTimer(&send_characteristic);
}

void loop()
{
}

/**
 * @brief Connect handle.
 *
 * @param[in]  status   BLE_STATUS_CONNECTION_ERROR or BLE_STATUS_OK.
 * @param[in]  handle   Connect handle.
 *
 * @retval None
 */
void bleConnectedCallback(BLEStatus_t status, uint16_t handle)
{
  switch (status)
  {
  case BLE_STATUS_OK:
    Serial.println("BLE device connected!");
    digitalWrite(BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN, HIGH);
    break;
  default:
    break;
  }
}

/**
 * @brief Disconnect handle.
 *
 * @param[in]  handle   Connect handle.
 *
 * @retval None
 */
void bleDisconnectedCallback(uint16_t handle)
{
  Serial.println("BLE device disconnected.");
  digitalWrite(BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN, LOW);
}

/**
 * @brief Callback for receiving data from Android (or whatever device you're connected to).
 *
 * @param[in]  value_handle
 * @param[in]  *buffer       The buffer pointer of writting data.
 * @param[in]  size          The length of writting data.
 *
 * @retval
 */
int bleReceiveDataCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size)
{
  //Serial.print("Getting something: ");
  if (receive_handle == value_handle)
  {
    memcpy(receive_data, buffer, RECEIVE_MAX_LEN);
    // Serial.print("Received data: ");
    for (uint8_t index = 0; index < RECEIVE_MAX_LEN; index++)
    {
      // Serial.print(receive_data[index]);
      //Serial.print(" ");
    }

    if (receive_data[0] == 0x04)
    { //receive the face data
      // we are getting age from MS AI
      age = receive_data[1];
      int new_angle = receive_data[2];
      if (new_angle > _servoAngle + 1 || new_angle < _servoAngle - 1)
      {
        //Serial.print("new_angle : ");
        //Serial.println(new_angle );

        _servo.write(new_angle);
        _servoAngle = new_angle;
      }
    }
  }
  return 0;
}

static void soundAlarm()
{
  digitalWrite(LED_ANALOG_OUT_PIN, LOW);
  tone(PIEZO_OUT_PIN, 261);
  delay(200);
  digitalWrite(LED_ANALOG_OUT_PIN, HIGH);
  noTone(PIEZO_OUT_PIN);
}

/**
 * @brief Timer task for sending status change to client.
 * @param[in]  *ts
 * @retval None
 *
 * Send the data from either analog read or digital read back to
 * the connected BLE device (e.g., Android)
 */
static void bleSendDataTimerCallback(btstack_timer_source_t *ts)
{

  //Serial.print("About to send distance Distance: ");
  byte b0 = (0x00);
  byte b1 = (0x00);
  byte b2 = (0x00);
  byte b3 = age;

  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  while ( digitalRead(ECHO_PIN) == 0 );

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min
  // TODO: We probably need to check for a timeout here just in case
  // the ECHO_PIN never goes HIGH... so like
  // while ( digitalRead(ECHO_PIN) == 1 && micros() - t1 < threshold);
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1);
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  // of sound in air at sea level (~340 m/s).
  // Datasheet: https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
  cm = pulse_width / 58.0;

  int distance = round(cm);
  distance = getSmoothedReading(distance);
  Serial.println(distance);
  if (distance > 400)
  { //out of range status
    b0 = (0x01);
  }
  else
  {
    if ((distance < ALARM_THRESHOLD) && (distance > 0) && (age> 0) && (age < 10))
      {
        soundAlarm();
      }

    b1 = distance / 255;
    b2 = distance % 255;
  }

  send_data[0] = b0;
  send_data[1] = b1;
  send_data[2] = b2;
  send_data[3] = b3;
  send_data[4] = _servoAngle;

  ble.sendNotify(send_handle, send_data, SEND_MAX_LEN);
  ble.setTimer(ts, _sendDataFrequency);
  ble.addTimer(ts);
}

/* From Jon's example but I also throw away the outliers */
int getSmoothedReading(int curReading)
{
  //get rid of obvious outliers
  /*if (abs(curReading - _readings[_readIndex]) > 100 && _readings[_readIndex] > 0)
  {
    return _readings[_readIndex];
  }*/
  _total = _total - _readings[_readIndex];
  _readings[_readIndex] = curReading;
  _total = _total + _readings[_readIndex];
  _readIndex = _readIndex + 1;
  // if we're at the end of the array...
  if (_readIndex >= SMOOTHING_WINDOW_SIZE)
  {
    _readIndex = 0;
  }

  _average = _total / SMOOTHING_WINDOW_SIZE;
  return _average;
}
