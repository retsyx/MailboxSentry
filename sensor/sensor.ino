#include <Wire.h>
#include "SparkFun_BMA400_Arduino_Library.h"
#include "HomeSpan.h"

#define HOUR_TO_MIN(T) ((T)*60ull)
#define MIN_TO_SEC(T) ((T)*60ull)
#define SEC_TO_MSEC(T) ((T) * 1000ull)
#define MIN_TO_MSEC(T) (SEC_TO_MSEC(MIN_TO_SEC(T)))
#define HOUR_TO_MSEC(T) MIN_TO_MSEC(HOUR_TO_MIN(T))
#define SEC_TO_USEC(T) ((T)*1000ull*1000ull)
#define MIN_TO_USEC(T) (SEC_TO_USEC(MIN_TO_SEC(T)))
#define HOUR_TO_USEC(T) MIN_TO_USEC(HOUR_TO_MIN(T))

struct msg_t {
  uint16_t battery_level;
  bool motion_detected;
};

// I2C address selection
uint8_t i2c_addr = BMA400_I2C_ADDRESS_DEFAULT;  // 0x14

// Pin used for interrupt detection (must be RTC GPIO for wakeup)
const int interrupt_pin = 5;

const int battery_pin = 1;

// Define I2C pins (modify if using different pins)
const int SDA_PIN = 6;
const int SCL_PIN = 7;

BMA400 accelerometer;

SpanPoint *hub;

esp_sleep_wakeup_cause_t wakeup_reason;

#define ENABLE_SERIAL 1

#if ENABLE_SERIAL

#define serial_begin() do { Serial.begin(115200); delay(1000); } while (false)
#define printf(...) Serial.printf(__VA_ARGS__)

#else

#define serial_begin()
#define printf(...)

#endif

void setup() {
  serial_begin();

  // Get the wakeup reason
  wakeup_reason = esp_sleep_get_wakeup_cause();

  // Start serial
  printf("MAC Address = %s\n", Network.macAddress().c_str());

  // Mac address of hub
  hub = new SpanPoint("12:34:56:78:9A:BC", sizeof(msg_t), 0);

  homeSpan.setLogLevel(1);

  // Initialize the I2C library
  Wire.begin(SDA_PIN, SCL_PIN);

  // Check if sensor is connected and initialize
  while (accelerometer.beginI2C(i2c_addr) != BMA400_OK) {
    printf("Error: BMA400 not connected, check wiring and I2C address!\n");
    delay(1000);
  }

  printf("BMA400 connected!\n");

  msg_t msg = {};
  msg.battery_level = analogRead(battery_pin);
  printf("Battery level %d\n", msg.battery_level);

  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    printf("Woke up due to ext1 interrupt!\n");
    msg.motion_detected = true;
  } else {
    msg.motion_detected = false;
    printf("Power on or other wakeup.\n");
  }

  bool success = send(msg);

  if (msg.motion_detected && success) {
    delay(SEC_TO_MSEC(5));
    msg.motion_detected = false;
    send(msg);
  }

  goto_sleep();
}

bool send(msg_t & msg) {
  bool success;
  int count = 0;
  while (count++ < 5) {
    success = hub->send(&msg);
    printf("Send(%d) msg %d %d %s\n", count, msg.motion_detected, msg.battery_level, success ? "Succeded" : "Failed");
    if (success) {
      return true;
    }
    delay(1000);
  }
  return false;
}

#define CHECK(A) do { int x; if (x = (A)) printf("Error %d "#A"\n", x); } while (false)

void goto_sleep() {
  // Configure wakeup interrupt
  bma400_wakeup_conf wakeupConfig = {
    .wakeup_ref_update = BMA400_UPDATE_ONE_TIME,
    .sample_count = BMA400_SAMPLE_COUNT_8,
    .wakeup_axes_en = BMA400_AXIS_XYZ_EN,
    .int_wkup_threshold = 4,
    .int_wkup_ref_x = 255,
    .int_wkup_ref_y = 255,
    .int_wkup_ref_z = 255,
    .int_chan = BMA400_INT_CHANNEL_1
  };
  CHECK(accelerometer.setWakeupInterrupt(&wakeupConfig));

  // Configure the INT1 pin to push/pull mode, active high
  CHECK(accelerometer.setInterruptPinMode(BMA400_INT_CHANNEL_1, BMA400_INT_PUSH_PULL_ACTIVE_1));

  // Enable auto wakeup interrupt condition
  CHECK(accelerometer.enableInterrupt(BMA400_AUTO_WAKEUP_EN, true));

  // Set up automatic transition to low power mode
  bma400_auto_lp_conf autoLPConfig = {
    .auto_low_power_trigger = BMA400_AUTO_LP_TIME_RESET_EN,
    .auto_lp_timeout_threshold = 400  // 1 second timeout
  };
  CHECK(accelerometer.setAutoLowPower(&autoLPConfig));

  // Configure generic interrupt to reset auto low power timer
  bma400_gen_int_conf config = {
    .gen_int_thres = 5,
    .gen_int_dur = 1,
    .axes_sel = BMA400_AXIS_XYZ_EN,
    .data_src = BMA400_DATA_SRC_ACCEL_FILT_2,
    .criterion_sel = BMA400_ACTIVITY_INT,
    .evaluate_axes = BMA400_ANY_AXES_INT,
    .ref_update = BMA400_UPDATE_EVERY_TIME,
    .hysteresis = BMA400_HYST_48_MG,
    .int_thres_ref_x = 0,
    .int_thres_ref_y = 0,
    .int_thres_ref_z = 512,
    .int_chan = BMA400_UNMAP_INT_PIN
  };
  CHECK(accelerometer.setGeneric2Interrupt(&config));

  // Enable generic 2 interrupt condition
  CHECK(accelerometer.enableInterrupt(BMA400_GEN2_INT_EN, true));

  CHECK(accelerometer.setMode(BMA400_MODE_LOW_POWER));

  // Configure the ESP32 to wake up when interruptPin goes high
  esp_sleep_enable_ext1_wakeup((1 << interrupt_pin), ESP_EXT1_WAKEUP_ANY_HIGH);

  esp_sleep_enable_timer_wakeup(HOUR_TO_USEC(3ull));

  // Prepare for deep sleep by configuring GPIOs
  // Disable I2C to prevent leakage currents
  Wire.end();

  // Set I2C pins to input with no pull-up/down
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);

  // Disable internal pull-up/pull-down resistors
  gpio_pulldown_dis((gpio_num_t)SDA_PIN);
  gpio_pullup_dis((gpio_num_t)SDA_PIN);
  gpio_pulldown_dis((gpio_num_t)SCL_PIN);
  gpio_pullup_dis((gpio_num_t)SCL_PIN);

  printf("Entering deep sleep...\n");

  esp_deep_sleep_start();  // Enter deep sleep
}

