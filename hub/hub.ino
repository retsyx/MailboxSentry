#include "HomeSpan.h"

#define HOUR_TO_MIN(T) ((T)*60)
#define MIN_TO_SEC(T) ((T)*60)
#define SEC_TO_MSEC(T) ((T) * 1000)
#define MIN_TO_MSEC(T) (SEC_TO_MSEC(MIN_TO_SEC(T)))
#define HOUR_TO_MSEC(T) MIN_TO_MSEC(HOUR_TO_MIN(T))

const int control_pin = 2;
const int status_led_pin = 23;

struct msg_t {
  uint16_t battery_level;
  bool motion_detected;
};

volatile bool have_msg;
msg_t msg;
SpanPoint * remote;

struct RemoteMotionSensor : Service::MotionSensor {
  const char * name;
  SpanCharacteristic * motion_detected;
  SpanCharacteristic * fault;

  RemoteMotionSensor(const char * name) : Service::MotionSensor() {
    this->name = name;
    motion_detected = new Characteristic::MotionDetected();
    fault = new Characteristic::StatusFault(1);
  }

  void loop() {
    if (have_msg) {
      motion_detected->setVal(msg.motion_detected);
      fault->setVal(0);
      Serial.printf("Sensor %s update: %d\n", name, msg.motion_detected);
    } else if (motion_detected->getVal() && remote->time() > SEC_TO_MSEC(10)) {
      motion_detected->setVal(false);
    } else if (remote->time() > HOUR_TO_MSEC(6) && !fault->getVal()) {
      fault->setVal(1);
      Serial.printf("Sensor %s update: FAULT\n", name);
    }
  }
};

struct RemoteBatteryService : Service::BatteryService {
  SpanCharacteristic * battery_level;
  SpanCharacteristic * status_low_battery;
  SpanCharacteristic * charging_state;

  RemoteBatteryService() : Service::BatteryService() {
    battery_level = new Characteristic::BatteryLevel(100);
    status_low_battery = new Characteristic::StatusLowBattery(0);
    charging_state = new Characteristic::ChargingState(0);
  }

  void loop() {
    if (have_msg) {
      // Battery levels.
      // The sensor measures a value of 2000 when the battery is fully charged at 4.2V
      // This corresponds to roughly 476 [units] / V
      // Battery will be considered fully discharged at 3.5V = 1666 [units]
      // Mapping to 100%, where 3.5V is 0%, and 4.2V is 100% yields:
      // P = 100 * (U - 1666) / (2000 - 1666) = (U - 1666) * 100 / 334

      uint16_t level = msg.battery_level;
      bool is_low = false;
      float v = level / 476.0f;
      if (level < 1666) {
        level = 1666;
      } else if (level > 2000) {
        level = 2000;
      }
      uint16_t p = (level - 1666) * 100 / 334;
      if (p < 20) {
        is_low = true;
      }
      battery_level->setVal(p);
      status_low_battery->setVal(is_low);
      Serial.printf("Battery level %d%% (%.2fV) is_low %d\n", p, v, is_low);
    }
  }
};

void setup() {
  btStop();
  WiFi.setHostname("MailboxSentry");

  // Start serial
  Serial.begin(115200);

  Serial.print("Start CPU frequency: ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");

  setCpuFrequencyMhz(80);

  Serial.print("Updated CPU frequency: ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");

  //homeSpan.setControlPin(control_pin);
  //homeSpan.setStatusPin(status_led_pin);
  homeSpan.setApSSID("Sentry");
  homeSpan.setApPassword("sentry");
  homeSpan.enableAutoStartAP();

  homeSpan.begin(Category::Sensors, "Mailbox Sentry", "MailboxSentry");

  new SpanAccessory();
    new Service::AccessoryInformation();
      new Characteristic::Identify();
      new Characteristic::Manufacturer("github.com/retsyx");
      new Characteristic::SerialNumber("1");
      new Characteristic::Model("Mailbox Sentry");
      new Characteristic::FirmwareRevision("1.0");
    new RemoteBatteryService();
    new RemoteMotionSensor("Device 1");

  const char * macAddress = "12:34:56:78:9A:BC";
  remote = new SpanPoint(macAddress, 0, sizeof(msg_t));
}

void loop() {
  have_msg = remote->get(&msg);
  if (have_msg) {
    Serial.printf("Have message %d %d\n", msg.motion_detected, msg.battery_level);
  }
  homeSpan.poll();
}

