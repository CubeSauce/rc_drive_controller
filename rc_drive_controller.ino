#include <IBusBM.h>
#include <FastIMU.h>
#include <PID_v1.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

struct Live_Telemetry_Bunch {
  float_t acceleration[3];
  float_t gyroscope[3];
  float_t gyro_z_ravg;
  int32_t iBUS_throttle;
  int32_t iBUS_steering;
  int32_t drift_control_steering;
  int32_t drift_control_throttle;
};

char buffer[4098] = "";
void message(const char *format, ...) {
  display.clearDisplay();
  display.setCursor(0, 0);

  va_list args;
  va_start(args, format);
    vsprintf(buffer, format, args);
  va_end(args);

  Serial.print(buffer);
  display.print(buffer);

  display.display();      // Show initial text
}

float_t moving_average(float alpha, float current, float average) {
  return (alpha * current) + ((1 - alpha) * average);
}

enum Hall_Sensor {
  HS_REAR = 0,
  HS_FRONT_LEFT = 1,
  HS_FRONT_RIGHT = 2,
  HS_COUNT
};

namespace Setup_Static {
  constexpr int32_t i2c_sda{ 12 };
  constexpr int32_t i2c_sck{ 11 };
  constexpr int32_t i2c_clock{ 400000 };

  constexpr int32_t hall_pins[HS_COUNT] = { 5, 6, 7 };

  constexpr int32_t throttle_output_pin{ 2 };
  constexpr int32_t steering_output_pin{ 3 };

  constexpr int32_t iBUS_pin{ 4 };
  constexpr int32_t iBUS_throttle_min{1000};
  constexpr int32_t iBUS_throttle_max{2000};

  constexpr int32_t imu_address{ 0x68 };
  constexpr int8_t imu_perform_calibration{ true };

  constexpr const char* ble_service_uuid = "f8a47c99-9dba-44ff-aedb-93feadc81de3";
  constexpr const char* live_telemetry_characteristic_uuid = "f8a47c99-9dba-44ff-aedc-93feadc81de3";
  constexpr const char* setup_characteristic_uuid = "f8a47c99-9dba-44ff-aedc-93feadc81de4";
}

struct Setup {
  uint32_t telemetry_send_interval = 50;

  int32_t output_interval{20};

  int8_t iBUS_throttle_channel{ 3 };
  int8_t iBUS_steering_channel{ 4 };
  int8_t iBUS_drift_control_toggle_channel{ 5 };
  int8_t iBUS_drift_control_amount_channel{ 6 };

  int32_t traction_control_rpm_threshold{ 30 };
  uint32_t traction_control_interval{ 10000 };
  double traction_control_P = 5000;
  double traction_control_I = 500;
};

struct Setup setup_dynamic;
class IBusBM iBUS;
class MPU9250 imu;

class BLEServer* ble_server = NULL;
class BLECharacteristic* setup_characteristic = NULL;
class BLECharacteristic* live_telemetry_characteristic = NULL;

namespace Inputs {
  int iBUS_throttle{ 1000 };
  int iBUS_steering{ 1500 };

  bool iBUS_drift_control_toggle{ false };
  int iBUS_drift_control_amount{ 1000 };
  float traction_control_slip_target{ 0.2 };
}

namespace State {
  bool is_iBUS_connected{ false };

  struct calData imu_calibration;
  struct AccelData imu_acceleration_data;
  struct GyroData imu_gyroscope_data;

  float gyro_z_running_avg{0};

  int32_t steering { 1500 };
  int32_t throttle { 1000 };

  int32_t drift_control_steering{ 1500 };
  int32_t drift_control_throttle{ 1000 };

  uint32_t hall_time[HS_COUNT] = { 0 };
  volatile int16_t hall_count[HS_COUNT] = { 0 };
  int32_t hall_rpm[HS_COUNT] = { 0 };
  float_t slip{ 0.0f };
  float_t cumulative_slip_error{ 0.0f };
  uint32_t traction_control_time{ 0 };
  int32_t traction_control_throttle{ 0 };

  uint32_t output_time{0};

  bool telemetry_device_connected = false;
  bool previous_telemetry_device_connected = false;
  Live_Telemetry_Bunch live_telemetry_bunch;
  uint32_t live_telemetry_send_time{ 0 };
}

struct Setup parse_setup(std::string string_setup) {
  struct Setup new_setup;
  Serial.println(string_setup.c_str());
  return new_setup;
}

class BLE_Server_Callbacks: public BLEServerCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      if (pCharacteristic == setup_characteristic) {
        struct Setup recieved_setup = parse_setup(pCharacteristic->getValue());
        setup_dynamic = recieved_setup;
      }
    }

    void onConnect(BLEServer* pServer) {
      State::telemetry_device_connected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      State::telemetry_device_connected = false;
    }
};

namespace Output_Values {
  int final_steering{ 0 };
  int final_throttle{ 0 };
}

namespace Output {
  Servo steering;
  Servo throttle;
}

void rear_hall_trigger() {
  State::hall_count[HS_REAR]++;
}

void front_left_hall_trigger() {
  State::hall_count[HS_FRONT_LEFT]++;
}

void front_right_hall_trigger() {
  State::hall_count[HS_FRONT_RIGHT]++;
}

void setup_display() {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.stopscroll();
}

void setup_hall() {
  attachInterrupt(Setup_Static::hall_pins[HS_REAR], rear_hall_trigger, RISING);
  attachInterrupt(Setup_Static::hall_pins[HS_FRONT_LEFT], front_left_hall_trigger, RISING);
  attachInterrupt(Setup_Static::hall_pins[HS_FRONT_RIGHT], front_right_hall_trigger, RISING);
}

void setup_imu() {
  int err = imu.init(State::imu_calibration, Setup_Static::imu_address);
  // Infinite loop, so we can't control vehicle with garbage data
  while (err != 0) {
    message("Failed to initalize IMU.\n");
    delay(1000);
  }

  const int duration = 5000;
  const int secs = duration/1000.0f;
  const int interval = 20;
  const int count = duration / interval;
  for (int i = 0; i < count; ++i) {
    const float elapsed = ((float)i / count) * secs;
    message("Keep vehicle level\n\t\t %f\n", secs - elapsed);
    delay(interval);
  }

  message("Calibrating!\n");
  imu.calibrateAccelGyro(&State::imu_calibration);

  err = imu.init(State::imu_calibration, Setup_Static::imu_address);
  // Infinite loop, so we can't control vehicle with garbage data
  while (err != 0) {
    message("Failed to initalize IMU after calibration.\n");
    delay(1000);
  }

  imu.setGyroRange(2000); //

  message("Calibration done!\n");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(State::imu_calibration.accelBias[0]);
  Serial.print(", ");
  Serial.print(State::imu_calibration.accelBias[1]);
  Serial.print(", ");
  Serial.println(State::imu_calibration.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(State::imu_calibration.gyroBias[0]);
  Serial.print(", ");
  Serial.print(State::imu_calibration.gyroBias[1]);
  Serial.print(", ");
  Serial.println(State::imu_calibration.gyroBias[2]);
}

void setup_ibus() {
  iBUS.begin(Serial1, IBUSBM_NOTIMER, Setup_Static::iBUS_pin);
}

void setup_output() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  Output::steering.setPeriodHertz(50);
  Output::steering.attach(Setup_Static::steering_output_pin);
  Output::steering.writeMicroseconds(1500);

  Output::throttle.setPeriodHertz(50);
  Output::throttle.attach(Setup_Static::throttle_output_pin);
  Output::throttle.writeMicroseconds(1000);
}

void setup_telemetry() {
  BLEDevice::init("RC Telemetry v0.1");

  // Renmae this as it's not only telemetry
  ble_server = BLEDevice::createServer();
  ble_server->setCallbacks(new BLE_Server_Callbacks());

  BLEService *pService = ble_server->createService(Setup_Static::ble_service_uuid);

  live_telemetry_characteristic = pService->createCharacteristic(
                      Setup_Static::live_telemetry_characteristic_uuid,
                      BLECharacteristic::PROPERTY_READ   |
                      //BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE);

  live_telemetry_characteristic->addDescriptor(new BLE2902());

  setup_characteristic = pService->createCharacteristic(
                      Setup_Static::setup_characteristic_uuid,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE);

  setup_characteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(Setup_Static::ble_service_uuid);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
}

void setup() {
  Serial.begin(115200);

  Wire.begin(Setup_Static::i2c_sda, Setup_Static::i2c_sck);
  Wire.setClock(Setup_Static::i2c_clock);  //400khz clock

  setup_display();
  setup_output(); // Do this here to initalize esc
  //setup_hall();
  setup_imu();
  setup_ibus();
  setup_telemetry();
}

void read_ibus() {
  if (State::is_iBUS_connected = Serial1.available()) {
    iBUS.loop();  // Update the values on IBUS object

    //message("%d,%d,%d,%d,%d,%d\n", iBUS.readChannel(0), iBUS.readChannel(1), iBUS.readChannel(2), iBUS.readChannel(3), iBUS.readChannel(4), iBUS.readChannel(5));
    Inputs::iBUS_throttle = iBUS.readChannel(setup_dynamic.iBUS_throttle_channel - 1);
    Inputs::iBUS_steering = iBUS.readChannel(setup_dynamic.iBUS_steering_channel - 1);
    Inputs::iBUS_drift_control_toggle = iBUS.readChannel(setup_dynamic.iBUS_drift_control_toggle_channel - 1) > 1500;
    Inputs::iBUS_drift_control_amount = iBUS.readChannel(setup_dynamic.iBUS_drift_control_amount_channel - 1);
  }
}

void read_imu() {
  imu.update();
  imu.getAccel(&State::imu_acceleration_data);
  imu.getGyro(&State::imu_gyroscope_data);

  State::gyro_z_running_avg = moving_average(0.8f, State::imu_gyroscope_data.gyroZ, State::gyro_z_running_avg);

  State::live_telemetry_bunch.acceleration[0] = State::imu_acceleration_data.accelX;
  State::live_telemetry_bunch.acceleration[1] = State::imu_acceleration_data.accelY;
  State::live_telemetry_bunch.acceleration[2] = State::imu_acceleration_data.accelZ;

  State::live_telemetry_bunch.gyroscope[0] = State::imu_gyroscope_data.gyroX;
  State::live_telemetry_bunch.gyroscope[1] = State::imu_gyroscope_data.gyroY;
  State::live_telemetry_bunch.gyroscope[2] = State::imu_gyroscope_data.gyroZ;

  State::live_telemetry_bunch.gyro_z_ravg = State::gyro_z_running_avg;

}

void read_inputs() {
  read_ibus();
  read_imu();
}

void process_ibus() {
  if (State::is_iBUS_connected) {
    State::throttle = Inputs::iBUS_throttle;
    State::steering = Inputs::iBUS_steering;

    State::live_telemetry_bunch.iBUS_throttle = State::throttle;
    State::live_telemetry_bunch.iBUS_steering = State::steering;
  } else {
    State::throttle = 1000;
    State::steering = 1500;

    State::live_telemetry_bunch.iBUS_throttle = 0;
    State::live_telemetry_bunch.iBUS_steering = 0;
  }
}

void process_inputs() {
  process_ibus();
}

void process_hall() {

  uint32_t current_time = micros();
  
  for (int h = 0; h < HS_COUNT; h++) {
    if (State::hall_count[h] >= 20) {
      State::hall_rpm[h] = 30 * 1000000 / (current_time - State::hall_time[h]) * State::hall_count[h];
      State::hall_time[h] = current_time;
      State::hall_count[h] = 0;
    }
  }

  const int32_t rear_rpm = State::hall_rpm[HS_REAR];
  const int32_t front_rpm = (State::steering > 1500) ? State::hall_rpm[HS_FRONT_RIGHT] : State::hall_rpm[HS_FRONT_LEFT];

  // Traction control
  if (current_time - State::traction_control_time > setup_dynamic.traction_control_interval) {
    State::traction_control_time = current_time;

    if (front_rpm > setup_dynamic.traction_control_rpm_threshold) {
      const float_t slip = (float_t)(front_rpm - rear_rpm) / front_rpm;
      const float_t constrained_slip = constrain(slip, 0, 1);

      State::slip = moving_average(0.8, State::slip, constrained_slip);
    }

    if (State::slip > Inputs::traction_control_slip_target) {
      const float_t slip_error = State::slip - Inputs::traction_control_slip_target;
      State::cumulative_slip_error += slip_error * setup_dynamic.traction_control_interval;

      int32_t slip_PID = setup_dynamic.traction_control_P * slip_error + setup_dynamic.traction_control_I * State::cumulative_slip_error;
      int32_t constrained_slip_correction = constrain(slip_PID, 0, 500);

      State::traction_control_throttle = constrain(Inputs::iBUS_throttle - constrained_slip_correction, Setup_Static::iBUS_throttle_min, Setup_Static::iBUS_throttle_max);
    }
    else {
      State::traction_control_throttle = Inputs::iBUS_throttle;
    }
  }
}

void process_imu() {
  if (Inputs::iBUS_drift_control_toggle) {
    const float drift_P = constrain(map(Inputs::iBUS_drift_control_amount, 1000, 2000, 0.0, 2.0), 0.0, 2.0);

    State::drift_control_steering = State::steering - drift_P * State::imu_gyroscope_data.gyroZ;
    State::drift_control_steering = constrain(State::drift_control_steering, 1000, 2000);

    State::drift_control_throttle = State::throttle - abs(State::imu_gyroscope_data.gyroZ) * 0.3f;  //100% is way too strong
    State::drift_control_throttle = constrain(State::drift_control_throttle, 1000, 2000);
    
    State::live_telemetry_bunch.drift_control_throttle = State::drift_control_throttle;
    State::live_telemetry_bunch.drift_control_steering = State::drift_control_steering;
  } 
}

void process_outputs() {
  message(
    "[%s] Reciever     \n"
    "[%s] Telemetry    \n"
    "[%s] Drift ctrl   \n"
    "[%s] Traction ctrl\n",
    State::is_iBUS_connected ? " ON" : "OFF",
    State::telemetry_device_connected ? " ON" : "OFF",
    Inputs::iBUS_drift_control_toggle ? " ON" : "OFF",
    "OFF"
  );

  if (Inputs::iBUS_drift_control_toggle) {
    Output_Values::final_throttle = State::drift_control_throttle;
    Output_Values::final_steering = State::drift_control_steering;
  } else {
    Output_Values::final_throttle = State::throttle;
    Output_Values::final_steering = State::steering;
  }
}

void process_sensors() {
  //process_hall();
  process_imu();
  process_outputs();
}

uint32_t t = 0;
void apply_outputs() {
    if (State::output_time - micros() > setup_dynamic.output_interval) {
      State::output_time = micros();

      if (Output::steering.readMicroseconds() != Output_Values::final_steering) {
        Output::steering.writeMicroseconds(Output_Values::final_steering);
      }
      if (Output::throttle.readMicroseconds() != Output_Values::final_throttle) {
        Output::throttle.writeMicroseconds(Output_Values::final_throttle);
      }
    }
}

void process_telemetry() {
      // notify changed value
    if (State::telemetry_device_connected) {
      if (micros() - State::live_telemetry_send_time > setup_dynamic.telemetry_send_interval) {
        State::live_telemetry_send_time = micros();
        live_telemetry_characteristic->setValue((uint8_t*)&State::live_telemetry_bunch, sizeof(Live_Telemetry_Bunch));
        live_telemetry_characteristic->notify();
      }
    }
    // disconnecting
    if (!State::telemetry_device_connected && State::previous_telemetry_device_connected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        ble_server->startAdvertising(); // restart advertising
        Serial.println("Start telemetry advertising");
        State::previous_telemetry_device_connected = State::telemetry_device_connected ;
    }
    if (State::telemetry_device_connected  && !State::previous_telemetry_device_connected) {
        // do stuff here on connecting
        State::previous_telemetry_device_connected = State::telemetry_device_connected ;
    }
}

void loop() {
  read_inputs();
  process_inputs();
  process_sensors();
  apply_outputs();
  process_telemetry();
}
