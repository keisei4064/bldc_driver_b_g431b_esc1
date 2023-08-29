#ifdef HOLE_SENSOR_VOLTAGE_FEEDBACK
// ホールセンサを用いたフィードバック制御（電圧を利用）

#include <Arduino.h>
#include <SimpleFOC.h>

constexpr int kLedPin = LED_RED;
constexpr int kPotentiometer{PB12};
constexpr int kHole1{PB6};
constexpr int kHole2{PB7};
constexpr int kHole3{PB8};

constexpr int kPhaseUH{PA8};
constexpr int kPhaseVH{PA9};
constexpr int kPhaseWH{PA10};
constexpr int kPhaseUL{PC13};
constexpr int kPhaseVL{PA12};
// constexpr int kPhaseWL{23};
constexpr int kPhaseWL{PB15};

constexpr int kPolePairs{7};
constexpr float kKvVal{180}; // 140KVより大きめ

BLDCMotor motor = BLDCMotor(kPolePairs, NOT_SET, kKvVal);
BLDCDriver6PWM driver = BLDCDriver6PWM(kPhaseUH, kPhaseUL, kPhaseVH, kPhaseVL, kPhaseWH, kPhaseWL);

HallSensor hole_sensor = HallSensor(kHole1, kHole2, kHole3, kPolePairs);
void DoA() { hole_sensor.handleA(); }
void DoB() { hole_sensor.handleB(); }
void DoC() { hole_sensor.handleC(); }

void setup()
{
  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  _delay(1000);

  hole_sensor.pullup = Pullup::USE_INTERN;
  hole_sensor.enableInterrupts(DoA, DoB, DoC);
  hole_sensor.init();
  motor.linkSensor(&hole_sensor);

  driver.voltage_power_supply = 24;
  driver.voltage_limit = 12;
  Serial.print("Driver init ");
  // init driver
  if (driver.init())
    Serial.println("success!");
  else
  {
    Serial.println("failed!");
    return;
  }
  motor.linkDriver(&driver);

  // motor.foc_modulation = FOCModulationType::Trapezoid_120;
  motor.controller = MotionControlType::velocity;
  // aligning voltage
  motor.voltage_sensor_align = 3;

  // motor.voltage_limit = 5;    // [V]
  motor.velocity_limit = 1024; // [rad/s]
  motor.sensor_direction = Direction::CW; // 回転方向

  motor.init();
  // init current sense
  if (motor.initFOC())
    Serial.println("FOC init success!");
  else
  {
    Serial.println("FOC init failed!");
    return;
  }

  Serial.println("ready");
  _delay(1000);
}

void loop()
{
  motor.loopFOC();
  motor.move(analogRead(kPotentiometer));
  Serial.print(hole_sensor.getAngle());
  Serial.println(" rad. ");
}


// --------------------------------------------------------------------------------------------------------------------


#endif
