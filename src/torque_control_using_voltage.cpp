// 参照：https://docs.simplefoc.com/voltage_torque_mode
#ifdef TORQUE_CONTROL_USING_VOLTAGE

#include <Arduino.h>
#include <SimpleFOC.h>

constexpr int kPotentiometer{PB12};
constexpr int kAPhase{PB6};
constexpr int kBPhase{PB7};

constexpr int kPhaseUH{PA8};
constexpr int kPhaseVH{PA9};
constexpr int kPhaseWH{PA10};
constexpr int kPhaseUL{PC13};
constexpr int kPhaseVL{PA12};
constexpr int kPhaseWL{PB15};

constexpr int kPolePairs{7};
constexpr float kKvVal{1000};
constexpr int kEncPulse{1000};

BLDCMotor motor = BLDCMotor(kPolePairs, NOT_SET, kKvVal);
BLDCDriver6PWM driver = BLDCDriver6PWM(kPhaseUH, kPhaseUL, kPhaseVH, kPhaseVL, kPhaseWH, kPhaseWL);

Encoder encoder{kAPhase, kBPhase, kEncPulse};
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

void setup()
{
  Serial.begin(115200);
  motor.useMonitoring(Serial);
  _delay(1000);

  // エンコーダー設定 ---------------------------------------------------
  encoder.quadrature = Quadrature::OFF;
  encoder.pullup = Pullup::USE_INTERN;
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  motor.linkSensor(&encoder);

  // ドライバー設定 ---------------------------------------------------
  driver.voltage_power_supply = 24;
  driver.voltage_limit = 12;
  Serial.print("Driver init ");
  if (driver.init())
    Serial.println("success!");
  else
  {
    Serial.println("failed!");
    return;
  }
  motor.linkDriver(&driver);

  // モーター設定 ---------------------------------------------------
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = 24;   // [V]
  motor.velocity_limit = 500; // [rad/s]
  // motor.sensor_direction = Direction::CW; // 回転方向
  motor.init();

  // -----------------------------------------------------------------

  if (motor.initFOC())
    Serial.println("FOC init success!");
  else
  {
    Serial.println("FOC init failed!");
    return;
  }
  _delay(1000);
}

void loop()
{
  motor.loopFOC();
  motor.move(analogRead(kPotentiometer) * 0.002);

  // Serial.print(encoder.getAngle());
  // Serial.print("\t");
  // Serial.println(encoder.getVelocity());
}

#endif
