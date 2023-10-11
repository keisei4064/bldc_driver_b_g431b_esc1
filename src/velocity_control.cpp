// 参考：https://docs.simplefoc.com/velocity_loop
#ifdef VELOCITY_CONTROL

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

int count = 0;

float GetTemperture()
{
  // 参照：https: // community.simplefoc.com/t/temperature-monitoring-on-b-g431b-esc1/2955/7
  // float adc_voltage = _readADCVoltageInline(A_TEMPERATURE, current_sense.params);
  float adc_voltage = analogRead(A_TEMPERATURE) * 3.3 / 1023;

  // Formula: https://www.giangrandi.org/electronics/ntc/ntc.shtml
  const float ResistorBalance = 4700.0;
  const float Beta = 3425.0F;
  const float RoomTempI = 1.0F / 298.15F; //[K]
  const float Rt = ResistorBalance * ((3.3F / adc_voltage) - 1);
  const float R25 = 10000.0F;

  float temperature = 1.0F / ((log(Rt / R25) / Beta) + RoomTempI);
  temperature = temperature - 273.15;

  return temperature;
}

void setup()
{
  pinMode(A_TEMPERATURE, INPUT_ANALOG);
  Serial.begin(115200);
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_TARGET | _MON_ANGLE | _MON_VEL | _MON_VOLT_D | _MON_VOLT_Q | _MON_CURR_D | _MON_CURR_Q;
  motor.monitor_downsample = 1000;
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
  motor.controller = MotionControlType::velocity;
  motor.voltage_limit = 12;   // [V]
  motor.velocity_limit = 500; // [rad/s]
  // motor.sensor_direction = Direction::CW; // 回転方向
  motor.target = 20.0;
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 0.01;
  motor.PID_velocity.D = 0.0;
  // motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.15;
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
  motor.move(analogRead(kPotentiometer) * 0.2);
  // motor.move();
  // motor.monitor();

  if (count++ % 1000 == 0)
    Serial.println(GetTemperture());
}

#endif
