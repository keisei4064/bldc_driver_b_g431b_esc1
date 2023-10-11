// 参照：https://docs.simplefoc.com/dc_current_torque_mode
#ifdef TORQUE_CONTROL_USING_DC_CURRENT

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
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

Encoder encoder{kAPhase, kBPhase, kEncPulse};
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

int count = 0;

float GetPotentiometerValue()
{
  return _readADCVoltageInline(kPotentiometer, current_sense.params);
}

float GetTemperture()
{
  // 参照：https: // community.simplefoc.com/t/temperature-monitoring-on-b-g431b-esc1/2955/7
  float adc_voltage = _readADCVoltageInline(A_TEMPERATURE, current_sense.params);

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

  // 電流センサー ---------------------------------------------------
  current_sense.linkDriver(&driver);
  if (current_sense.init())
    Serial.println("Current sense init success!");
  else
  {
    Serial.println("Current sense init failed!");
    return;
  }
  motor.linkCurrentSense(&current_sense);

  // motor.current_limit = 0.5;

  // // PID parameters - default
  // motor.PID_current_q.P = 5;    // 3    - Arduino UNO/MEGA
  // motor.PID_current_q.I = 1000; // 300  - Arduino UNO/MEGA
  // motor.PID_current_q.D = 0;
  // motor.PID_current_q.limit = motor.voltage_limit;
  // motor.PID_current_q.output_ramp = 1e6; // 1000 - Arduino UNO/MEGA
  // // Low pass filtering - default
  // motor.LPF_current_q.Tf = 0.005; // 0.01 - Arduino UNO/MEGA

  // モーター設定 ---------------------------------------------------
  motor.torque_controller = TorqueControlType::dc_current;
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
  motor.move(GetPotentiometerValue() * 0.5);
  // motor.monitor();

  // 温度を出力
  if (++count % 1000 == 0)
  {
    Serial.print("Temperature: ");
    Serial.print(GetTemperture());
    Serial.println(" [C]");
  }
}

#endif
