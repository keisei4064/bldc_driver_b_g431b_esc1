#ifdef SIMPLE_FOC
#include <SimpleFOC.h>

constexpr int kPotentiometer{PB12};

constexpr int kPhaseUH{PA8};
constexpr int kPhaseVH{PA9};
constexpr int kPhaseWH{PA10};
constexpr int kPhaseUL{PC13};
constexpr int kPhaseVL{PA12};
// constexpr int kPhaseWL{23};
constexpr int kPhaseWL{PB15};

// NUMBER OF POLE PAIRS, NOT POLES
BLDCMotor motor = BLDCMotor(7);
// MUST USE 6PWM FOR B-G431 DRIVER
BLDCDriver6PWM driver = BLDCDriver6PWM(kPhaseUH, kPhaseUL, kPhaseVH, kPhaseVL, kPhaseWH, kPhaseWL);

void setup()
{
    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = 24;
    driver.init();

    // link the motor and the driver
    motor.linkDriver(&driver);

    // limiting motor movements
    motor.voltage_limit = 3;     // [V]
    motor.velocity_limit = 1024; // [rad/s]

    // open loop control config
    motor.controller = MotionControlType::velocity_openloop;

    // init motor hardware
    motor.init();

    // Serial.begin(9600);
}

void loop()
{
    motor.move(analogRead(kPotentiometer));
    // Serial.println(analogRead(kPotentiometer));
}

#endif
