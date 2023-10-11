#ifdef HOLE_SENSOR

#include <Arduino.h>
#include <SimpleFOC.h>

constexpr int kLedPin = LED_RED;
constexpr int kPotentiometer{PB12};
constexpr int kHole1{PB6};
constexpr int kHole2{PB7};
constexpr int kHole3{PB8};

constexpr int kPolePairs{7};

HallSensor hole_sensor{kHole1, kHole2, kHole3, kPolePairs};
void DoA() { hole_sensor.handleA(); }
void DoB() { hole_sensor.handleB(); }
void DoC() { hole_sensor.handleC(); }

void setup()
{
  // put your setup code here, to run once:
  pinMode(kLedPin, OUTPUT);
  Serial.begin(115200);

  hole_sensor.pullup = Pullup::USE_INTERN;
  hole_sensor.enableInterrupts(DoA, DoB, DoC);

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop()
{
  hole_sensor.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(hole_sensor.getAngle());
  Serial.print(" rad. ");
  Serial.print("\t");
  Serial.print(hole_sensor.getVelocity());
  Serial.println(" rad/s.");
}

#endif
