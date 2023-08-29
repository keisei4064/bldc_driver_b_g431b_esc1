#ifdef TEST_INCREMENTAL_ENCODER

#include <Arduino.h>
#include <SimpleFOC.h>

constexpr int kLedPin = LED_RED;
constexpr int kAPhase{PB6};
constexpr int kBPhase{PB7};
// constexpr int kIndexPin{PB8};

Encoder encoder{kAPhase, kBPhase, 1024};
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

void setup()
{
  // put your setup code here, to run once:
  // pinMode(kLedPin, OUTPUT);

  Serial.begin(115200);

  // enable/disable quadrature mode
  encoder.quadrature = Quadrature::OFF;

  // check if you need internal pullups
  encoder.pullup = Pullup::USE_EXTERN;

  // initialize encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);

  Serial.println("Encoder ready");
  _delay(1000);
}

void loop()
{
  // IMPORTANT - call as frequently as possible
  // update the sensor values
  encoder.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(encoder.getAngle());
  Serial.print("\t");
  Serial.println(encoder.getVelocity());
}

#endif
