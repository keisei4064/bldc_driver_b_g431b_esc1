#ifdef POTENTIOMETER

#include <Arduino.h>

constexpr int kLedPin = LED_RED;
constexpr int kPotentiometer{PB12};

void setup() {
  // put your setup code here, to run once:
  pinMode(kLedPin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(kLedPin, HIGH);
  delay(100);
  digitalWrite(kLedPin, LOW);
  delay(300);

  Serial.println(analogRead(kPotentiometer)); // 0~1023
}

#endif
