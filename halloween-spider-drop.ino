#include <NewPing.h>
#include <PCA9685.h>
#include "pins.h"

#define releaseButtonPin PIN_D4

#define trigPin PIN_D8
#define echoPin PIN_D7
NewPing sonar(trigPin, echoPin);

PCA9685 pwmController(Wire);
PCA9685_ServoEval servoEval;

// configuation
const int triggerDistance = 10; // in centimeters
const int spoolAngleReleased = -60;
const int spoolAngleWindUp = 60;
const int tiltBackTime = 1000; // in milliseconds
const int windUpTime = 1000; // in milliseconds

bool canReleaseTheSpider = false;
bool isReleaseButtonPressed = false;
bool isTriggered = false;
int state = 0;
int measuredDistance = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Moin!");

  Wire.begin(PIN_D2, PIN_D1);
  pwmController.resetDevices();
  pwmController.init();
  pwmController.setPWMFreqServo();

  pinMode(releaseButtonPin, INPUT);

  release(1000);
  windUp();
}

void loop() {
  measuredDistance = sonar.ping_cm();
  isReleaseButtonPressed = !digitalRead(releaseButtonPin);
  isTriggered = isWithinTriggerDistance(measuredDistance);

  Serial.print("button state: ");
  Serial.print(isReleaseButtonPressed);
  Serial.print(" / distance: ");
  Serial.println(measuredDistance);

  if (canReleaseTheSpider && (isTriggered || isReleaseButtonPressed)) {
    Serial.println("release the kraken...ehm..the spider");
    release(5000);
    windUp();
    canReleaseTheSpider = false;
  }

  if (isReleaseButtonPressed || (measuredDistance > triggerDistance && !canReleaseTheSpider)) {
    canReleaseTheSpider = true;
  }

  delay(100);
}

bool isWithinTriggerDistance(unsigned long measuredDistance) {
  return measuredDistance <= triggerDistance && measuredDistance > 3;
}

void release(int delayToSetToWindUpPosition) {
  Serial.print("releasing...");
  pwmController.setChannelPWM(0, servoEval.pwmForAngle(spoolAngleReleased));
  Serial.print("spool moved down...");
  delay(tiltBackTime);
  pwmController.setChannelPWM(0, servoEval.pwmForAngle(spoolAngleWindUp));
  delay(delayToSetToWindUpPosition);
  Serial.println("moved up again");
}

void windUp() {
  Serial.print("winding up...");
  pwmController.setChannelPWM(1, servoEval.pwmForAngle(180));
  delay(windUpTime);
  pwmController.setChannelPWM(1, servoEval.pwmForAngle(90));
  Serial.println("done");
}