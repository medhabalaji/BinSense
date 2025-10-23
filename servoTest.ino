#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// IR sensor pins
#define IR_25_PIN 32
#define IR_50_PIN 33
#define IR_75_PIN 25
#define IR_100_PIN 26

// Ultrasonic sensor pins
#define TRIG_PIN 27
#define ECHO_PIN 14  // Connected via voltage divider

// PCA9685 setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Default I2C address 0x40
#define SERVO_CHANNEL 0  // Channel 0 on PCA9685
#define SERVO_MIN 150    // Closed position
#define SERVO_MAX 600    // Open position

// Hand detection threshold (in cm)
#define HAND_TRIGGER_DISTANCE 20

bool lidOpen = false;

void setup() {
  Serial.begin(115200);

  // IR sensors
  pinMode(IR_25_PIN, INPUT);
  pinMode(IR_50_PIN, INPUT);
  pinMode(IR_75_PIN, INPUT);
  pinMode(IR_100_PIN, INPUT);

  // Ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // PCA9685 setup
  pwm.begin();
  pwm.setPWMFreq(50); // Standard servo frequency
  delay(10);

  // Initialize lid position
  pwm.setPWM(SERVO_CHANNEL, 0, SERVO_MIN); // Lid closed
}

int getBinLevelFromIR() {
  if (digitalRead(IR_100_PIN) == LOW) return 100;
  else if (digitalRead(IR_75_PIN) == LOW) return 75;
  else if (digitalRead(IR_50_PIN) == LOW) return 50;
  else if (digitalRead(IR_25_PIN) == LOW) return 25;
  else return 0;
}

long getDistanceFromUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2; // cm
  return distance;
}

void openLid() {
  Serial.println("Opening lid...");
  pwm.setPWM(SERVO_CHANNEL, 0, SERVO_MAX); // Open
  delay(3000);
  Serial.println("Closing lid...");
  pwm.setPWM(SERVO_CHANNEL, 0, SERVO_MIN); // Close
}

void loop() {
  int binLevel = getBinLevelFromIR();
  Serial.print("Bin fill level: ");
  Serial.print(binLevel);
  Serial.println("%");

  long handDistance = getDistanceFromUltrasonic();
  Serial.print("Hand distance: ");
  Serial.print(handDistance);
  Serial.println(" cm");

  if (handDistance > 0 && handDistance < HAND_TRIGGER_DISTANCE && !lidOpen) {
    openLid();
    lidOpen = true;
  } else if (handDistance >= HAND_TRIGGER_DISTANCE) {
    lidOpen = false;
  }

  Serial.println("----------------------");
  delay(1000);
}