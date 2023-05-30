#include <AFMotor.h>
#include <PID_v1.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int BUTTONT = A0;
const int BUTTON1 = A1;
const int BUTTON2 = A2;
const int BUTTON3 = A3;

uint16_t checkPoint3Floor = 58;
uint16_t checkPoint2Floor = 140;
uint16_t checkPoint1Floor = 247;
uint16_t checkPointTFloor = 310;

uint16_t startTime;
uint16_t currentTime;
uint16_t period = 8500;  //8.5 seconds

double setPointPID, entryPointPID, outputPID;
// Olhometro
float Kp = 3;
float Ki = 5;
float Kd = 2;

// Lugar das raizes
// float Kp = 142.8216694;
// float Ki = 14.28221669;
// float Kd = 14.222169;

// ZN metodo 2
// float Kp = 8569.300128;
// float Ki = 92550.62174;
// float Kd = 1983.587557;

// ZN metodo 2 dividido por kp
// float Kp = 1;
// float Ki = 10.800;
// float Kd = 0.2314760;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
AF_DCMotor motor(3);
PID myPID(&entryPointPID, &outputPID, &setPointPID, Kp, Ki, Kd, DIRECT);

void setup() {
  // setup buttons
  pinMode(BUTTONT, INPUT_PULLUP);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  // setup serial monitor
  Serial.begin(2400);
  // setup lcd
  lcd.setBacklight(LOW);
  lcd.begin(16, 2);
  // setup sensor
  lox.begin();

  // pid setup
  myPID.SetMode(AUTOMATIC);

  //start time
  startTime = millis();
}

uint16_t getMeasure() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  } else {
    return 0;
  }
}

void displayFloor(int floor) {
  lcd.setCursor(0, 1);
  lcd.print("Floor: ");
  lcd.print(floor);
}

void displayMeasure(uint16_t measure) {
  lcd.setCursor(0, 0);
  lcd.print("Distance: ");
  lcd.print(measure);
}

int checkFloor(uint16_t measure) {
  if (measure >= checkPointTFloor) {
    return 0;
  } else if (measure >= checkPoint1Floor) {
    return 1;
  } else if (measure >= checkPoint2Floor) {
    return 2;
  } else if (measure >= checkPoint3Floor) {
    return 3;
  } else {
    return -1;
  }
}

uint16_t checkInputs() {
  if (digitalRead(BUTTONT) == 0) {
    Serial.print("Button T pushed, actual floor: ");
    Serial.println(checkFloor(getMeasure()));
    return checkPointTFloor;
  }
  if (digitalRead(BUTTON1) == 0) {
    Serial.print("Button 1 pushed, actual floor: ");
    Serial.println(checkFloor(getMeasure()));
    return checkPoint1Floor;
  }
  if (digitalRead(BUTTON2) == 0) {
    Serial.print("Button 2 pushed, actual floor: ");
    Serial.println(checkFloor(getMeasure()));
    return checkPoint2Floor;
  }
  if (digitalRead(BUTTON3) == 0) {
    Serial.print("Button 3 pushed, actual floor: ");
    Serial.println(checkFloor(getMeasure()));
    return checkPoint3Floor;
  }
  return 0.0;
}

void analitics() {
  Serial.print("Distance:");
  Serial.print(getMeasure());
  Serial.print(",");
  Serial.print("Time:");
  Serial.print((millis() - startTime) / 1000.0);
  Serial.print(",");
  Serial.print("Output:");
  Serial.println(outputPID);
}

void moveDown(int speed) {
  motor.setSpeed(speed);  // Define a velocidade maxima
  motor.run(FORWARD);
}

void moveUp(int speed) {
  motor.setSpeed(speed);
  motor.run(BACKWARD);
}

void stoped() {
  motor.setSpeed(0);
  motor.run(RELEASE);
}

void calculatePID(int input) {
  entryPointPID = getMeasure();
  setPointPID = input;
  myPID.Compute();
  // Serial.print("Input: ");
  // Serial.print(entryPointPID);
  // Serial.print(" setPoint: ");
  // Serial.print(setPointPID);
  // Serial.print(" output: ");
  // Serial.println(outputPID);
}

void loop() {

  uint16_t measure = getMeasure();

  lcd.clear();
  displayMeasure(measure);
  displayFloor(checkFloor(measure));
  // analitics();
  int input = checkInputs();

  if (input == 0) {
    stoped();
  } else {
    // startTime = millis();
    // while (abs(input - getMeasure()) >= 1)
    // {
    //     int diff = input - getMeasure();

    //     if (diff < 0)
    //     {
    //         analitics();
    //         calculatePID(input);
    //         moveUp(255-outputPID);
    //     }
    //     else
    //     {
    //         analitics();
    //         calculatePID(input);
    //         moveDown(outputPID);
    //     }
    // }

    startTime = millis();
    if (input > getMeasure()) {
      while (input > getMeasure()) {
        analitics();
        calculatePID(input);
        moveDown(outputPID);
      }
      while (millis() - startTime <= period) {
        stoped();
        analitics();
      }
      return;
    }
    if (input < getMeasure()) {
      while (input < getMeasure()) {
        analitics();
        calculatePID(input);
        moveUp(255 - outputPID);
      }
      while (millis() - startTime <= period) {
        stoped();
        analitics();
      }
      return;
    }
  }



  // if (digitalRead(BUTTONT) == 0) {
  //   Serial.println("Botao T pressionado: ");
  //   moveUp();
  //   return;
  // }
  // if (digitalRead(BUTTON1) == 0) {
  //   Serial.println("Botao 1 pressionado: ");
  //   moveDown();
  //   return;
  // }
  // stoped();

  // if (digitalRead(BUTTONT) == 0) {
  //   moveDown();
  //   delay(1000);
  // }
  // delay(100);
}
