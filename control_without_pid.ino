#include <AFMotor.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int BUTTONT = A0;
const int BUTTON1 = A1;
const int BUTTON2 = A2;
const int BUTTON3 = A3;

uint16_t checkPoint3Floor = 58;
uint16_t checkPoint2Floor = 130;
uint16_t checkPoint1Floor = 250;
uint16_t checkPointTFloor = 300;
uint16_t startTime;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
AF_DCMotor motor(3);

void setup()
{
  // setup buttons
  pinMode(BUTTONT, INPUT_PULLUP);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  // setup serial monitor
  Serial.begin(115200);
  // setup lcd
  lcd.setBacklight(HIGH);
  lcd.begin(16, 2);
  // setup sensor
  lox.begin();
}

uint16_t getMeasure()
{
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4)
  {
    return measure.RangeMilliMeter;
  }
  else
  {
    return 0;
  }
}

void displayFloor(int floor)
{
  lcd.setCursor(0, 1);
  lcd.print("Floor: ");
  lcd.print(floor);
}

void displayMeasure(uint16_t measure)
{
  lcd.setCursor(0, 0);
  lcd.print("Distance: ");
  lcd.print(measure);
}

int checkFloor(uint16_t measure)
{
  if (measure >= checkPointTFloor)
  {
    return 0;
  }
  else if (measure >= checkPoint1Floor)
  {
    return 1;
  }
  else if (measure >= checkPoint2Floor)
  {
    return 2;
  }
  else if (measure >= checkPoint3Floor)
  {
    return 3;
  }
  else
  {
    return -1;
  }
}

uint16_t checkInputs()
{
  if (digitalRead(BUTTONT) == 0)
  {
    Serial.print("Button T pushed, actual floor: ");
    Serial.println(checkFloor(getMeasure()));
    return checkPointTFloor;
  }
  if (digitalRead(BUTTON1) == 0)
  {
    Serial.print("Button 1 pushed, actual floor: ");
    Serial.println(checkFloor(getMeasure()));
    return checkPoint1Floor;
  }
  if (digitalRead(BUTTON2) == 0)
  {
    Serial.print("Button 2 pushed, actual floor: ");
    Serial.println(checkFloor(getMeasure()));
    return checkPoint2Floor;
  }
  if (digitalRead(BUTTON3) == 0)
  {
    Serial.print("Button 3 pushed, actual floor: ");
    Serial.println(checkFloor(getMeasure()));
    return checkPoint3Floor;
  }
  return 0.0;
}

void analitics()
{
  Serial.print("Distance: ");
  Serial.print(getMeasure());
  Serial.print(" Time: ");
  Serial.println((millis() - startTime) / 1000.0);
}

void moveDown()
{
  motor.setSpeed(255); // Define a velocidade maxima
  motor.run(FORWARD);
}

void moveUp()
{
  motor.setSpeed(255);
  motor.run(BACKWARD);
}

void stoped()
{
  motor.setSpeed(0);
  motor.run(RELEASE);
}

void loop()
{

  uint16_t measure = getMeasure();

  lcd.clear();
  displayMeasure(measure);
  displayFloor(checkFloor(measure));
  analitics();
  int input = checkInputs();

  if (input == 0)
  {
    stoped();
  }
  else
  {
    // startTime = millis();
    // while (abs(input - getMeasure()) >= 1)
    // {
    //   int diff = input - getMeasure();
    //   Serial.println();
    //   Serial.print(diff);
    //   if (diff < 0)
    //   {
    //     moveUp();
    //   }
    //   else
    //   {
    //     moveDown();
    //   }
    // }
    if (input > getMeasure())
    {
      while (input > getMeasure())
      {
        analitics();
        moveDown();
      }
      return;
    }
    if (input < getMeasure())
    {
      while (input < getMeasure())
      {
        analitics();
        moveUp();
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
