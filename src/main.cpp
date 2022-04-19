#include <Arduino.h>

#define DEBUG false

//--------------------Serial----------------------//
#include <AsyncStream.h>
AsyncStream<255> serial(&Serial, '\n');
//-----------------End of Serial------------------//

//-------------------------LCD---------------------//
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#include <TimerMs.h>
// настройка (период, мс), (не запущен/запущен), (период/таймер)
TimerMs loading_tmr(3500, false, 0);
int count_of_dots = 0;
//---------------------End of LCD------------------//

//-------------------OS Setup---------------------//
#include <GyverOS.h>
GyverOS<3> OS;

#define LCD_TASK 0
#define MOVEMENT_TASK 1
#define SERIAL_MONITOR_TASK 2
//---------------End of OS Setup------------------//

//-----------------Sensors-------------------------//
#include <NewPing.h>
// Maximum distance we want to ping for (in centimeters).
// Maximum sensor distance is rated at 400-500cm.
#define MAX_DISTANCE 500

// Left Sensor Pins
#define TRIGGER_PIN_LEFT 8
#define ECHO_PIN_LEFT 9

// Right Sensor Pins
#define TRIGGER_PIN_RIGHT 12
#define ECHO_PIN_RIGHT 13

// Front Sensor Pins
#define TRIGGER_PIN_FRONT 11
#define ECHO_PIN_FRONT 10

// NewPing setup of pins and maximum distance.
NewPing sonarLeft(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);

#include "GyverFilters.h"

int leftSensor, rightSensor, frontSensor;

GMedian3<int> sonarLeftFilter, sonarRightFilter, sonarFrontFilter;

int *mainSensor = &leftSensor;

const int wall_threshold = 20;
const int front_threshold = 25;

bool frontwall, leftwall, rightwall;

#define LEFT_LINE_PIN 2  // Left Line Sendor pin
#define RIGHT_LINE_PIN 3 // Right Line Sensor pin

bool leftline, rightline;
//-----------------End 0f Sensors-------------------------//

//-------------------------Motor---------------------//
#include <GyverMotor.h>

#define DIR_LEFT 7
#define SPEED_LEFT 6
#define DIR_RIGHT 4
#define SPEED_RIGHT 5

#define SPEED 150
#define BRAKE_K 2

GMotor leftmotor(DRIVER2WIRE, DIR_LEFT, SPEED_LEFT, LOW);
GMotor rightmotor(DRIVER2WIRE, DIR_RIGHT, SPEED_RIGHT, LOW);

int leftmotorspeed, rightmotorspeed;

bool move_state = false; // Переменная отвечает за активный режим

bool move_direction = true;
int target = 1;
int locationt_id = 0;
//--------------------------End of Motor--------------//

//-------------------------PID Setup----------------------//
const float kP = 0.7;
const float kI = 0.4;
const float kD = 0.5;
const int offset = 25;
float oldErrorP, totalError;
float errorP, errorI, errorD;
//---------------------End of PID Setup------------------//

//-------------------------Motor---------------------//
void SetSpeed(int leftWheelSpeed, int rightWheelSpeed) // Установка скорости на оба мотора
{
  leftmotor.setSpeed(leftWheelSpeed);
  rightmotor.setSpeed(rightWheelSpeed);
}

void Rotation()
{
  SetSpeed(-SPEED, -SPEED);
  delay(1000);
  if (move_direction)
    SetSpeed(-SPEED, SPEED);
  else
    SetSpeed(SPEED, -SPEED);
  // while (!rotate_tmr.tick())  {} TO.DO
  delay(500);
}
//----------------------End of Motor-----------------//

//------------------------Utilities------------------//
String DebugData() // Упаковывает  данные для отправки по UART
{
  constexpr size_t size = 4;
  String dataString;
  int *data[size]{&leftSensor, &rightSensor, &frontSensor, &locationt_id};
  for (int i = 0; i < size; i++)
  {
    dataString += String(*data[i]) + ",";
  }
  return dataString;
}

void sendLocation()
{
  lcd.clear();
  lcd.print(locationt_id);
  Serial.println(locationt_id);
}

void LCD_dot_animation_controller()
{
  if (count_of_dots > 2)
  {
    lcd.setCursor(11, 1);
    lcd.print("   ");
    count_of_dots = 0;
    lcd.setCursor(11, 1);
  }
  else
  {
    lcd.print(".");
    count_of_dots++;
  }
}

void target_controller()
{
  bool lastmove_direction = move_direction;
  if (target == locationt_id)
  {
    move_state = false;
    Serial.println("Already on Target!");
  }
  else
  {
    move_direction = target > locationt_id;

    if (!(lastmove_direction == move_direction))
    {
      Rotation();
      mainSensor = move_direction ? &leftSensor : &rightSensor;
    }
    move_state = true;
    OS.start(MOVEMENT_TASK);
  }
}
//---------------------End of Utilities--------------//
//------------------------PID------------------------//
void PID(int mode) // Корректировка траектории на основе ПИД регулятора
{

  errorP = *mainSensor - offset;

  errorP = errorP > 255 ? 255 : errorP;
  errorI = 0.6 * errorI + errorP;
  errorD = errorP - oldErrorP;
  totalError = kP * errorP + kI * errorI + kD * errorD;
  oldErrorP = errorP;

  switch (mode)
  {
  case 0:
    leftmotorspeed = SPEED - totalError;
    rightmotorspeed = SPEED + totalError;
    break;
  case 1:
    leftmotorspeed = SPEED + totalError;
    rightmotorspeed = SPEED - totalError;
    break;
  case 2:
    leftmotorspeed = SPEED + totalError;
    rightmotorspeed = SPEED - totalError;
    break;
  case 3:
    leftmotorspeed = SPEED - totalError;
    rightmotorspeed = SPEED + totalError;
    break;
  case 4:
    leftmotorspeed = SPEED + totalError;
    rightmotorspeed = SPEED + totalError;
    break;
  }

  SetSpeed(leftmotorspeed, rightmotorspeed);
}

void resetPIDparams()
{
  oldErrorP = 0;
  errorI = 0;
}
//---------------------End of PID--------------------//

//--------------------------Sensors--------------------------//
// TODO sonar.convert_cm(sonar.ping_median(iterations));
int one_cycle_of_sensor(NewPing sonar, GMedian3<int> filter)
{
  int count_of_calls = 0;
  int Sensor;
  while (count_of_calls < 2)
  {
    int range = sonar.ping_cm();
    if (range == 0)
    {
      Sensor = MAX_DISTANCE;
    }
    else
    {
      Sensor = filter.filtered(range);
    }
    count_of_calls++;
  }

  return Sensor;
}

int one_cycle_of_sensor_for_mainSensor(NewPing sonar, GMedian3<int> filter, int Lastrange)
{
  int Sensor = sonar.convert_cm(sonar.ping_median(5));
  if (Sensor == 0)
  {
    Sensor = MAX_DISTANCE;
  }
  return Sensor;
}
//---------------------End 0f Sensors------------------------//

//--------------------------Controllers----------------------//
void start_controller()
{
  lcd.init();
  lcd.home();
  lcd.noBlink();
  lcd.backlight();
  lcd.print("Started");

  leftmotor.setMode(AUTO);
  rightmotor.setMode(AUTO);

  delay(1000);
}

void serial_controller()
{
  if (serial.available())
  {
    int cmd = String(serial.buf).toInt();

    if (cmd == 0)
    {
      move_state = false;
    }
    else if (cmd == 1)
    {
      move_state = true;
      OS.start(MOVEMENT_TASK);
    }
    else
    {
      target = cmd - 2;
      target_controller();
    }
  }
  if (DEBUG)
    Serial.println(DebugData());
}

void lcd_controller()
{
  loading_tmr.tick();
  if (!move_state && !loading_tmr.active())
  {
    lcd.clear();
    lcd.home();
    lcd.print("Waiting");
    lcd.setCursor(0, 1);
    lcd.print("for command");
    loading_tmr.resume();
  }
  if (move_state)
  {
    lcd.clear();
    lcd.home();
    if (true)
    {
      lcd.print(String(leftSensor) + "|" + String(rightSensor) + "|" + String(frontSensor));
      lcd.setCursor(0, 1);
      lcd.print(String(totalError) + "|" + String(leftmotorspeed) + "|" + String(rightmotorspeed));
    }
    if (!(loading_tmr.active()))
    {
      loading_tmr.stop();
      lcd.print("Moving to Target");
    }
  }
}

void sensor_controller()
{
  leftline = digitalRead(LEFT_LINE_PIN);
  rightline = digitalRead(RIGHT_LINE_PIN);

  leftSensor = one_cycle_of_sensor_for_mainSensor(sonarLeft, sonarLeftFilter, leftSensor);
  rightSensor = one_cycle_of_sensor_for_mainSensor(sonarRight, sonarRightFilter, rightSensor);

  frontSensor = one_cycle_of_sensor(sonarFront, sonarFrontFilter);

  if (mainSensor == &leftSensor)
  {
    leftwall = leftSensor >= wall_threshold && leftSensor <= wall_threshold + 10;
    rightwall = rightSensor <= wall_threshold;
  }
  else
  {
    leftwall = leftSensor <= wall_threshold;
    rightwall = rightSensor >= wall_threshold && rightSensor <= wall_threshold + 10;
  }

  frontwall = frontSensor <= wall_threshold;
}

void movement_controller()
{
  if (leftline || rightline) // Остановка при достижении чекпоинта
  {
    locationt_id += move_direction ? 1 : -1;
    sendLocation();
    if (locationt_id == target)
    {
      Serial.println("Reach Target! I'm Waiting...");
      move_state = false;
    }
    SetSpeed(SPEED, SPEED);
    delay(300);
    SetSpeed(1, 1);
  }
  else
  {
    if (frontwall) // Объезд препятствия впереди
    {
      if (move_direction == 1)
      {
        SetSpeed(SPEED, -SPEED);
      }
      else
      {
        SetSpeed(-SPEED, SPEED);
      }
      delay(50);
    }
    else
    {
      if (*mainSensor >= 20 && *mainSensor <= 30)
      {
        PID(0);
      }
      else
      {
        resetPIDparams();
        if (*mainSensor < 20)
        {
          if (move_direction == 1)
          {
            SetSpeed(SPEED, SPEED / BRAKE_K);
          }
          else
          {
            SetSpeed(SPEED / BRAKE_K, SPEED);
          }
        }
        else
        {
          if (move_direction == 1)
          {
            SetSpeed(SPEED / BRAKE_K, SPEED);
          }
          else
          {
            SetSpeed(SPEED, SPEED / BRAKE_K);
          }
        }
      }
    }
  }
}

void sensor_and_movement_controller()
{
  sensor_controller();
  if (move_state)
    movement_controller();
  else
  {
    SetSpeed(1, 1);
    OS.stop(MOVEMENT_TASK);
  }
}
//---------------------End 0f Controllers--------------------//

void setup() // Инициализациия
{
  start_controller();

  // Запуск слушателя UART
  Serial.begin(115200);

  // Присвоение функций обратного вызова
  OS.attach(LCD_TASK, lcd_controller, 1000);
  OS.attach(MOVEMENT_TASK, sensor_and_movement_controller, 50);
  OS.attach(SERIAL_MONITOR_TASK, serial_controller, 500);
  loading_tmr.attach(LCD_dot_animation_controller);
}

void loop()
{
  OS.tick();
}
