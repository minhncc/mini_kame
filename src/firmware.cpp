#include <Arduino.h>
#include <setjmp.h>
#include <MiniKame.h>

//uncomment out below line if you want to use HC_SR04 sensor
//#define __HC_SR04__

#ifdef __HC_SR04__
#define HC_SR04_TRIGGER_PIN A4
#define HC_SR04_ECHO_PIN A5
#define MIN_DISTANCE 10
#define MAX_DISTANCE MIN_DISTANCE + 10
#endif

#define CAL_TRIGGER_PIN 12
#define LED_PIN A6

#define TIME_INTERVAL 5000
#define SERIAL_DATA_PERIOD 200

#define FORWARD 'f'
#define LEFT 'l'
#define STAND 's'
#define RIGHT 'r'
#define BACKWARD 'b'
#define PUSH_UP 'p'
#define UP_DOWN 'u'
#define DANCE 'n'
#define OMNI_WALK_R 'o'
#define OMNI_WALK_L 'i'
#define MOON_WALK 'm'
#define FRONT_BACK 't'
#define HELLO 'h'
#define WALK_FORWARD '1'
#define WALK_BACKWARD '2'

MiniKame robot;

bool auto_mode = false;
bool random_walk = false;
bool stopSerial = false;
unsigned long cur_time, prev_serial_data_time, perv_sensor_time;
char cmd = STAND;
char bluetoothCmd = STAND;

jmp_buf jump_env;

void setup()
{
  Serial.begin(9600); // For bluetooth HC-05

#ifdef __HC_SR04__
  pinMode(HC_SR04_TRIGGER_PIN, OUTPUT);
  pinMode(HC_SR04_ECHO_PIN, INPUT);
#endif
  randomSeed(analogRead(A7));
  //
  robot.init();
  delay(2000);

  { //begin: triggering delay for servo calibrating
    bool state = true;
    pinMode(CAL_TRIGGER_PIN, OUTPUT);
    digitalWrite(CAL_TRIGGER_PIN, 0);
    pinMode(CAL_TRIGGER_PIN, INPUT);
    while (digitalRead(CAL_TRIGGER_PIN))
    {
      analogWrite(LED_PIN, 128 * state); // on calibarting indication LED
      delay(1000);
      state = !state;
    }
    analogWrite(LED_PIN, 0); // off calibarting indication LED
  }
  //end:

  perv_sensor_time = prev_serial_data_time = millis();
  //robot.run();
  //robot.turnL();
  //robot.turnR();
  //robot.pushUp();
  //robot.upDown();
  //robot.dance();
  //robot.frontBack();
  //robot.moonwalkL();
  //robot.omniWalk();
  //robot.hello();
  //robot.jump();
  if (auto_mode)
    cmd = FORWARD;
}

boolean gaits(char cmd)
{
  static char prev_cmd = '.';
  bool taken = true;

  if (prev_cmd == cmd)
    return taken;

  robot.init();
  switch (cmd)
  {
  case FORWARD:
    robot.run();
    break;
  case BACKWARD:
    robot.run(0);
    break;
  case WALK_FORWARD:
    robot.walk();
    break;
  case WALK_BACKWARD:
    robot.walk(0);
    break;
  case RIGHT:
    robot.turnR(1, 550);
    break;
  case LEFT:
    robot.turnL(1, 550);
    break;
  case STAND:
    robot.home();
    break;
  case PUSH_UP:
    robot.pushUp();
    break;
  case UP_DOWN:
    robot.upDown();
    break;
  case HELLO:
    robot.hello();
    break;
  case MOON_WALK:
    robot.moonwalkL();
    break;
  case OMNI_WALK_L:
    robot.omniWalk(false);
    break;
  case OMNI_WALK_R:
    robot.omniWalk();
    break;
  case DANCE:
    robot.dance();
    break;
  case FRONT_BACK:
    robot.frontBack();
    break;
  default:
    taken = false;
  }
  if (taken)
    prev_cmd = cmd;
  return taken;
}

boolean checkBluetoothCommand()
{
  bool rc = false;
  if (Serial.available() > 0)
  {
    bluetoothCmd = Serial.read();

    rc = true;

    if (!auto_mode) {
      cmd = bluetoothCmd;
    }
  }

  if (auto_mode)
  {
    static char movements[] = {FORWARD, LEFT, OMNI_WALK_R, OMNI_WALK_L, RIGHT, BACKWARD, PUSH_UP, UP_DOWN, HELLO, DANCE, FRONT_BACK, MOON_WALK};
    static unsigned long old_time = cur_time;
    static int c = 0;
    if (cur_time - old_time >= TIME_INTERVAL)
    {
      old_time = cur_time;
      //c = (int)random(0, sizeof(movements));
      // cmd = movements[c];
      if (!random_walk)
      {
        c = c % (sizeof(movements) / sizeof(char));
        cmd = movements[c++];
      }
      else
      {
        c = (int)random(0, sizeof(movements) / sizeof(char));
        cmd = movements[c];
      }
    }
  }
  return rc;
}

void pause(int period) {
  unsigned long timeout = millis() + period;
  do
  {
    if (checkBluetoothCommand())
      longjmp(jump_env, 1);
  } while (millis() <= timeout);
}

#ifdef __HC_SR04__
char detect_obstacle(char cmd)
{
  char i = cmd;
  long cm = distance_measure();
  if (cm < MIN_DISTANCE)
  {
    i = STAND;
  }
  else if (cm >= MIN_DISTANCE && cm <= MAX_DISTANCE)
  {
    i = BACKWARD;
  }
  return i;
}

long distance_measure()
{
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:

  digitalWrite(HC_SR04_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(HC_SR04_TRIGGER_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(HC_SR04_TRIGGER_PIN, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(HC_SR04_ECHO_PIN, HIGH);

  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  cm = duration / 29 / 2;
  //softSerial.print(cm);
  //Serial.println("cm");
  // delay(100);
  return cm;
}
#endif

void loop() {

  cur_time = millis();
  if (cur_time - prev_serial_data_time >= 100) {
    prev_serial_data_time = cur_time;
  }
  checkBluetoothCommand();
  setjmp(jump_env);

#ifdef __HC_SR04__
  if (cur_time - perv_sensor_time >= SERIAL_DATA_PERIOD)
  {
    perv_sensor_time = cur_time;
    long cm = distance_measure();
    if (cm >= MIN_DISTANCE && cm <= MAX_DISTANCE)
    {
      cmd = BACKWARD;
    }
  }
#endif
  gaits(cmd);
  robot.refresh();
}
