#include <Arduino.h>
#include <setjmp.h>
#include <MiniKame.h>

//uncomment out below line if you want to use HC_SR04 sensor
//#define __HC_SR04__
// #define __DEBUG__

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

// Basic moves
#define STAND 1
#define FORWARD 2
#define BACKWARD 3
#define LEFT 4
#define RIGHT 5

// Intermediate move
#define REST 6
#define WALK_FORWARD 7
#define WALK_BACKWARD 8
#define PUSH_UP 9
#define UP_DOWN 10
#define DANCE 11
#define MOON_WALK 12
#define HELLO 13
#define LEG_TEST 14
#define LEVELING 15
#define FRONT_BACK 16
#define OMNI_WALK_R 17
#define OMNI_WALK_L 18

#define COMMAND_MAX_SIZE 32
#define COMMAND_MAX_ARG 3

MiniKame robot;

bool auto_mode = false;
bool random_walk = false;
bool stopSerial = false;
unsigned long cur_time, prev_serial_data_time, perv_sensor_time;
int cmd[COMMAND_MAX_ARG] = {STAND, 0, 0};
char bluetoothRead;
String bluetoothCmd = "";

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
    // robot.loadTrim();
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
  {
    memset(cmd, 0, sizeof(cmd));
    cmd[0] = STAND;
  }
}

boolean compareCmd(int *cmd1, int *cmd2)
{
  for (int i = 0; i < COMMAND_MAX_ARG; i++)
  {
    if (cmd1[i] != cmd2[i])
    {
      return false;
    }
  }

  return true;
}

void setCmd(int *from_cmd, int *to_cmd)
{
  for (int i = 0; i < COMMAND_MAX_ARG; i++)
  {
    to_cmd[i] = from_cmd[i];
  }
}

boolean gaits(int *cmd)
{
  static int prev_cmd[] = {0, 0, 0};
  bool taken = true;

  if (compareCmd(prev_cmd, cmd))
    return taken;

#ifdef __DEBUG__
  Serial.print("cmd:");
  Serial.print(cmd[0]);
  Serial.print(',');
  Serial.print(cmd[1]);
  Serial.print(',');
  Serial.println(cmd[2]);
  Serial.print("prev_cmd:");
  Serial.println(prev_cmd[0]);
  Serial.println("Executing...");
#endif
  // robot.init();

  switch (cmd[0])
  {
  case STAND:
    robot.home();
    break;
  case FORWARD:
    robot.run();
    break;
  case BACKWARD:
    robot.run(0);
    break;
  case LEFT:
    robot.turnL(1, 550);
    break;
  case RIGHT:
    robot.turnR(1, 550);
    break;
  case REST:
    robot.rest();
    break;
  case WALK_FORWARD:
    robot.walk();
    break;
  case WALK_BACKWARD:
    robot.walk(0);
    break;
  case PUSH_UP:
    robot.pushUp();
    break;
  case UP_DOWN:
    robot.upDown();
    break;
  case DANCE:
    robot.dance();
    break;
  case MOON_WALK:
    robot.moonwalkL();
    break;
  case HELLO:
    robot.hello();
    break;
  case LEG_TEST:
    robot.setServo(cmd[1], 20);
    delay(800);
    robot.setServo(cmd[1], 90);
    delay(800);
    robot.setServo(cmd[1], 160);
    break;
  case LEVELING:
    robot.leveling(cmd[1]);
    break;
  case OMNI_WALK_R:
    robot.omniWalk();
    break;
  case OMNI_WALK_L:
    robot.omniWalk(false);
    break;
  case FRONT_BACK:
    robot.frontBack();
    break;
  default:
    cmd[0] = STAND;
    cmd[1] = 0;
    cmd[2] = 0;
    taken = false;
    break;
  }

  if (taken)
    setCmd(prev_cmd, cmd);

  return taken;
}

boolean checkBluetoothCommand()
{
  bool rc = false;
  while (Serial.available() > 0)
  {
    bluetoothRead = Serial.read();
    // bluetoothCmd = bluetoothCmd + bluetoothRead; // Concat read to command
    bluetoothCmd.concat(bluetoothRead); // Concat read to command
#ifdef __DEBUG__
    Serial.print("bluetoothRead:");
    Serial.println(bluetoothRead);
    Serial.print("bluetoothCmd:");
    Serial.println(bluetoothCmd);
#endif
    rc = true;

    // if (!auto_mode && bluetoothRead == '*')
    if (bluetoothRead == '*')
    {
      bluetoothCmd = bluetoothCmd.substring(0, bluetoothCmd.indexOf('*')); // Delete last char *
      bluetoothCmd.trim();
      if (bluetoothCmd.length() > 0)
      {
        char bluetoothCmdChars[COMMAND_MAX_SIZE];
        bluetoothCmd.toCharArray(bluetoothCmdChars, COMMAND_MAX_SIZE);
        if (strchr(bluetoothCmdChars, ',') != NULL)
        {
          int argIdx = 0;
          char *chuck;
          char *p = bluetoothCmdChars;

#ifdef __DEBUG__
          Serial.print("bluetoothCmd");
          Serial.println(bluetoothCmd);
          Serial.print("bluetoothCmdChars");
          Serial.println(bluetoothCmdChars);
#endif

          while ((chuck = strtok_r(p, ",", &p)) != NULL)
          {
            cmd[argIdx] = atoi(chuck);
            argIdx = argIdx + 1;
            // Serial.print("chuck:");
            // Serial.println(chuck);
          }
        }
        else
        {
          memset(cmd, 0, sizeof(cmd));
          cmd[0] = bluetoothCmd.toInt();
        }
      }
      bluetoothCmd = ""; // Reset command after found '*'
    }
  }

  // if (auto_mode)
  // {
  //   static String *movements[] = {FORWARD, LEFT, OMNI_WALK_R, OMNI_WALK_L, RIGHT, BACKWARD, PUSH_UP, UP_DOWN, HELLO, DANCE, FRONT_BACK, MOON_WALK};
  //   static unsigned long old_time = cur_time;
  //   static int c = 0;
  //   if (cur_time - old_time >= TIME_INTERVAL)
  //   {
  //     old_time = cur_time;
  //     //c = (int)random(0, sizeof(movements));
  //     // cmd = movements[c];
  //     if (!random_walk)
  //     {
  //       c = c % (sizeof(movements) / sizeof(char));
  //       cmd = movements[c++];
  //     }
  //     else
  //     {
  //       c = (int)random(0, sizeof(movements) / sizeof(char));
  //       cmd = movements[c];
  //     }
  //   }
  // }
  return rc;
}

void pause(int period)
{
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

void loop()
{

  cur_time = millis();
  if (cur_time - prev_serial_data_time >= 100)
  {
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
