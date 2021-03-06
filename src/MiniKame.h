#ifndef minikame_h
#define minikame_h
#include <Servo.h>
#include "Octosnake.h"

// servo index to board_pins

#define FRONT_RIGHT_HIP 0
#define FRONT_LEFT_HIP 1
#define FRONT_RIGHT_LEG 2
#define FRONT_LEFT_LEG 3
#define BACK_RIGHT_HIP 4
#define BACK_LEFT_HIP 5
#define BACK_RIGHT_LEG 6
#define BACK_LEFT_LEG 7

extern "C" void pause(int);

class MiniKame {

  public:
    MiniKame();
    void init();
    void home();
    void rest();
    void leveling(int level = 90);
    void run(int dir = 1, float steps = 2, float T = 1000); //default 550
    void walk(int dir = 1, float steps = 0, float T = 1000); // default 550
    void turnL(float steps = 4, float period = 550);
    void turnR(float steps = 4, float period = 550);
    void omniWalk(bool side = true, float T = 1000, float turn_factor = 1.5);
    void moonwalkL(float steps = 0, float period = 2000);
    void dance(float steps = 0, float period = 2000);
    void upDown(float steps = 0, float period = 500);
    void pushUp(float steps = 0, float period = 5000);
    void frontBack(float steps = 0, float period = 2000);
    void hello();
    void jump();
    void moveServos(int time, float target[8]);
    void setServo(int id, float target);
    void setTrim(int index, int value) {
      trim[index] = value;
    }
    void refresh();
    void storeTrim();
    void loadTrim();
  private:
    Oscillator oscillator[8];
    Servo servo[8];
    int board_pins[8];
    int trim[8]; //deviation servo offset
    //unsigned long _init_time;
    // unsigned long _final_time;
    void execute(float steps, float period[8], int amplitude[8], int offset[8], int phase[8]);

    int EEPROMReadWord(int p_address);
    void EEPROMWriteWord(int p_address, int p_value);
};

#endif
