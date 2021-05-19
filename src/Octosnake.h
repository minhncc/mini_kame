#ifndef octosnake_h
#define octosnake_h

#include <Arduino.h>
#include <Servo.h>

#ifndef PI
  #define PI 3.14159
#endif


class Oscillator{

    public:
        Oscillator();
        float refresh();
        void reset();
        void start();
        void start(unsigned long ref_time);
        void stop();
        boolean isStop();
        float time_to_radians(double time);
        float degrees_to_radians(float degrees);
        float degrees_to_time(float degrees);
        void setPeriod(int period);
        int getPeriod();
        void setAmplitude(int amplitude);
        void setPhase(int phase);
        void setOffset(int offset);
        void setTime(unsigned long ref);
        float getOutput();
        float getPhaseProgress();
        unsigned long getTime();
        void setCycles(float c) { _stop_at = c * (_amplitude) * 8; };
        // void setStopAmplitude(int amplitude) { _stop_at_amplitude = amplitude; };

      private:
        int _period;
        int _amplitude;
        int _phase;
        int _offset;
        float _output;
        float _prev_output;
        bool _current_direction; // true: increasing, false: decreasing
        bool _stop;
        float _stop_at = 0;
        // int _stop_at_amplitude = NULL;
        float _increase_amplitude = 0;
        float _moved_amplitude = 0;
        unsigned long _ref_time = 0;
        float _delta_time = 0;
};

#endif
