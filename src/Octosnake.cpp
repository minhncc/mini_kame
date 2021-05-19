#include "Octosnake.h"
#include <Servo.h>
#include <math.h>

Oscillator::Oscillator()
{
  _period = 2000;
  _amplitude = 50;
  _phase = 0;
  _offset = 0;
  _stop = true;
  _stop_at = 0;
  _ref_time = millis();
  _delta_time = 0;
  _prev_output = 0;
  _current_direction = true;
}

float Oscillator::refresh()
{
  _prev_output = _output;

  if (_stop_at != 0)
  {
    _moved_amplitude += _increase_amplitude;
    if (_moved_amplitude > _stop_at)
    {
      _moved_amplitude = 0;
      _stop_at = 0;
      _stop = true;
    }
  }

  if (!_stop)
  {
    _delta_time = (millis() - _ref_time) % _period;
    _output = (float)_amplitude * sin(time_to_radians(_delta_time) + degrees_to_radians(_phase)) + _offset;

    _current_direction = _prev_output < _output;
    _increase_amplitude = abs(_prev_output - _output);
  }

  return _output;
}

void Oscillator::reset()
{
  _ref_time = millis();
}

void Oscillator::start()
{
  reset();
  _stop = false;
}

void Oscillator::start(unsigned long ref_time)
{
  _ref_time = ref_time;
  _stop = false;
}

void Oscillator::stop()
{
  _stop = true;
}

boolean Oscillator::isStop()
{
  return _stop;
}
void Oscillator::setPeriod(int period)
{
  _period = period;
}

int Oscillator::getPeriod()
{
  return _period;
}

void Oscillator::setAmplitude(int amplitude)
{
  _amplitude = amplitude;
}

void Oscillator::setPhase(int phase)
{
  _phase = phase;
}

void Oscillator::setOffset(int offset)
{
  _offset = offset;
}

void Oscillator::setTime(unsigned long ref)
{
  _ref_time = ref;
}

float Oscillator::getOutput()
{
  return _output;
}

unsigned long Oscillator::getTime()
{
  return _ref_time;
}

float Oscillator::getPhaseProgress()
{
  return ((float)_delta_time / _period) * 360;
}

float Oscillator::time_to_radians(double time)
{
  return time * 2 * PI / _period;
}

float Oscillator::degrees_to_radians(float degrees)
{
  return degrees * 2 * PI / 360;
}

float Oscillator::degrees_to_time(float degrees)
{
  return degrees * _period / 360;
}
