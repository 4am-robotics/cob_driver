/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef SEQUENCEMODE_H
#define SEQUENCEMODE_H

#include <mode.h>

typedef struct sequence
{
  color::rgba color;
  double holdtime;
  double crosstime;
}seq_t;

enum State{INIT_SEQ,CROSSFADE,HOLD,NEXT,BEGIN};

class SequenceMode : public Mode
{
public:
  SequenceMode(std::vector<seq_t> sequences, int priority = 0, double freq = 0.25, int pulses = 0, double timeout = 0)
    :Mode(priority, freq, pulses, timeout), _init(true), _int_inc(0.0), _int_count(0.0)
  {
    _seqences = sequences;
    _state = INIT_SEQ;
  }

  void execute()
  {
    switch (_state)
    {
    case INIT_SEQ:
      //std::cout<<"INIT_SEQ"<<std::endl;
      _seqidx = 0;
      _int_inc = 0.0;
      _int_count = 0.0;
      //_color_old = getColor();
      _int_inc = 1.0/(_seqences[_seqidx].crosstime * UPDATE_RATE_HZ);
      _state = CROSSFADE;
      //std::cout<<"Setting color: "<<_seqences[_seqidx].color.r<<" "<<_seqences[_seqidx].color.g<<" "<<_seqences[_seqidx].color.b<<" "<<_seqences[_seqidx].color.a<<std::endl;
      break;

    case CROSSFADE:
      //std::cout<<"CROSSFADE"<<std::endl;
      //first execute cross fade between old and new color for current seq
      if(_int_count <= 1.0001)
      {
        //std::cout<<"_int_count "<<_int_count<<std::endl;
        _color = interpolateColor(_actualColor, _seqences[_seqidx].color, _int_count);
        _int_count += _int_inc;
        m_sigColorReady(_color);
      }
      else
      {
        _state = HOLD;
        _int_count = 0.0;
        _int_inc = 1.0/(_seqences[_seqidx].holdtime * UPDATE_RATE_HZ);
        _actualColor = _color;
        m_sigColorReady(_color);
      }
      //calculate
      break;

    case HOLD:
      if(_int_count < 1)
        _int_count += _int_inc;
      else
        _state = NEXT;
      break;

    case NEXT:
      //std::cout<<"NEXT"<<std::endl;
      _seqidx++;
      if(_seqidx == _seqences.size())
        _seqidx = 0;
      _int_inc = 0.0;
      _int_count = 0.0;
      _int_inc = 1.0/(_seqences[_seqidx].crosstime * UPDATE_RATE_HZ);
      _state = CROSSFADE;
      //std::cout<<"Setting color: "<<_seqences[_seqidx].color.r<<" "<<_seqences[_seqidx].color.g<<" "<<_seqences[_seqidx].color.b<<" "<<_seqences[_seqidx].color.a<<std::endl;
      break;
    }
  }

  std::string getName(){ return std::string("SequenceMode"); }

private:
  std::vector<seq_t> _seqences;
  int _seqidx;
  bool _init;
  float _int_inc;
  float _int_count;
  int _state;

  color::rgba _color;

  color::rgba interpolateColor(color::rgba start, color::rgba goal, float t)
  {
    color::hsv ca;
    color::hsv cb;
    color::hsv cr;
    color::rgba a, b;
    a = start;
    b = goal;

    a.r *= a.a;
    a.g *= a.a;
    a.b *= a.a;
    b.r *= b.a;
    b.g *= b.a;
    b.b *= b.a;
    color::Color::rgb2hsv(a.r, a.g, a.b, ca.h, ca.s, ca.v);
    color::Color::rgb2hsv(b.r, b.g, b.b, cb.h, cb.s, cb.v);

    cr.h = linearInterpolate(ca.h, cb.h, t);
    cr.s = linearInterpolate(ca.s, cb.s, t);
    cr.v = linearInterpolate(ca.v, cb.v, t);

    color::rgba result;
    color::Color::hsv2rgb(cr.h, cr.s, cr.v, result.r, result.g, result.b);
    result.a = 1.0;

//    std::cout<<"Original h:"<<ca.h<<" s:"<<ca.s<<" v:"<<ca.v<<std::endl;
//    std::cout<<"Original r:"<<a.r<<" g:"<<a.g<<" b:"<<a.b<<std::endl;
//    std::cout<<"Goal     h:"<<cb.h<<" s:"<<cb.s<<" v:"<<cb.v<<std::endl;
//    std::cout<<"Goal     r:"<<b.r<<" g:"<<b.g<<" b:"<<b.b<<std::endl;
//    std::cout<<"New      h:"<<cr.h<<" s:"<<cr.s<<" v:"<<cr.v<<std::endl;
//    std::cout<<"New      r:"<<result.r<<" g:"<<result.g<<" b:"<<result.b<<std::endl;

    return result;
  }

  float linearInterpolate(float a, float b, float t)
  {
    return a * (1 - t) + b * t;
  }
};

#endif
