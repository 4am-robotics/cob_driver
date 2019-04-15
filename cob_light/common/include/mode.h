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


#ifndef MODE_H
#define MODE_H

#include <ros/ros.h>
#include <colorUtils.h>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>

class Mode
{
public:
    Mode(int priority = 0, double freq = 0, int pulses = 0, double timeout = 0)
        : _priority(priority), _freq(freq), _pulses(pulses), _timeout(timeout),
          _finished(false), _pulsed(0), _isStopRequested(false), _isPauseRequested(false),
          _isRunning(false)
          {
              if(this->getFrequency() == 0.0)
                  this->setFrequency(1.0);
          }
    virtual ~Mode(){}

    void start()
    {
        if(_thread == NULL)
            _thread.reset(new boost::thread(&Mode::run, this));
        if(isPauseRequested())
        {
            boost::mutex::scoped_lock lock(_mutex_pause);
            _isPauseRequested = false;
            _cond_pause.notify_one();
        }
        _isRunning = true;
    }

    void stop()
    {
        _mutex.lock();
        _isStopRequested = true;
        _mutex.unlock();
        if(isPauseRequested())
        {
            _isPauseRequested = false;
            boost::mutex::scoped_lock lock(_mutex_pause);
            _cond_pause.notify_one();
        }
        if(_thread != NULL)
        {
            _thread->join();
            _thread.reset();
        }
        _isStopRequested = false;
        _isRunning = false;
    }

    void pause()
    {
        _mutex.lock();
        _isPauseRequested = true;
        _mutex.unlock();
        _isRunning = false;
    }

    virtual void execute() = 0;

    virtual std::string getName() = 0;

    bool finished(){ return _finished; }

    void setPriority(int priority){ _priority = priority; }
    int getPriority(){ return _priority; }

    void setTimeout(double timeout){ _timeout = timeout; }
    double getTimeout(){ return _timeout; }

    void setFrequency(double freq){ _freq = freq; }
    double getFrequency(){ return _freq; }

    void setPulses(int pulses){ _pulses = pulses; }
    int getPulses(){ return _pulses; }

    int pulsed(){ return _pulsed; }

    void setColor(color::rgba color){ _color = color; }
    color::rgba getColor(){ return _color; }

    void setActualColor(color::rgba color){ _actualColor = color; }
    color::rgba getActualColor(){ return _color; }

    bool isRunning(){ return _isRunning; }

    boost::signals2::signal<void (color::rgba color)>* signalColorReady(){ return &m_sigColorReady; }
    boost::signals2::signal<void (std::vector<color::rgba> &colors)>* signalColorsReady(){ return &m_sigColorsReady; }
    boost::signals2::signal<void (int)>* signalModeFinished(){ return &m_sigFinished; }

protected:
    int _priority;
    double _freq;
    int _pulses;
    double _timeout;

    bool _finished;
    int _pulsed;

    color::rgba _color;
    std::vector<color::rgba> _colors;
    color::rgba _actualColor;
    color::rgba _init_color;

    static const unsigned int UPDATE_RATE_HZ = 100;

    boost::signals2::signal<void (color::rgba color)> m_sigColorReady;
    boost::signals2::signal<void (std::vector<color::rgba> &colors)> m_sigColorsReady;
    boost::signals2::signal<void (int)> m_sigFinished;

private:
    boost::shared_ptr<boost::thread> _thread;
    boost::mutex _mutex;
    boost::mutex _mutex_pause;
    bool _isStopRequested;
    bool _isPauseRequested;
    bool _isRunning;

    boost::condition_variable _cond_pause;

    bool isStopRequested()
    {
        bool ret;
        _mutex.lock();
        ret = _isStopRequested;
        _mutex.unlock();
        return ret;
    }

    bool isPauseRequested()
    {
        bool ret;
        _mutex.lock();
        ret = _isPauseRequested;
        _mutex.unlock();
        return ret;
    }

protected:
    virtual void run()
    {
        ros::Rate r(UPDATE_RATE_HZ);

        ros::Time timeStart = ros::Time::now();

        while(!isStopRequested() && !ros::isShuttingDown())
        {
            while(isPauseRequested())
            {
                boost::mutex::scoped_lock lock(_mutex_pause);
                _cond_pause.wait(lock);
            }
            this->execute();

            if((this->getPulses() != 0) &&
                (this->getPulses() <= this->pulsed()))
                break;

            if(this->getTimeout() != 0)
            {
                ros::Duration timePassed = ros::Time::now() - timeStart;
                if(timePassed.toSec() >= this->getTimeout())
                    break;
            }
            r.sleep();
        }
        ROS_DEBUG("Mode %s finished",this->getName().c_str());
        if(!isStopRequested())
            m_sigFinished(this->getPriority());
    }
};

#endif
