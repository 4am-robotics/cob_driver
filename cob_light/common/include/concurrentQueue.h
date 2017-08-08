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


#include <iostream>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>

template<typename T>
class ConcurrentQueue
{
private:
    boost::circular_buffer<T> _queue;
    mutable boost::mutex _mutex;
    boost::condition_variable _condition;
public:
    ConcurrentQueue()
	: _queue(2)
	{
	}

    bool empty() const
    {
        boost::mutex::scoped_lock lock(_mutex);
        return _queue.empty();
    }

    void push(T const& data)
    {
        boost::mutex::scoped_lock lock(_mutex);
        _queue.push_back(data);
        lock.unlock();
        _condition.notify_one();
    }

    bool pop(T& data)
    {
        boost::mutex::scoped_lock lock(_mutex);
        if(_queue.empty())
            return false;

        data = _queue.front();
        _queue.pop_front();

        return true;
    }

    void wait_pop(T& data)
    {
        boost::mutex::scoped_lock lock(_mutex);
        while(_queue.empty())
        	_condition.wait(lock);

        data=_queue.front();
        _queue.pop_front();
    }
};
