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
