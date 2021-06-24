#ifndef TEMPLATE_QUEUE_H
#define TEMPLATE_QUEUE_H

#include <queue>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <iostream>

template<typename T>
class Queue
{
public:

  /*!
  \brief get number of entries in queue
  \return Number of entries in queue
  \sa isQueueEmpty()
  */
  int getNumberOfEntriesInQueue()
  {
    int retVal = 0;
    boost::mutex::scoped_lock mlock(mutex_);
    retVal = queue_.size();
    return (retVal);
  }


  bool isQueueEmpty()
  {
    bool retVal = false;
    boost::mutex::scoped_lock mlock(mutex_);
    retVal = queue_.empty();
    return (retVal);
  }

  bool waitForIncomingObject(int timeOutInMs)
  {
    boost::mutex::scoped_lock mlock(mutex_);
    bool ret = true;
    boost::posix_time::time_duration td = boost::posix_time::millisec(timeOutInMs);
    while (queue_.empty() && (ret == true))
    {
      ret = cond_.timed_wait(mlock, td);
    }
    return (ret);
  }

  T pop()
  {
    boost::mutex::scoped_lock mlock(mutex_);
    while (queue_.empty())
    {
      cond_.wait(mlock);
    }
    T item = queue_.front();
    queue_.pop();
    return item;
  }

  void pop(T &item)
  {
    boost::mutex::scoped_lock mlock(mutex_);
    while (queue_.empty())
    {
      cond_.wait(mlock);
    }
    item = queue_.front();
    queue_.pop();
  }

  void push(const T &item)
  {
    boost::mutex::scoped_lock mlock(mutex_);
    queue_.push(item);
    mlock.unlock();
    cond_.notify_one();
  }

  void push(T &item)
  {
    boost::mutex::scoped_lock mlock(mutex_);
    queue_.push(item);
    mlock.unlock();
    cond_.notify_one();
  }

private:
  std::queue<T> queue_;
  boost::mutex mutex_;
  boost::condition_variable cond_;
};

#endif
