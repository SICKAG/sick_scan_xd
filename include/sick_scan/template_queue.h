#ifndef TEMPLATE_QUEUE_H
#define TEMPLATE_QUEUE_H

#include <queue>
#include <thread>
#include <boost/chrono.hpp>
#include <thread>
#include <boost/chrono.hpp>
#include <iostream>
#include <mutex>
#include <condition_variable>

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
    std::unique_lock<std::mutex> mlock(mutex_);
    retVal = queue_.size();
    return (retVal);
  }


  bool isQueueEmpty()
  {
    bool retVal = false;
    std::unique_lock<std::mutex> mlock(mutex_);
    retVal = queue_.empty();
    return (retVal);
  }

  bool waitForIncomingObject(int timeOutInMs)
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    bool ret = true;
    while (queue_.empty() && (ret == true))
    {
      ret = (cond_.wait_for(mlock, std::chrono::milliseconds(timeOutInMs)) == std::cv_status::no_timeout);
    }
    return (ret);
  }

  T pop()
  {
    std::unique_lock<std::mutex> mlock(mutex_);
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
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
      cond_.wait(mlock);
    }
    item = queue_.front();
    queue_.pop();
  }

  void push(const T &item)
  {
    {
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push(item);
    }
    cond_.notify_one();
  }

  void push(T &item)
  {
    {
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push(item);
    }
    cond_.notify_one();
  }

private:
  std::queue<T> queue_;
  std::mutex mutex_;
  std::condition_variable cond_;
};

#endif
