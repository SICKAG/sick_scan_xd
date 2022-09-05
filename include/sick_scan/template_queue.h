#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
#ifndef TEMPLATE_QUEUE_H
#define TEMPLATE_QUEUE_H

#include <queue>
#include <thread>
#include <thread>
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

  bool waitForIncomingObject(int timeOutInMs, const std::vector<std::string>& datagram_keywords)
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    bool ret = true;
    typename std::list<T>::iterator datagram_found;
    while (findFirstByKeyword(datagram_keywords, datagram_found) == false && (ret == true))
    {
      ret = (cond_.wait_for(mlock, std::chrono::milliseconds(timeOutInMs)) == std::cv_status::no_timeout);
    }
    return (ret);
  }

  T pop(const std::vector<std::string>& datagram_keywords) // T pop(const std::vector<std::string>& datagram_keywords = {})
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    typename std::list<T>::iterator datagram_found;
    while (findFirstByKeyword(datagram_keywords, datagram_found) == false)
    {
      cond_.wait(mlock);
    }
    T item = *datagram_found;
    queue_.erase(datagram_found);
    return item;
  }

  void push(const T &item)
  {
    {
      std::unique_lock<std::mutex> mlock(mutex_);
      queue_.push_back(item);
    }
    cond_.notify_all(); // cond_.notify_one();
  }


protected:
  
  bool findFirstByKeyword(const std::vector<std::string>& keywords, typename std::list<T>::iterator & iter)
  {
    iter = queue_.begin();
    if(keywords.empty())
    {
      return !queue_.empty();
    }
    for( ; iter != queue_.end(); iter++)
    {
      std::vector<unsigned char>& datagram = iter->data();
      uint32_t cola_b_start = 0x02020202;
      uint8_t* datagram_keyword_start = 0;
      int datagram_keyword_maxlen = (int)datagram.size();
      int commandIdOffset = 1;
      if(datagram.size() > 12 && memcmp(datagram.data(),&cola_b_start, sizeof(cola_b_start)) == 0)
      {
        commandIdOffset = 8; // command id behind 0x02020202 + { 4 byte payload length }
        datagram_keyword_start = datagram.data() + 12; // 0x02020202 + { 4 byte payload length } + { 4 byte command id incl. space }
        datagram_keyword_maxlen -= 12;
      }
      else if(datagram.size() > 5)
      {
        commandIdOffset = 1;
        datagram_keyword_start = datagram.data() + 5; // 0x02 + { 4 byte command id incl. space }
        datagram_keyword_maxlen -= 5;
      }
      else
        continue;
      
      for(int keyword_idx = 0; keyword_idx < keywords.size(); keyword_idx++)
      {
        const std::string& keyword = keywords[keyword_idx];
        if(keyword.size() <= datagram_keyword_maxlen && memcmp(datagram_keyword_start, keyword.data(), keyword.size()) == 0)
        {
          // ROS_DEBUG_STREAM("Queue::findFirstByKeyword(): keyword_start=\"" << std::string((char*)datagram_keyword_start, keyword.size()) << "\", keyword=\"" << keyword << "\"");
          return true;
        }
      }

      // keyword not found.
      // Check for possible sFA command as an error reply
      std::string errorIdentifier = "sFA";
      if (datagram.size() >= (commandIdOffset + errorIdentifier.length()))
      {
        const char* errorIdentifierPtr = errorIdentifier.c_str();
        const char* cmpPtr = (const char *)(datagram.data() + commandIdOffset);
        if (memcmp(cmpPtr, errorIdentifierPtr, errorIdentifier.length() )== 0)
        {
          ROS_DEBUG_STREAM("Queue::findFirstByKeyword(): error identifier sFA found in datagram");
          return true;
        }
      }
    }
    return false; // keyword not found, iter == queue_.end()
  }

  std::list<T> queue_;
  std::mutex mutex_;
  std::condition_variable cond_;
};

#endif
