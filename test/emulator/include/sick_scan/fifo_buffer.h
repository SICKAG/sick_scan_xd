/*
 * @brief sim_loc_fifo implements a threadsafe fifo-buffer ("first in, first out").
 *
 * Copyright (C) 2019 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2019 SICK AG, Waldkirch
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of SICK AG nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 *  Copyright 2019 SICK AG
 *  Copyright 2019 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifndef __SIM_LOC_FIFO_H_INCLUDED
#define __SIM_LOC_FIFO_H_INCLUDED

#include <boost/thread.hpp>
#include <list>

namespace sick_scan
{
  /*!
   * Class FifoBuffer implements a threadsafe fifo-buffer ("first in, first out").
   */
  template<typename ElementType, typename MutexType = boost::mutex> class FifoBuffer
  {
  public:
    
    /*!
     * Constructor
     */
    FifoBuffer() : m_fifo_buffer() {}
    
    /*!
     * Destructor
     */
    ~FifoBuffer()
    {
      notify(); // interrupt a possible wait in waitForNotify()
    }
  
    /*!
     * Returns true, if the fifo buffer is empty.
     */
    bool empty(void)
    {
      boost::lock_guard<MutexType> message_lockguard(m_fifo_mutex);
      return m_fifo_buffer.empty();
    }
  
    /*!
     * Returns the number of elements in the fifo buffer.
     */
    size_t size(void)
    {
      boost::lock_guard<MutexType> message_lockguard(m_fifo_mutex);
      return m_fifo_buffer.size();
    }
  
    /*!
     * Pushes an element to the end of the fifo buffer.
     */
    void push(const ElementType & elem)
    {
      push_back(elem);
      notifyAll();
    }
  
    /*!
     * Removes and returns the first element from the fifo buffer.
     * @return first element in the buffer, or T() if the fifo is empty.
     */
    ElementType pop(void)
    {
      boost::lock_guard<MutexType> message_lockguard(m_fifo_mutex);
      if(!m_fifo_buffer.empty())
      {
        ElementType elem(m_fifo_buffer.front());
        m_fifo_buffer.pop_front();
        return elem;
      }
      return ElementType();
    }
  
    /*!
     * Waits until there's at least one element in the fifo buffer.
     */
    void waitForElement()
    {
      while (ROS::ok() && empty())
      {
        waitForNotify();
      }
    }
  
    /*!
     * Waits until there's at least one element in the fifo buffer, or a notification has been signalled.
     */
    void waitOnceForElement()
    {
      if (ROS::ok() && empty())
      {
        waitForNotify();
      }
    }

    /*!
     * Signal a notification to interrupt a waiting waitForElement() call
     */
    void notify(void)
    {
      notifyAll();
    }
    
    /*!
     * Interface class to search for an element with a unary condition.
     */
    class UnaryConditionIf
    {
    public:
      /** Search callback: return true, if the search condition for an element is true, or false otherwise */
      virtual bool condition(const ElementType & element) = 0;
    };
    
    /*!
     * Searches for an element in the fifo by a unary condition.
     * Returns and optionally removes an element from the fifo buffer, if condition_impl.condition(ElementType &) returns true.
     * @param[in] condition_impl condition (callback interface, returns true, if condition for an element is true, or false otherwise)
     * @param[in] erase_if_found if true, the element found will be erased (otherwise left untouched)
     * @return element found and removed in the buffer, or T() if the fifo is empty.
     */
    ElementType findFirstIf(UnaryConditionIf & condition_impl, bool erase_if_found = false)
    {
      boost::lock_guard<MutexType> message_lockguard(m_fifo_mutex);
      for(auto iter = m_fifo_buffer.begin(); iter != m_fifo_buffer.end(); iter++)
      {
        if(condition_impl.condition(*iter))
        {
          ElementType elem = *iter;
          if(erase_if_found)
          {
            m_fifo_buffer.erase(iter);
          }
          return elem;
        }
      }
      return ElementType();
    }

  protected:
  
    /*! Pushes an element to the end of the fifo buffer. */
    void push_back(const ElementType & elem)
    {
      boost::lock_guard<MutexType> message_lockguard(m_fifo_mutex);
      m_fifo_buffer.push_back(elem);
    }
  
    /*! Notification after changes in fifo buffer size. */
    void notifyAll(void)
    {
      m_buffer_condition.notify_all();
    }
  
    /*! Wait until notification signalled. */
    void waitForNotify(void)
    {
      boost::mutex::scoped_lock lock(m_condition_mutex);
      m_buffer_condition.wait(lock);
    }

    /*
     * member data
     */
    
    std::list<ElementType> m_fifo_buffer;          ///< list of all elements of the fifo buffer
    MutexType m_fifo_mutex;                        ///< mutex to lock m_fifo_buffer
    boost::mutex m_condition_mutex;                ///< mutex to lock m_buffer_condition
    boost::condition_variable m_buffer_condition;  ///< condition variable to signal changes in buffer size
    
  }; // class FifoBuffer
  
} // namespace sick_scan
#endif // __SIM_LOC_FIFO_H_INCLUDED
