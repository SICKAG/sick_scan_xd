/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: Blaise Gassend
#ifndef DIAGNOSTIC_UPDATER__PUBLISHER_HPP_
#define DIAGNOSTIC_UPDATER__PUBLISHER_HPP_

#include <memory>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_updater/update_functions.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"

namespace
{
/**
 * \brief Helper struct to check message for header member.
 */
template<typename T, typename = void>
struct has_header : public std::false_type
{};

/**
 * \brief Helper struct to check message for header member.
 */
template<typename T>
struct has_header<T,
  typename std::enable_if<std::is_same<std_msgs::msg::Header,
  decltype(std::declval<T>().header)>::value>::type>
  : public std::true_type
{};
}  // namespace

namespace diagnostic_updater
{

/**
 * \brief A class to facilitate making diagnostics for a topic using a
 * FrequencyStatus.
 *
 * The word "headerless" in the class name refers to the fact that it is
 * mainly designed for use with messages that do not have a header, and
 * that cannot therefore be checked using a TimeStampStatus.
 */
class HeaderlessTopicDiagnostic : public CompositeDiagnosticTask
{
public:
  /**
   * \brief Constructs a HeaderlessTopicDiagnostic.
   *
   * \param name The name of the topic that is being diagnosed.
   *
   * \param diag The diagnostic_updater that the CompositeDiagnosticTask
   * should add itself to.
   *
   * \param freq The parameters for the FrequencyStatus class that will be
   * computing statistics.
   */

  HeaderlessTopicDiagnostic(
    std::string name, diagnostic_updater::Updater & diag,
    const diagnostic_updater::FrequencyStatusParam & freq)
  : CompositeDiagnosticTask(name + " topic status"), freq_(freq)
  {
    addTask(&freq_);
    diag.add(*this);
  }

  virtual ~HeaderlessTopicDiagnostic() {}

  /**
   * \brief Signals that a publication has occurred.
   */

  virtual void tick() {freq_.tick();}

  /**
   * \brief Clears the frequency statistics.
   */

  virtual void clear_window() {freq_.clear();}

private:
  diagnostic_updater::FrequencyStatus freq_;
};

/**
 * \brief A class to facilitate making diagnostics for a topic using a
 * FrequencyStatus and TimeStampStatus.
 */

class TopicDiagnostic : public HeaderlessTopicDiagnostic
{
public:
  /**
   * \brief Constructs a TopicDiagnostic.
   *
   * \param name The name of the topic that is being diagnosed.
   *
   * \param diag The diagnostic_updater that the CompositeDiagnosticTask
   * should add itself to.
   *
   * \param freq The parameters for the FrequencyStatus class that will be
   * computing statistics.
   *
   * \param stamp The parameters for the TimeStampStatus class that will be
   * computing statistics.
   *
   * \param clock Pointer to a clock instance. If not provided, the default
   * one will be used
   */

  TopicDiagnostic(
    std::string name, diagnostic_updater::Updater & diag,
    const diagnostic_updater::FrequencyStatusParam & freq,
    const diagnostic_updater::TimeStampStatusParam & stamp,
    const rclcpp::Clock::SharedPtr & clock = std::make_shared<rclcpp::Clock>())
  : HeaderlessTopicDiagnostic(name, diag, freq),
    stamp_(stamp, clock),
    error_logger_(rclcpp::get_logger("TopicDiagnostic_error_logger"))
  {
    addTask(&stamp_);
  }

  virtual ~TopicDiagnostic() {}

  /**
   * This method should never be called on a TopicDiagnostic as a
   * timestamp
   * is needed to collect the timestamp diagnostics. It is defined here to
   * prevent the inherited tick method from being used accidentally.
   */
  virtual void tick()
  {
    std::string error_msg = "tick(void) has been called on a TopicDiagnostic.";
    error_msg += " This is never correct. Use tick(rclcpp::Time &) instead.";
    RCLCPP_FATAL(error_logger_, "%s", error_msg.c_str());
  }

  /**
   * \brief Collects statistics and publishes the message.
   *
   * \param stamp Timestamp to use for interval computation by the
   * TimeStampStatus class.
   */
  virtual void tick(const rclcpp::Time & stamp)
  {
    stamp_.tick(stamp);
    HeaderlessTopicDiagnostic::tick();
  }

private:
  TimeStampStatus stamp_;
  rclcpp::Logger error_logger_;
};

/**
 * \brief A TopicDiagnostic combined with a ros::Publisher.
 *
 * For a standard ros::Publisher, this class allows the ros::Publisher and
 * the TopicDiagnostic to be combined for added convenience.
 */

template<typename MessageT, typename AllocatorT = std::allocator<void>>
class DiagnosedPublisher : public TopicDiagnostic
{
public:
  /**
   * \brief Constructs a DiagnosedPublisher.
   *
   * \param pub The publisher on which statistics are being collected.
   *
   * \param diag The diagnostic_updater that the CompositeDiagnosticTask
   * should add itself to.
   *
   * \param freq The parameters for the FrequencyStatus class that will be
   * computing statistics.
   *
   * \param stamp The parameters for the TimeStampStatus class that will be
   * computing statistics.
   */

  using PublisherT = rclcpp::Publisher<MessageT, AllocatorT>;

  DiagnosedPublisher(
    const typename PublisherT::SharedPtr & pub,
    diagnostic_updater::Updater & diag,
    const diagnostic_updater::FrequencyStatusParam & freq,
    const diagnostic_updater::TimeStampStatusParam & stamp)
  : TopicDiagnostic(pub->get_topic_name(), diag, freq, stamp),
    publisher_(pub)
  {
    static_assert(has_header<MessageT>::value, "Message type has to have a header.");
  }

  virtual ~DiagnosedPublisher() {}

  /**
   * \brief Collects statistics and publishes the message.
   *
   * The timestamp to be used by the TimeStampStatus class will be
   * extracted from message.header.stamp.
   */
  virtual void publish(typename PublisherT::MessageUniquePtr message)
  {
    tick(message->header.stamp);
    publisher_->publish(std::move(message));
  }

  /**
   * \brief Collects statistics and publishes the message.
   *
   * The timestamp to be used by the TimeStampStatus class will be
   * extracted from message.header.stamp.
   */
  virtual void publish(const MessageT & message)
  {
    tick(message.header.stamp);
    publisher_->publish(message);
  }

  /**
   * \brief Returns the publisher.
   */
  typename PublisherT::SharedPtr
  getPublisher() const
  {
    return publisher_;
  }

  /**
   * \brief Changes the publisher.
   */
  void setPublisher(typename PublisherT::SharedPtr pub)
  {
    publisher_ = pub;
  }

private:
  typename PublisherT::SharedPtr publisher_;
};
}   // namespace diagnostic_updater

#endif  // DIAGNOSTIC_UPDATER__PUBLISHER_HPP_
