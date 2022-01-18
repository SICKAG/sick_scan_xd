/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/**
 * @author Blaise Gassend
 */

#ifndef DIAGNOSTIC_UPDATER__DIAGNOSTIC_STATUS_WRAPPER_HPP_
#define DIAGNOSTIC_UPDATER__DIAGNOSTIC_STATUS_WRAPPER_HPP_

#include <stdarg.h>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "rclcpp/rclcpp.hpp"

namespace diagnostic_updater
{

/**
 *
 * \brief Wrapper for the diagnostic_msgs::msg::DiagnosticStatus message that
 * makes it easier to update.
 *
 * This class handles common string formatting and vector handling issues
 * for filling the diagnostic_msgs::msg::DiagnosticStatus message. It is a
 * subclass of
 * diagnostic_msgs::msg::DiagnosticStatus, so it can be passed directly to
 * diagnostic publish calls.
 *
 */
class DiagnosticStatusWrapper : public diagnostic_msgs::msg::DiagnosticStatus
{
public:
  DiagnosticStatusWrapper()
  : debug_logger_(rclcpp::get_logger("diagnostics_wrapper_logger"))
  {}

  /**
   * \brief Copy constructor
   * Defined and marked explicit so that you don't accidentally use a
   * function<void(DiagnosticStatusWrapper)> where a function<void(DiagnosticStatusWrapper &)>
   * is needed. Otherwise, it's easy to accidentally create a DiagnosticTask that silently does
   * nothing.
   * \param other Reference to object to copy
   */
  explicit DiagnosticStatusWrapper(const DiagnosticStatusWrapper & other);

  /**
   * \brief Fills out the level and message fields of the DiagnosticStatus.
   *
   * \param lvl Numerical level to assign to this Status (OK, Warn, Err).
   * \param s Descriptive status message.
   */
  void summary(unsigned char lvl, const std::string s)
  {
    level = lvl;
    message = s;
  }

  /**
   * \brief Merges a level and message with the existing ones.
   *
   * It is sometimes useful to merge two DiagnosticStatus messages. In that
   * case,
   * the key value pairs can be unioned, but the level and summary message
   * have to be merged more intelligently. This function does the merge in
   * an intelligent manner, combining the summary in *this, with the one
   * that is passed in.
   *
   * The combined level is the greater of the two levels to be merged.
   * If both levels are non-zero (not OK), the messages are combined with a
   * semicolon separator. If only one level is zero, and the other is
   * non-zero, the message for the zero level is discarded. If both are
   * zero, the new message is ignored.
   *
   * \param lvl Numerical level to of the merged-in summary.
   * \param s Descriptive status message for the merged-in summary.
   */

  void mergeSummary(unsigned char lvl, const std::string s)
  {
    if ((lvl > 0) && (level > 0)) {
      if (!message.empty()) {
        message += "; ";
        message += s;
      }
    } else if (lvl > level) {
      message = s;
    }
    if (lvl > level) {
      level = lvl;
    }
  }

  /**
   * \brief Version of mergeSummary that merges in the summary from
   * another DiagnosticStatus.
   *
   * \param src DiagnosticStatus from which to merge the summary.
   */

  void mergeSummary(const diagnostic_msgs::msg::DiagnosticStatus & src)
  {
    mergeSummary(src.level, src.message);
  }

  /**
   * \brief Formatted version of mergeSummary.
   *
   * This method is identical to mergeSummary, except that the message is
   * an sprintf-style format string.
   *
   * \param lvl Numerical level to of the merged-in summary.
   * \param format Format string for the descriptive status message for the
   * merged-in summary.
   * \param ... Values to be formatted by the format string.
   */

  void mergeSummaryf(unsigned char lvl, const char * format, ...)
  {
    va_list va;
    const int kBufferSize = 1000;
    char buff[kBufferSize];  //  @todo This could be done more elegantly.
    va_start(va, format);
    if (vsnprintf(buff, sizeof(buff), format, va) >= kBufferSize) {
      RCLCPP_DEBUG(
        debug_logger_,
        "Really long string in DiagnosticStatusWrapper::mergeSummaryf, it was truncated.");
    }
    std::string value = std::string(buff);
    mergeSummary(lvl, value);
    va_end(va);
  }

  /**
   * \brief Formatted version of summary.
   *
   * This method is identical to summary, except that the message is an
   * sprintf-style format string.
   *
   * \param lvl Numerical level to assign to this Status (OK, Warn, Err).
   * \param s Format string for the descriptive status message.
   * \param ... Values to be formatted by the format string.
   *
   */
  void summaryf(unsigned char lvl, const char * format, ...)
  {
    va_list va;
    const int kBufferSize = 1000;
    char buff[kBufferSize];
    va_start(va, format);
    if (vsnprintf(buff, sizeof(buff), format, va) >= kBufferSize) {
      RCLCPP_DEBUG(
        debug_logger_,
        "Really long string in DiagnosticStatusWrapper::summaryf, it was truncated.");
    }
    std::string value = std::string(buff);
    summary(lvl, value);
    va_end(va);
  }

  /**
   * \brief clears the summary, setting the level to zero and the
   * message to "".
   */
  void clearSummary() {summary(0, "");}

  /**
   * \brief copies the summary from a DiagnosticStatus message
   *
   * \param src StatusWrapper to copy the summary from.
   */
  void summary(const diagnostic_msgs::msg::DiagnosticStatus & src)
  {
    summary(src.level, src.message);
  }

  /**
   * \brief Add a key-value pair.
   *
   * This method adds a key-value pair. Any type that has a << stream
   * operator can be passed as the second argument.  Formatting is done
   * using a std::stringstream.
   *
   * \param key Key to be added.  \param value Value to be added.
   */
  template<class T>
  void add(const std::string & key, const T & val)
  {
    std::stringstream ss;
    ss << val;
    std::string sval = ss.str();
    add(key, sval);
  }

  /**
   * \brief Add a key-value pair using a format string.
   *
   * This method adds a key-value pair. A format string is used to set the
   * value. The current implementation limits the value to 1000 characters
   * in length.
   */

  void addf(const std::string & key, const char * format, ...);

  /**
   * \brief Clear the key-value pairs.
   *
   * The values vector containing the key-value pairs is cleared.
   */

  void clear() {values.clear();}

private:
  rclcpp::Logger debug_logger_;
};

template<>
inline void DiagnosticStatusWrapper::add<std::string>(
  const std::string & key,
  const std::string & s)
{
  diagnostic_msgs::msg::KeyValue ds;
  ds.key = key;
  ds.value = s;
  values.push_back(ds);
}

//  /\brief For bool, diagnostic value is "True" or "False"
template<>
inline void DiagnosticStatusWrapper::add<bool>(
  const std::string & key,
  const bool & b)
{
  diagnostic_msgs::msg::KeyValue ds;
  ds.key = key;
  ds.value = b ? "True" : "False";

  values.push_back(ds);
}

// Need to place addf after DiagnosticStatusWrapper::add<std::string> or
// gcc complains that the specialization occurs after instatiation.
inline void
DiagnosticStatusWrapper::addf(
  const std::string & key, const char * format, ...)  // In practice format will always be a char *
{
  va_list va;
  const int kBufferSize = 1000;
  char buff[kBufferSize];  // @todo This could be done more elegantly.
  va_start(va, format);
  if (vsnprintf(buff, sizeof(buff), format, va) >= kBufferSize) {
    RCLCPP_DEBUG(
      debug_logger_,
      "Really long string in DiagnosticStatusWrapper::addf, it was truncated.");
  }
  std::string value = std::string(buff);
  add(key, value);
  va_end(va);
}
}  // namespace diagnostic_updater
#endif  // DIAGNOSTIC_UPDATER__DIAGNOSTIC_STATUS_WRAPPER_HPP_
