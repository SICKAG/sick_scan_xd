#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
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

#ifndef DIAGNOSTIC_UPDATER__DIAGNOSTIC_UPDATER_HPP_
#define DIAGNOSTIC_UPDATER__DIAGNOSTIC_UPDATER_HPP_

#include <functional>  // for bind()
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "diagnostic_updater/diagnostic_status_wrapper.hpp"

#include "rcl/time.h"

#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diagnostic_updater
{

typedef std::function<void (DiagnosticStatusWrapper &)> TaskFunction;
typedef std::function<void (diagnostic_msgs::msg::DiagnosticStatus &)>
  UnwrappedTaskFunction;

/**
 * \brief DiagnosticTask is an abstract base class for collecting diagnostic
 * data.
 *
 * Subclasses are provided for generating common diagnostic information.
 *
 * A DiagnosticTask has a name, and a function that is called to cleate a
 * DiagnosticStatusWrapper.
 */
class DiagnosticTask
{
public:
  /**
   * \brief Constructs a DiagnosticTask setting its name in the process.
   */
  explicit DiagnosticTask(const std::string name)
  : name_(name) {}

  /**
   * \brief Returns the name of the DiagnosticTask.
   */
  const std::string & getName() {return name_;}

  /**
   * \brief Fills out this Task's DiagnosticStatusWrapper.
   */
  virtual void run(diagnostic_updater::DiagnosticStatusWrapper & stat) = 0;

  /**
   * Virtual destructor as this is a base class.
   */
  virtual ~DiagnosticTask() {}

private:
  const std::string name_;
};

/**
 * \brief a DiagnosticTask based on a std::function.
 *
 * The GenericFunctionDiagnosticTask calls the function when it updates. The
 * function
 * updates the DiagnosticStatusWrapper and collects data.
 *
 * This is useful for gathering information about a device or driver, like
 * temperature,
 * calibration, etc.
 */
template<class T>
class GenericFunctionDiagnosticTask : public DiagnosticTask
{
public:
  /**
   * Constructs a GenericFunctionDiagnosticTask based on the given name and
   * function.
   *
   * \param name Name of the function.
   *
   * \param fn Function to be called when DiagnosticTask::run is called.
   */
  GenericFunctionDiagnosticTask(
    const std::string & name,
    std::function<void(T &)> fn)
  : DiagnosticTask(name), fn_(fn) {}

  virtual void run(DiagnosticStatusWrapper & stat) {fn_(stat);}

private:
  const std::string name_;
  const TaskFunction fn_;
};

typedef GenericFunctionDiagnosticTask<diagnostic_msgs::msg::DiagnosticStatus>
  UnwrappedFunctionDiagnosticTask;
typedef GenericFunctionDiagnosticTask<DiagnosticStatusWrapper>
  FunctionDiagnosticTask;

/**
 * \brief Merges CompositeDiagnosticTask into a single DiagnosticTask.
 *
 * The CompositeDiagnosticTask allows multiple DiagnosticTask instances to
 * be combined into a single task that produces a single single
 * DiagnosticStatusWrapped. The output of the combination has the max of
 * the status levels, and a concatenation of the non-zero-level messages.
 *
 * For instance, this could be used to combine the calibration and offset data
 * from an
 * IMU driver.
 */
class CompositeDiagnosticTask : public DiagnosticTask
{
public:
  /**
   * \brief Constructs a CompositeDiagnosticTask with the given name.
   */
  explicit CompositeDiagnosticTask(const std::string name)
  : DiagnosticTask(name) {}

  /**
   * \brief Runs each child and merges their outputs.
   */
  virtual void run(DiagnosticStatusWrapper & stat)
  {
    DiagnosticStatusWrapper combined_summary;
    DiagnosticStatusWrapper original_summary;

    original_summary.summary(stat);

    for (std::vector<DiagnosticTask *>::iterator i = tasks_.begin();
      i != tasks_.end(); i++)
    {
      // Put the summary that was passed in.
      stat.summary(original_summary);
      // Let the next task add entries and put its summary.
      (*i)->run(stat);
      // Merge the new summary into the combined summary.
      combined_summary.mergeSummary(stat);
    }

    // Copy the combined summary into the output.
    stat.summary(combined_summary);
  }

  /**
   * \brief Adds a child CompositeDiagnosticTask.
   *
   * This CompositeDiagnosticTask will be called each time this
   * CompositeDiagnosticTask is run.
   */
  void addTask(DiagnosticTask * t) {tasks_.push_back(t);}

private:
  std::vector<DiagnosticTask *> tasks_;
};

/**
 * \brief Internal use only.
 *
 * Base class for diagnostic_updater::Updater and self_test::Dispatcher.
 * The class manages a collection of diagnostic updaters. It contains the
 * common functionality used for producing diagnostic updates and for
 * self-tests.
 */
class DiagnosticTaskVector
{
protected:
  /**
   * \brief Class used to represent a diagnostic task internally in
   * DiagnosticTaskVector.
   */
  class DiagnosticTaskInternal
  {
public:
    DiagnosticTaskInternal(const std::string name, TaskFunction f)
    : name_(name), fn_(f) {}

    void run(diagnostic_updater::DiagnosticStatusWrapper & stat) const
    {
      stat.name = name_;
      fn_(stat);
    }

    const std::string & getName() const {return name_;}

private:
    std::string name_;
    TaskFunction fn_;
  };

  std::mutex lock_;

  /**
   * \brief Returns the vector of tasks.
   */
  const std::vector<DiagnosticTaskInternal> & getTasks() {return tasks_;}

public:
  virtual ~DiagnosticTaskVector() {}

  /**
   * \brief Add a DiagnosticTask embodied by a name and function to the
   * DiagnosticTaskVector
   *
   * \param name Name to autofill in the DiagnosticStatusWrapper for this task.
   *
   * \param f Function to call to fill out the DiagnosticStatusWrapper.
   * This function need not remain valid after the last time the tasks are
   * called, and in particular it need not be valid at the time the
   * DiagnosticTaskVector is destructed.
   */
  void add(const std::string & name, TaskFunction f)
  {
    DiagnosticTaskInternal int_task(name, f);
    addInternal(int_task);
  }

  /**
   * \brief Add a DiagnosticTask to the DiagnosticTaskVector
   *
   * \param task The DiagnosticTask to be added. It must remain live at
   * least until the last time its diagnostic method is called. It need not be
   * valid at the time the DiagnosticTaskVector is destructed.
   */
  void add(DiagnosticTask & task)
  {
    TaskFunction f = std::bind(&DiagnosticTask::run, &task, std::placeholders::_1);
    add(task.getName(), f);
  }

  /**
   * \brief Add a DiagnosticTask embodied by a name and method to the
   * DiagnosticTaskVector
   *
   * \param name Name to autofill in the DiagnosticStatusWrapper for this task.
   *
   * \param c Class instance the method is being called on.
   *
   * \param f Method to call to fill out the DiagnosticStatusWrapper.
   * This method need not remain valid after the last time the tasks are
   * called, and in particular it need not be valid at the time the
   * DiagnosticTaskVector is destructed.
   */
  template<class T>
  void add(
    const std::string name, T * c,
    void (T::* f)(diagnostic_updater::DiagnosticStatusWrapper &))
  {
    DiagnosticTaskInternal int_task(name, std::bind(f, c, std::placeholders::_1));
    addInternal(int_task);
  }

  /**
   * \brief Remove a task based on its name.
   *
   * Removes the first task that matches the specified name. (New in
   * version 1.1.2)
   *
   * \param name Name of the task to remove.
   *
   * \return Returns true if a task matched and was removed.
   */
  bool removeByName(const std::string name)
  {
    std::unique_lock<std::mutex> lock(lock_);
    for (std::vector<DiagnosticTaskInternal>::iterator iter = tasks_.begin();
      iter != tasks_.end(); iter++)
    {
      if (iter->getName() == name) {
        tasks_.erase(iter);
        return true;
      }

      diagnostic_updater::DiagnosticStatusWrapper status;
    }
    return false;
  }

private:
  /**
   * Allows an action to be taken when a task is added. The Updater class
   * uses this to immediately publish a diagnostic that says that the node
   * is loading.
   */
  virtual void addedTaskCallback(DiagnosticTaskInternal &) {}
  std::vector<DiagnosticTaskInternal> tasks_;

protected:
  /**
   * Common code for all add methods.
   */
  void addInternal(DiagnosticTaskInternal & task)
  {
    std::unique_lock<std::mutex> lock(lock_);
    tasks_.push_back(task);
    addedTaskCallback(task);
  }
};

/**
 * \brief Manages a list of diagnostic tasks, and calls them in a
 * rate-limited manner.
 *
 * This class manages a list of diagnostic tasks. Its update function
 * should be called frequently. At some predetermined rate, the update
 * function will cause all the diagnostic tasks to run, and will collate
 * and publish the resulting diagnostics. The publication rate is
 * determined by the "~/diagnostic_updater.period" ros2 parameter.
 * The force_update function can always be triggered async to the period interval.
 */
class Updater : public DiagnosticTaskVector
{
public:
  bool verbose_;

  /**
   * \brief Constructs an updater class.
   *
   * \param node Node pointer to set up diagnostics
   * \param period Value in seconds to set the update period
   * \note The given period value not being used if the `diagnostic_updater.period`
   * ros2 parameter was set previously.
   */
  template<class NodeT>
  explicit Updater(NodeT node, double period = 1.0)
  : Updater(
      node->get_node_base_interface(),
      node->get_node_logging_interface(),
      node->get_node_parameters_interface(),
      node->get_node_timers_interface(),
      node->get_node_topics_interface(),
      period)
  {}

  Updater(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> logging_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics_interface,
    double period = 1.0)
  : verbose_(false),
    base_interface_(base_interface),
    timers_interface_(timers_interface),
    clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
    period_(static_cast<rcl_duration_value_t>(period * 1e9)),
    publisher_(
      rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        topics_interface, "/diagnostics", 1)),
    logger_(logging_interface->get_logger()),
    node_name_(base_interface->get_name()),
    warn_nohwid_done_(false)
  {
    period = parameters_interface->declare_parameter(
      "diagnostic_updater.period", rclcpp::ParameterValue(period)).get<double>();
    period_ = rclcpp::Duration(static_cast<rcl_duration_value_t>(period * 1e9));

    reset_timer();
  }

  /**
   * \brief Returns the interval between updates.
   */
  auto getPeriod() const {return period_;}

  /**
   * \brief Sets the period as a rclcpp::Duration
   */
  void setPeriod(rclcpp::Duration period)
  {
    period_ = period;
    reset_timer();
  }

  /**
   * \brief Sets the period as a rcl_duration_value_t
   */
  void setPeriod(rcl_duration_value_t period)
  {
    setPeriod(rclcpp::Duration(period));
  }

  /**
   * \brief Sets the period given a value in seconds
   */
  void setPeriod(double period)
  {
    setPeriod(static_cast<rcl_duration_value_t>(period * 1e9));
  }

  /**
   * \brief Forces to send out an update for all known DiagnosticStatus.
   */
  void force_update()
  {
    update();
  }

  /**
   * \brief Output a message on all the known DiagnosticStatus.
   *
   * Useful if something drastic is happening such as shutdown or a
   * self-test.
   *
   * \param lvl Level of the diagnostic being output.
   *
   * \param msg Status message to output.
   */
  void broadcast(int lvl, const std::string msg)
  {
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec;

    const std::vector<DiagnosticTaskInternal> & tasks = getTasks();
    for (std::vector<DiagnosticTaskInternal>::const_iterator iter =
      tasks.begin();
      iter != tasks.end(); iter++)
    {
      diagnostic_updater::DiagnosticStatusWrapper status;

      status.name = iter->getName();
      status.summary(lvl, msg);

      status_vec.push_back(status);
    }

    publish(status_vec);
  }

  void setHardwareIDf(const char * format, ...)
  {
    va_list va;
    const int kBufferSize = 1000;
    char buff[kBufferSize];  // @todo This could be done more elegantly.
    va_start(va, format);
    if (vsnprintf(buff, kBufferSize, format, va) >= kBufferSize) {
      RCLCPP_DEBUG(logger_, "Really long string in diagnostic_updater::setHardwareIDf.");
    }
    hwid_ = std::string(buff);
    va_end(va);
  }

  void setHardwareID(const std::string & hwid) {hwid_ = hwid;}

private:
  void reset_timer()
  {
    update_timer_ = rclcpp::create_timer(
      base_interface_,
      timers_interface_,
      clock_,
      period_,
      std::bind(&Updater::update, this));
  }

  /**
   * \brief Causes the diagnostics to update if the inter-update interval
   * has been exceeded.
   */
  void update()
  {
    if (rclcpp::ok()) {
      bool warn_nohwid = hwid_.empty();

      std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec;

      std::unique_lock<std::mutex> lock(
        lock_);    // Make sure no adds happen while we are processing here.
      const std::vector<DiagnosticTaskInternal> & tasks = getTasks();
      for (std::vector<DiagnosticTaskInternal>::const_iterator iter =
        tasks.begin();
        iter != tasks.end(); iter++)
      {
        diagnostic_updater::DiagnosticStatusWrapper status;

        status.name = iter->getName();
        status.level = 2;
        status.message = "No message was set";
        status.hardware_id = hwid_;

        iter->run(status);

        status_vec.push_back(status);

        if (status.level) {
          warn_nohwid = false;
        }

        if (verbose_ && status.level) {
          RCLCPP_WARN(
            logger_, "Non-zero diagnostic status. Name: '%s', status %i: '%s'",
            status.name.c_str(), status.level, status.message.c_str());
        }
      }

      if (warn_nohwid && !warn_nohwid_done_) {
        std::string error_msg = "diagnostic_updater: No HW_ID was set.";
        error_msg += " This is probably a bug. Please report it.";
        error_msg += " For devices that do not have a HW_ID, set this value to 'none'.";
        error_msg += " This warning only occurs once all diagnostics are OK.";
        error_msg += " It is okay to wait until the device is open before calling setHardwareID.";
        RCLCPP_WARN(logger_, error_msg);
        warn_nohwid_done_ = true;
      }

      publish(status_vec);
    }
  }

  /**
   * Recheck the diagnostic_period on the parameter server. (Cached)
   */

  // TODO(Karsten1987) Follow up PR for eloquent
  // void update_diagnostic_period()
  // {
  //   // rcl_duration_value_t old_period = period_;
  //   // next_time_ = next_time_ +
  //   //   rclcpp::Duration(period_ - old_period);             // Update next_time_
  // }

  /**
   * Publishes a single diagnostic status.
   */
  void publish(diagnostic_msgs::msg::DiagnosticStatus & stat)
  {
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec;
    status_vec.push_back(stat);
    publish(status_vec);
  }

  /**
   * Publishes a vector of diagnostic statuses.
   */
  void publish(std::vector<diagnostic_msgs::msg::DiagnosticStatus> & status_vec)
  {
    for (std::vector<diagnostic_msgs::msg::DiagnosticStatus>::iterator iter =
      status_vec.begin();
      iter != status_vec.end(); iter++)
    {
      iter->name = node_name_ + std::string(": ") + iter->name;
    }
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.status = status_vec;
    msg.header.stamp = rclcpp::Clock().now();
    publisher_->publish(msg);
  }

  /**
   * Causes a placeholder DiagnosticStatus to be published as soon as a
   * diagnostic task is added to the Updater.
   */
  virtual void addedTaskCallback(DiagnosticTaskInternal & task)
  {
    DiagnosticStatusWrapper stat;
    stat.name = task.getName();
    stat.summary(0, "Node starting up");
    publish(stat);
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base_interface_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers_interface_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Duration period_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::Logger logger_;

  std::string hwid_;
  std::string node_name_;
  bool warn_nohwid_done_;
};
}   // namespace diagnostic_updater

#endif  // DIAGNOSTIC_UPDATER__DIAGNOSTIC_UPDATER_HPP_
