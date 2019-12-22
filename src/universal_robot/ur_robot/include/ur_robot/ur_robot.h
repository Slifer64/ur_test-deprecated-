#ifndef ur_ROBOT_H
#define ur_ROBOT_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <thread>
#include <mutex>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <armadillo>

#include <ur_modern_driver/utils.h>
#include <ur_modern_driver/ur_driver.h>

namespace ur_
{

/**
 * Enumeration defining the control modes of the robot.
 */
enum Mode
{
  STOPPED,          /**< When the robot is stopped and does not accept commands */
  POSITION_CONTROL, /**< For sending position commands */
  VELOCITY_CONTROL, /**< For sending velocity commands */
  FORCE_MODE,        /**< When the robot is in force mode */
  FREEDRIVE_MODE,    /**< When the robot is in freedrive mode */
};

/**
 * UR10 robot class.
 *
 * Implements functions for sending URScript like commands. These functions are executed as
 * unique programs at the actual robot and terminate when the command finishes.
 *
 * It also contains functions that allow to control the robot in a continuous way, sending
 * commands and reading the robot's state in each control cycle.
 *
 */
class Robot
{

  /**
   * Struct with the state of the robot.
   * The state includes the following:
   * - current time stamp.
   * - joint positions, velocities and torques
   * - end-effector pose, twist and wrench
   * - jacobian
   */
  struct RobotState
  {
    uint64_t timestamp_sec;
    uint64_t timestamp_nsec;

    arma::vec q, dq;
    arma::mat pose;
    arma::mat Jrobot;
    arma::vec pos, Q;
    arma::vec v_lin, v_rot;
    arma::vec wrench;
    arma::vec jTorques;

    RobotState()
    {
      q.resize(6);
      dq.resize(6);
      pose.resize(4,4);
      pos.resize(3);
      Q.resize(4);
      v_lin.resize(3);
      v_rot.resize(3);
      wrench.resize(6);
      jTorques.resize(6);
    }
  };

  /**
   * Struct with the data that are logged, which are:
   * - time.
   * - joint positions, velocities and torques
   * - end-effector pose, twist and wrench
   */
  struct LoggedData
  {
    arma::rowvec Time;
    arma::mat q_data, dq_data;
    arma::mat pos_data, Q_data;
    arma::mat V_data;
    arma::mat wrench_data;
    arma::mat jTorques_data;
  };

public:
  /** Constructor. */
  Robot(const std::string &robot_ip="localhost", int reverse_port=50001);

  /** Destructor. */
  ~Robot();

  // ******************************************
  // *******   UR script like commands  *******
  // ******************************************
  // Refer to the URscript language manual to see how these functions work.
  void freedrive_mode();
  void end_freedrive_mode();

  void force_mode(const arma::vec &task_frame, const arma::vec &selection_vector,
                  const arma::vec &wrench, int type, const arma::vec &limits);
  void end_force_mode();
  void force_mode_set_damping(double damping);

  void movej(const arma::vec &q, double a=1.4, double v=1.05, double t=0, double r=0);
  void movel(const arma::vec &p, double a=1.2, double v=0.25, double t=0, double r=0);

  void speedj(arma::vec dq, double a=1.4, double t=-1);
  void speedl(arma::vec dp, double a=1.2, double t=-1);

  void stopj(double a);
  void stopl(double a);

  void sleep(double t);
  void powerdown();

  // ********************************************************
  // *******   General commands to control the robot  *******
  // ********************************************************

  /**
   * Updates the robot's state by reading the actual joint, velocity and force/torque values
   * from the hardware. Must in every control cycle.
   */
  void waitNextCycle();

  /**
   * @return The control cycle of the robot.
   */
  double getCtrlCycle() const { return this->cycle; }

  /** Returns the joint positions.
   * @return 6x1 vector with the joint positions.
   */
  arma::vec getJointsPosition() const { return ur_driver->getJointPos(); }

  /** Returns the robot's end-effector pose relative to the base frame as a 4x4 homogenous transform.
   * @return 4x4 matrix with the robot's pose relative to the base frame.
   */
  arma::mat getTaskPose() const { return rSt.pose; }

  /** Returns the robot's end-effector Cartesian position relative to the base frame as a 3x1 vector.
   * @return 3x1 vector with the robot's Cartesian position relative to the base frame.
   */
  arma::vec getTaskPosition() const { return ur_driver->getTcpPos(); }

  /** Returns the robot's end-effector orientation relative to the base frame as a 4x1 unit quaternion.
   * @return 4x1 vector with the robot's orientation relative to the base frame as a unit quaternion.
   */
  arma::vec getTaskOrientation() const { return ur_driver->getTcpQuat(); }

  /** Returns the robot's joint velocities.
   * @return 6x1 vector with the joint velocities.
   */
  arma::vec getJointsVelocity() const { return ur_driver->getJointVel(); }

  /** Returns the robot's twist relative to the base frame.
   * @return 6x1 vector with robot's twist relative to the base frame.
   */
  arma::vec getTaskVelocity() const { return ur_driver->getTcpVel(); }

  /** Returns the wrench exerted on the end-effector relative to the base frame.
   * @return 6x1 vector with wrench exerted on the end-effector relative to the base frame.
   */
  arma::vec getTaskWrench() const;

  /** Returns the torques exerted on the robot's joints.
   * @return 6x1 vector with the torques exerted on the robot's joints.
   */
  arma::vec getJointsTorque() const { return rSt.jTorques; }

  /** TODO
   */
  arma::mat getJacobian() const { return rSt.Jrobot; }

  /** Sets the robot's control mode.
   * @param[in] mode The control mode to set the robot in.
   */
  void setMode(const ur_::Mode &mode);

  /** Returns the robot's control mode.
   * @return The control mode of the robot.
   */
  ur_::Mode getMode() const { return this->mode; }

  /** Moves the robot from the current configuration to the desired configuration within the specified duration.
   * @param[in] qT 6x1 vector with joint values of the desired configuration.
   * @param[in] duration desired movement duration.
   */
  bool setJointsTrajectory(const arma::vec &qT, double duration);

  /** Sets the robot's joint positions for the next control cycle.
   * @param[in] qd 6x1 vector with the desired joint values.
   */
  void setJointsPosition(const arma::vec &qd);

  /** Sets the robot's joint velocities for the next control cycle.
   * @param[in] dqd 6x1 vector with the desired joint velocity values.
   */
  void setJointsVelocity(const arma::vec &dqd);

  /** Sets the robot's task pose relative to the base for the next control cycle.
   * @param[in] pose 4x4 homogenous transform matrix with robot's desired pose relative to the base.
   */
  void setTaskPose(const arma::mat &pose);

  /** Sets the robot's task velocity relative to the base for the next control cycle.
   * @param[in] pose 6x1 vector with velocity/twist relative to the base frame.
   */
  void setTaskVelocity(const arma::vec &Twist);

  /**
   * @return true if the robot is ok, false if some error occured.
   */
  bool isOk() const { return true; }

  /** Loads a URscript file and stores it on the memory.
   * @param[in] path_to_URScript Absolute or relative path to the UR script file.
   */
  void load_URScript(const std::string &path_to_URScript);

  /** Executes the URscript that is stored on the memory.
   */
  void execute_URScript() const;

  /**
   * @param[out] robotState Struct with robot's state.
   */
  void getRobotState(RobotState &robotState) const;

  /** Enables logging the robot's state in each control cycle.
   */
  void startLogging();

  /** Disables logging the robot's state.
   */
  void stopLogging();

  /** Saves the robot's state data that were logged.
   * @param[in] filename The name of the file where the data will be saved. Can be relative or absolute path.
   * @param[in] binary True to write the data in binary format, false for text format (optional, default = true).
   * param[in] precision Printing precision for text format (optional, default = 7).
   */
  void saveLoggedData(const std::string filename, bool binary=true, int precision=7);


private:

  ur_::JointStatePublisher jState_pub;

  UrDriver *ur_driver;

  void runUrDriver();
  std::thread ur_driver_thr;
  std::string robot_ip;
  int reverse_port;
  ur_::Semaphore shutdown_sem;

  std::string ur_script; ///< string that can store an entire URscript file.

  double cycle; ///< robot's control cycle.

  Mode mode; ///< robot's control mode.

  bool logging_on; ///< flag that if set to true enables logging data in each control cycle.

  RobotState rSt; ///< Robot state struct.
  LoggedData log_data; ///< Log data struct.

  ros::NodeHandle n;

  std::vector<std::string> joint_names;
  std::string joint_prefix;

  /** Prints a vector as a URscript string. */
  std::string print_vector(const arma::vec &v) const
  {
    std::ostringstream out;
    out << "[" << v(0);
    for (int i=1;i<v.size();i++) out << "," << v(i);
    out << "]";

    return out.str();
  }

  /** Parses the config file where the topics from which to read the robot's state are defined. */
  void parseConfigFile();

  /** Sends a urscript command to set the robot in control mode 'mode'. */
  void command_mode(const std::string &mode) const;

  /** Logs the current robot's state. */
  void logDataStep();

  /** Sets the control mode in position control. */
  void position_control_mode();

  /** Sets the control mode in velocity control. */
  void velocity_control_mode();

  static arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime);

  void addJointState(sensor_msgs::JointState &joint_state_msg);
};


} // namespace ur_

#endif
