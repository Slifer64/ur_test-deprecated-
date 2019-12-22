#include <ur_robot/ur_robot.h>

#include <exception>
#include <iomanip>

#include <ros/package.h>

#include <io_lib/io_lib.h>

#include <ur_kinematics/ur_kin.h>
#include <ur_kinematics/ikfast.h>

using namespace as64_;

namespace ur_
{

  arma::mat quat2rotm(const arma::vec &quat)
  {
    double qw=quat(0), qx=quat(1), qy=quat(2), qz=quat(3);

    arma::mat rotm;
    rotm = {{1 - 2*qy*qy - 2*qz*qz,      2*qx*qy - 2*qz*qw,      2*qx*qz + 2*qy*qw},
  	        {    2*qx*qy + 2*qz*qw,  1 - 2*qx*qx - 2*qz*qz,      2*qy*qz - 2*qx*qw},
  	        {    2*qx*qz - 2*qy*qw,      2*qy*qz + 2*qx*qw,  1 - 2*qx*qx - 2*qy*qy}};

    return rotm;
  }

  Robot::Robot(const std::string &robot_ip, int reverse_port)
  {
    joint_prefix = "";
    err_msg = "";

    parseConfigFile();

    joint_names.push_back(joint_prefix + "shoulder_pan_joint");
    joint_names.push_back(joint_prefix + "shoulder_lift_joint");
    joint_names.push_back(joint_prefix + "elbow_joint");
    joint_names.push_back(joint_prefix + "wrist_1_joint");
    joint_names.push_back(joint_prefix + "wrist_2_joint");
    joint_names.push_back(joint_prefix + "wrist_3_joint");

    this->robot_ip = robot_ip;
    this->reverse_port = reverse_port;

    runUrDriver();

    mode = ur_::Mode::POSITION_CONTROL;

    ros::Duration(4.0).sleep(); // needed to let UR initialize

    waitNextCycle();
  }

  Robot::~Robot()
  {
    shutdown_sem.notify();
    if (ur_driver_thr.joinable()) ur_driver_thr.join();
    jState_pub.stop();
    delete ur_driver;
  }

  void Robot::runUrDriver()
  {
    ur_::Semaphore start_ur_driver_sem;

    ur_driver_thr = std::thread( [this, &start_ur_driver_sem]()
    {
      this->ur_driver = new UrDriver(this->robot_ip, this->reverse_port);
      if (!this->ur_driver->start()) throw std::runtime_error("[ur_::Robot::runUrDriver]: Failed to start the UrDriver...\n");
      start_ur_driver_sem.notify();
    	//ros::waitForShutdown();
      this->shutdown_sem.wait();
    });

    start_ur_driver_sem.wait();

    cycle = ur_driver->getServojTime();

    jState_pub.setPublishCycle(0.0333); // 30 Hz
    jState_pub.setPublishTopic("joint_states");
    jState_pub.addFun(&ur_::Robot::addJointState, this);

    jState_pub.start(); // launches joint states publisher thread
  }

  void Robot::addJointState(sensor_msgs::JointState &joint_state_msg)
  {
    // std::unique_lock<std::mutex> lck(robot_state_mtx);

    arma::vec j_pos = getJointsPosition();
    arma::vec j_vel = getJointsVelocity();

    for (int i=0;i<j_pos.size();i++)
    {
      joint_state_msg.name.push_back(joint_names[i]);
      joint_state_msg.position.push_back(j_pos(i));
      joint_state_msg.velocity.push_back(j_vel(i));
      joint_state_msg.effort.push_back(0.0);
    }
  }

  void Robot::parseConfigFile()
  {
//    std::string params_path = ros::package::getPath("ur_robot") + "/config/ur_config.yml";
//    param_::Parser parser(params_path);

//    if (!parser.getParam("command_ur_topic", command_ur_topic))
//      throw std::ios_base::failure("ur_::Robot::getParam(command_ur_topic) could not be retrieved.\n");
//
//    if (!parser.getParam("read_wrench_topic", read_wrench_topic))
//      throw std::ios_base::failure("ur_::Robot::getParam(read_wrench_topic) could not be retrieved.\n");
//
//    if (!parser.getParam("read_toolVel_topic", read_toolVel_topic))
//      throw std::ios_base::failure("ur_::Robot::getParam(read_toolVel_topic) could not be retrieved.\n");
//
//    if (!parser.getParam("read_jointState_topic", read_jointState_topic))
//      throw std::ios_base::failure("ur_::Robot::getParam(read_jointState_topic) could not be retrieved.\n");
//
//    if (!parser.getParam("base_frame", base_frame))
//      throw std::ios_base::failure("ur_::Robot::getParam(base_frame) could not be retrieved.\n");
//
//    if (!parser.getParam("tool_frame", tool_frame))
//      throw std::ios_base::failure("ur_::Robot::getParam(tool_frame) could not be retrieved.\n");
  }

  void Robot::setMode(const ur_::Mode &mode)
  {
    if (this->getMode() == mode) return;

    switch (mode)
    {
      case ur_::Mode::FREEDRIVE_MODE:
        this->freedrive_mode();
        break;
      case ur_::Mode::FORCE_MODE:
        // this->force_mode();
        break;
      case ur_::Mode::POSITION_CONTROL:
        this->setModeToPosCtrl();
        break;
      case ur_::Mode::VELOCITY_CONTROL:
        this->setModeToVelCtrl();
        break;
    }
    this->mode = mode;
  }

  void Robot::setModeToPosCtrl()
  {
    if (this->getMode() != ur_::Mode::POSITION_CONTROL)
    {
      this->stopj(2.0); // just stop the robot if it was moving
      this->mode = ur_::Mode::POSITION_CONTROL;
    }
  }

  void Robot::setModeToVelCtrl()
  {
    if (this->getMode() != ur_::Mode::VELOCITY_CONTROL)
    {
      this->stopj(2.0); // just stop the robot if it was moving
      this->mode = ur_::Mode::VELOCITY_CONTROL;
    }
  }

  void Robot::freedrive_mode()
  {
    if (this->getMode() != ur_::Mode::FREEDRIVE_MODE)
    {
      command_mode("freedrive_mode()\n");
      this->mode = ur_::Mode::FREEDRIVE_MODE;
    }
  }

  void Robot::end_freedrive_mode()
  {
    ur_driver->setUrScriptCmd("end_freedrive_mode()\n");
    this->setMode(ur_::Mode::POSITION_CONTROL);
  }

  void Robot::force_mode(const arma::vec &task_frame, const arma::vec &selection_vector,
                  const arma::vec &wrench, int type, const arma::vec &limits)
  {
    if (type<1 || type>3) throw std::invalid_argument("[Error]: Robot::force_mode: type must be in {1,2,3}");
    std::ostringstream out;
    out << "force_mode(p" << print_vector(task_frame) << "," << print_vector(selection_vector) << ","
        << print_vector(wrench) << "," << type << "," << print_vector(limits) << ")\n";
    command_mode("sleep(0.02)\n\t" + out.str());
    this->mode = ur_::Mode::FORCE_MODE;
  }

  void Robot::end_force_mode()
  {
    ur_driver->setUrScriptCmd("end_force_mode()\n");
    this->mode = ur_::Mode::POSITION_CONTROL;
  }

  void Robot::force_mode_set_damping(double damping)
  {
    if (damping<0)
    {
      damping = 0.0;
      std::cerr << "[WARNING]: Robot::force_mode_set_damping: Saturating damping to 0.0";
    }

    if (damping>1)
    {
      damping = 1.0;
      std::cerr << "[WARNING]: Robot::force_mode_set_damping: Saturating damping to 1.0";
    }

    std::ostringstream out;
    out << "force_mode_set_damping(" << damping << ")\n";
    ur_driver->setUrScriptCmd(out.str());
  }

  void Robot::movej(const arma::vec &q, double a, double v, double t, double r)
  {
    std::ostringstream out;
    out << "movej(" << print_vector(q) << "," << a << "," << v << "," << t << "," << r << ")\n";
    ur_driver->setUrScriptCmd(out.str());
  }

  void Robot::movel(const arma::vec &p, double a, double v, double t, double r)
  {
    std::ostringstream out;
    out << "movel(p" << print_vector(p) << "," << a << "," << v << "," << t << "," << r << ")\n";
    ur_driver->setUrScriptCmd(out.str());
  }

  void Robot::speedj(arma::vec dq, double a, double t)
  {
    std::ostringstream out;
    out << "speedj(" << print_vector(dq) << "," << a;
    if (t > 0.0) out << "," << t;
    out << ")\n";
    ur_driver->setUrScriptCmd(out.str());
  }

  void Robot::speedl(arma::vec dp, double a, double t)
  {
    std::ostringstream out;
    out << "speedl(" << print_vector(dp) << "," << a;
    if (t > 0.0) out << "," << t;
    out << ")\n";
    ur_driver->setUrScriptCmd(out.str());
  }

  void Robot::stopj(double a)
  {
    std::ostringstream out;
    out << "stopj(" << a << ")\n";
    ur_driver->setUrScriptCmd(out.str());
  }

  void Robot::stopl(double a)
  {
    std::ostringstream out;
    out << "stopl(" << a << ")\n";
    ur_driver->setUrScriptCmd(out.str());
  }

  void Robot::sleep(double t)
  {
    std::ostringstream out;
    out << "sleep(" << t << ")\n";
    ur_driver->setUrScriptCmd(out.str());
  }

  void Robot::powerdown()
  {
    ur_driver->setUrScriptCmd("powerdown()\n");
  }

  void Robot::waitNextCycle()
  {
    ur_driver->update_sem.wait();
  }

  void Robot::load_URScript(const std::string &path_to_URScript)
  {
    try{ io_::readFile(path_to_URScript, ur_script); }
    catch(std::exception &e) { throw std::ios_base::failure(std::string("ur_::Robot::load_URScript: failed to read \""+path_to_URScript+"\"...\n")); }
  }

  void Robot::execute_URScript() const
  {
    ur_driver->setUrScriptCmd(ur_script);
  }

  void Robot::command_mode(const std::string &mode) const
  {
    std::string cmd;
    cmd = "def command_mode():\n\n\t" + mode + "\n\twhile (True):\n\t\tsync()\n\tend\nend\n";
    ur_driver->setUrScriptCmd(cmd);
  }

  void Robot::startLogging()
  {
    ur_driver->log_data_ = true;
  }

  void Robot::stopLogging()
  {
    ur_driver->log_data_ = false;
  }

  void Robot::saveLoggedData(const std::string filename)
  {
    std::ofstream out(filename, std::ios::out | std::ios::binary);
    if (!out) throw std::ios_base::failure("[ur_::Robot::saveLoggedData]: Couldn't create file \"" + filename + "\"...\n");

    ur_driver->time_data.resize(ur_driver->n_data);
    ur_driver->joint_vel_data.resize(6, ur_driver->n_data);
    ur_driver->joint_vel_cmd_data.resize(6, ur_driver->n_data);

    io_::write_mat(ur_driver->time_data, out);
    io_::write_mat(ur_driver->joint_vel_data, out);
    //io_::write_mat(ur_driver->joint_target_vel_data, out);
    io_::write_mat(ur_driver->joint_vel_cmd_data, out);

    out.close();
  }

  // void Robot::setJointsTrajectory(const arma::vec &qT, double duration)
  // {
  //   this->movej(qT, 4.0, 3.5, duration);
  //   ros::Duration(duration).sleep();
  // }

  bool Robot::setJointsTrajectory(const arma::vec &qT, double duration)
  {
    // keep last known robot mode
    Mode prev_mode = this->getMode();
    // start controller
    this->setMode(Mode::VELOCITY_CONTROL);

    // waits for the next tick
    waitNextCycle();

    arma::vec q0 = this->getJointsPosition();
    arma::vec q = q0;
    arma::vec qref = q0;
    arma::vec qref_dot;

    double t = 0.0;
    double click = 0.0;
    // the main while
    while (t < duration)
    {
      if (!isOk())
      {
        std::cerr << getErrMsg();
        return false;
      }

      // compute time now
      t += getCtrlCycle();
      // update trajectory
      arma::mat ref_traj =  get5thOrder(t, q0, qT, duration);
      qref = get5thOrder(t, q0, qT, duration).col(0);
      qref_dot = get5thOrder(t, q0, qT, duration).col(1);

      arma::vec q_dot_cmd = qref_dot + click*(qref-q);

      this->setJointsVelocity(q_dot_cmd);

      // waits for the next tick
      waitNextCycle();

      q = this->getJointsPosition();
    }

    std::cerr << "t = " << t << "\n";
    std::cerr << "qref = " << qref.t() << "\n";
    std::cerr << "q = " << q.t() << "\n";

    // reset last known robot mode
    this->setMode(prev_mode);

    return true;
  }

  arma::mat Robot::get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime)
  {
    arma::mat retTemp = arma::zeros<arma::mat>(p0.n_rows, 3);

    if (t < 0)
    {
      // before start
      retTemp.col(0) = p0;
    }
    else if (t > totalTime)
    {
      // after the end
      retTemp.col(0) = pT;
    }
    else
    {
      // somewhere betweeen ...
      // position
      retTemp.col(0) = p0 +
                       (pT - p0) * (10 * pow(t / totalTime, 3) -
                       15 * pow(t / totalTime, 4) +
                       6 * pow(t / totalTime, 5));
      // vecolity
      retTemp.col(1) = (pT - p0) * (30 * pow(t, 2) / pow(totalTime, 3) -
                       60 * pow(t, 3) / pow(totalTime, 4) +
                       30 * pow(t, 4) / pow(totalTime, 5));
      // acceleration
      retTemp.col(2) = (pT - p0) * (60 * t / pow(totalTime, 3) -
                       180 * pow(t, 2) / pow(totalTime, 4) +
                       120 * pow(t, 3) / pow(totalTime, 5));
    }

    // return vector
    return retTemp;
  }

  void Robot::setJointsPosition(const arma::vec &qd)
  {
    this->movej(qd, 1.4, 1.0, this->cycle);
  }

  void Robot::setJointsVelocity(const arma::vec &dqd)
  {
    ur_driver->joint_vel_cmd = dqd;
    this->speedj(dqd, 6.0, this->cycle);
  }

  void Robot::setTaskPose(const arma::mat &pose)
  {
    const arma::vec p;
    //convertPose2PosAngles(pose, p);
    this->movel(p, 1.2, 1.0, this->cycle);
  }

  void Robot::setTaskVelocity(const arma::vec &Twist)
  {
    this->speedl(Twist, arma::max(arma::abs((Twist-getTaskVelocity()))/this->cycle), this->cycle);
    // this->speedl(Twist, 1.5, this->cycle);
  }

  bool Robot::isOk() const
  {
    if (ur_driver->isEmergencyStopped())
    {
      *(const_cast<std::string *>(&err_msg)) = "EMERGENCY STOP!\n";
      return false;
    }

    if (ur_driver->isProtectiveStopped())
    {
      *(const_cast<std::string *>(&err_msg)) = "PROTECTIVE STOP!\n";
      return false;
    }

    *(const_cast<std::string *>(&err_msg)) = "";

    return true;
  }


} // namespace ur_
