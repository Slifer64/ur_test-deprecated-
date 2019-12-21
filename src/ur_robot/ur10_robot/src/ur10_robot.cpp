#include <ur10_robot/ur10_robot.h>

#include <exception>
#include <iomanip>

#include <ros/package.h>

#include <io_lib/io_lib.h>

#include <ur_kinematics/ur_kin.h>
#include <ur_kinematics/ikfast.h>


namespace ur10_
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

  Robot::Robot(const std::string &robot_ip, int reverse_port):spinner(0)
  {
    this->robot_ip = robot_ip;
    this->reverse_port = reverse_port;
    runUrDriver();

    parseConfigFile();

    this->pub2ur10 = n.advertise<std_msgs::String>(this->command_ur10_topic, 1);

    wrench_sub = n.subscribe(this->read_wrench_topic, 1, &Robot::readWrenchCallback, this);
    toolVel_sub = n.subscribe(this->read_toolVel_topic, 1, &Robot::readToolVelCallback, this);
    jointState_sub = n.subscribe(this->read_jointState_topic, 1, &Robot::readJointStateCallback, this);

    this->tfListener.reset(new tf2_ros::TransformListener(this->tfBuffer));

    mode = ur10_::Mode::POSITION_CONTROL;

    logging_on = false;
    cycle = 0.008; // control cycle of 8 ms

    ros::Duration(4.0).sleep(); // needed to let UR initialize

    waitNextCycle();
    time_offset = rSt.timestamp_sec;

  }

  Robot::~Robot()
  {
    shutdown_sem.notify();
    if (ur_driver_thr.joinable()) ur_driver_thr.join();
  }

  void Robot::runUrDriver()
  {
    ur_::Semaphore start_ur_driver_sem;

    ur_driver_thr = std::thread( [this, &start_ur_driver_sem]()
    {
    	ur_::RosWrapper interface(this->robot_ip, this->reverse_port);
      start_ur_driver_sem.notify();
    	//ros::waitForShutdown();
      this->shutdown_sem.wait();
    });

    start_ur_driver_sem.wait();
  }

  void Robot::parseConfigFile()
  {
    std::string params_path = ros::package::getPath("ur10_robot") + "/config/ur10_config.yml";
    param_::Parser parser(params_path);

    if (!parser.getParam("command_ur10_topic", command_ur10_topic))
      throw std::ios_base::failure("ur10_::Robot::getParam(command_ur10_topic) could not be retrieved.\n");

    if (!parser.getParam("read_wrench_topic", read_wrench_topic))
      throw std::ios_base::failure("ur10_::Robot::getParam(read_wrench_topic) could not be retrieved.\n");

    if (!parser.getParam("read_toolVel_topic", read_toolVel_topic))
      throw std::ios_base::failure("ur10_::Robot::getParam(read_toolVel_topic) could not be retrieved.\n");

    if (!parser.getParam("read_jointState_topic", read_jointState_topic))
      throw std::ios_base::failure("ur10_::Robot::getParam(read_jointState_topic) could not be retrieved.\n");

    if (!parser.getParam("base_frame", base_frame))
      throw std::ios_base::failure("ur10_::Robot::getParam(base_frame) could not be retrieved.\n");

    if (!parser.getParam("tool_frame", tool_frame))
      throw std::ios_base::failure("ur10_::Robot::getParam(tool_frame) could not be retrieved.\n");
  }

  void Robot::setMode(const ur10_::Mode &mode)
  {
    if (this->getMode() == mode) return;

    switch (mode)
    {
      case ur10_::Mode::FREEDRIVE_MODE:
        this->freedrive_mode();
        break;
      case ur10_::Mode::FORCE_MODE:
        // this->force_mode();
        break;
      case ur10_::Mode::POSITION_CONTROL:
        this->position_control_mode();
        break;
      case ur10_::Mode::VELOCITY_CONTROL:
        this->velocity_control_mode();
        break;
    }
    this->mode = mode;
  }

  void Robot::position_control_mode()
  {
    if (this->getMode() != ur10_::Mode::POSITION_CONTROL)
    {
      this->stopj(2.0); // just stop the robot if it was moving
      this->mode = ur10_::Mode::POSITION_CONTROL;
    }
  }

  void Robot::velocity_control_mode()
  {
    if (this->getMode() != ur10_::Mode::VELOCITY_CONTROL)
    {
      this->stopj(2.0); // just stop the robot if it was moving
      this->mode = ur10_::Mode::VELOCITY_CONTROL;
    }
  }

  void Robot::freedrive_mode()
  {
    if (this->getMode() != ur10_::Mode::FREEDRIVE_MODE)
    {
      command_mode("freedrive_mode()\n");
      this->mode = ur10_::Mode::FREEDRIVE_MODE;
    }
  }

  void Robot::end_freedrive_mode()
  {
    urScript_command("end_freedrive_mode()\n");
    this->setMode(ur10_::Mode::POSITION_CONTROL);
  }

  void Robot::teach_mode()
  {
    command_mode("teach_mode()\n");
    this->mode = ur10_::Mode::FREEDRIVE_MODE;
  }

  void Robot::end_teach_mode()
  {
    urScript_command("end_teach_mode()\n");
    this->mode = ur10_::Mode::POSITION_CONTROL;
  }

  void Robot::force_mode(const arma::vec &task_frame, const arma::vec &selection_vector,
                  const arma::vec &wrench, int type, const arma::vec &limits)
  {
    if (type<1 || type>3) throw std::invalid_argument("[Error]: Robot::force_mode: type must be in {1,2,3}");
    std::ostringstream out;
    out << "force_mode(p" << print_vector(task_frame) << "," << print_vector(selection_vector) << ","
        << print_vector(wrench) << "," << type << "," << print_vector(limits) << ")\n";
    command_mode("sleep(0.02)\n\t" + out.str());
    this->mode = ur10_::Mode::FORCE_MODE;
  }

  void Robot::end_force_mode()
  {
    urScript_command("end_force_mode()\n");
    this->mode = ur10_::Mode::POSITION_CONTROL;
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
    urScript_command(out.str());
  }

  void Robot::movej(const arma::vec &q, double a, double v, double t, double r) const
  {
    std::ostringstream out;
    out << "movej(" << print_vector(q) << "," << a << "," << v << "," << t << "," << r << ")\n";
    urScript_command(out.str());
  }

  void Robot::movel(const arma::vec &p, double a, double v, double t, double r) const
  {
    std::ostringstream out;
    out << "movel(p" << print_vector(p) << "," << a << "," << v << "," << t << "," << r << ")\n";
    urScript_command(out.str());
  }

  void Robot::speedj(arma::vec dq, double a, double t) const
  {
    std::ostringstream out;
    out << "speedj(" << print_vector(dq) << "," << a;
    if (t > 0.0) out << "," << t;
    out << ")\n";
    urScript_command(out.str());
  }

  void Robot::speedl(arma::vec dp, double a, double t) const
  {
    std::ostringstream out;
    out << "speedl(" << print_vector(dp) << "," << a;
    if (t > 0.0) out << "," << t;
    out << ")\n";
    urScript_command(out.str());
  }

  void Robot::stopj(double a) const
  {
    std::ostringstream out;
    out << "stopj(" << a << ")\n";
    urScript_command(out.str());
  }

  void Robot::stopl(double a) const
  {
    std::ostringstream out;
    out << "stopl(" << a << ")\n";
    urScript_command(out.str());
  }

  void Robot::set_gravity(const arma::vec &g) const
  {
    std::ostringstream out;
    out << "set_gravity(" << print_vector(g) << ")\n";
    urScript_command(out.str());
  }

  void Robot::set_payload(double m, const arma::vec &CoG) const
  {
    std::ostringstream out;
    out << "set_payload(" << m << "," << print_vector(CoG) << ")\n";
    urScript_command(out.str());
  }

  void Robot::set_payload_cog(const arma::vec &CoG) const
  {
    std::ostringstream out;
    out << "set_payload_cog(" << print_vector(CoG) << ")\n";
    urScript_command(out.str());
  }

  void Robot::set_payload_mass(double m) const
  {
    std::ostringstream out;
    out << "set_payload_mass(" << m << ")\n";
    urScript_command(out.str());
  }

  void Robot::set_tcp(const arma::vec &pose) const
  {
    std::ostringstream out;
    out << "set_tcp(p" << print_vector(pose) << ")\n";
    urScript_command(out.str());
  }

  void Robot::sleep(double t) const
  {
    std::ostringstream out;
    out << "sleep(" << t << ")\n";
    urScript_command(out.str());
  }

  void Robot::powerdown() const
  {
    urScript_command("powerdown()\n");
  }

  void Robot::waitNextCycle()
  {
    if (!timer_start)
    {
      timer_start = true;
      timer.tic();
    }

    std::mutex robotState_mtx;
    std::unique_lock<std::mutex> robotState_lck(robotState_mtx);

    ros::spinOnce();
    // spinner.start();
    this->readTaskPoseCallback();
    // spinner.stop();

    // std::cout << "===> Robot::waitNextCycle(): elapsed time = " << timer.toc()*1e3 << " ms\n";

    if (logging_on) logDataStep();

    int elapsed_time = timer.toc()*1000000000;
    timer_start = false;
    if (elapsed_time<8000000) std::this_thread::sleep_for(std::chrono::nanoseconds(8000000-elapsed_time));
  }

  void Robot::readWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    rSt.wrench << msg->wrench.force.x << msg->wrench.force.y << msg->wrench.force.z
              << msg->wrench.torque.x << msg->wrench.torque.y << msg->wrench.torque.z;
  }

  void Robot::readToolVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
  {
    rSt.v_lin << msg->twist.linear.x  << msg->twist.linear.y  << msg->twist.linear.z;
    rSt.v_rot << msg->twist.angular.x  << msg->twist.angular.y  << msg->twist.angular.z;
  }

  void Robot::readJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    int len = msg->position.size();
    rSt.q.resize(len);
    rSt.dq.resize(len);
    rSt.jTorques.resize(len);
    for (int i=0;i<len; i++)
    {
      rSt.q(i) = msg->position[i];
      rSt.dq(i) = msg->velocity[i];
      rSt.jTorques(i) = msg->effort[i];
    }

  }

  void Robot::readTaskPoseCallback()
  {
    try{
       this->transformStamped = tfBuffer.lookupTransform(this->base_frame, this->tool_frame, ros::Time(0));
       rSt.pos << this->transformStamped.transform.translation.x
               << this->transformStamped.transform.translation.y
               << this->transformStamped.transform.translation.z;

       rSt.Q << this->transformStamped.transform.rotation.w
             << this->transformStamped.transform.rotation.x
             << this->transformStamped.transform.rotation.y
             << this->transformStamped.transform.rotation.z;

       rSt.pose.submat(0,0,2,2) = quat2rotm(rSt.Q);
       rSt.pose.submat(0,3,2,3) = rSt.pos;
       rSt.pose.row(3) = arma::rowvec({0, 0, 0, 1});

       rSt.timestamp_sec = this->transformStamped.header.stamp.sec;
       rSt.timestamp_nsec = this->transformStamped.header.stamp.nsec;

     }
     catch (tf2::TransformException &ex) {
       ROS_WARN("%s",ex.what());
       // ros::Duration(1.0).sleep();
     }

  }

  void Robot::load_URScript(const std::string &path_to_URScript)
  {
    try{ io_::readFile(path_to_URScript, ur_script); }
    catch(std::exception &e) { throw std::ios_base::failure(std::string("ur10_::Robot::load_URScript: failed to read \""+path_to_URScript+"\"...\n")); }
  }

  void Robot::execute_URScript() const
  {
    urScript_command(this->ur_script);
  }

  void Robot::getRobotState(RobotState &robotState) const
  {
    robotState = this->rSt;
  }

  void Robot::command_mode(const std::string &mode) const
  {
    std::string cmd;
    cmd = "def command_mode():\n\n\t" + mode + "\n\twhile (True):\n\t\tsync()\n\tend\nend\n";
    //std::cout << "cmd=\n" << cmd << "\n";
    urScript_command(cmd);
  }

  void Robot::startLogging()
  {
    logging_on = true;
  }

  void Robot::stopLogging()
  {
    logging_on = false;
  }

  void Robot::saveLoggedData(const std::string filename, bool binary, int precision)
  {
    std::ofstream out(filename, std::ios::out);
    if (!out) throw std::ios_base::failure("Couldn't create file \"" + filename + "\"...\n");

    io_::write_mat(log_data.Time, out, binary, precision);
    io_::write_mat(log_data.q_data, out, binary, precision);
    io_::write_mat(log_data.dq_data, out, binary, precision);
    io_::write_mat(log_data.pos_data, out, binary, precision);
    io_::write_mat(log_data.Q_data, out, binary, precision);
    io_::write_mat(log_data.V_data, out, binary, precision);
    io_::write_mat(log_data.wrench_data, out, binary, precision);
    io_::write_mat(log_data.jTorques_data, out, binary, precision);

    out.close();
  }

  void Robot::logDataStep()
  {
    log_data.Time = arma::join_horiz(log_data.Time, arma::mat({getTime()}));
    log_data.q_data = arma::join_horiz(log_data.q_data, getJointPosition());
    log_data.dq_data = arma::join_horiz(log_data.dq_data, getJointVelocity());
    log_data.pos_data = arma::join_horiz(log_data.pos_data, getTaskPosition());
    log_data.Q_data = arma::join_horiz(log_data.Q_data, getTaskOrientation());
    log_data.V_data = arma::join_horiz(log_data.V_data, getTaskVelocity());
    log_data.wrench_data = arma::join_horiz(log_data.wrench_data, getTaskWrench());
    log_data.jTorques_data = arma::join_horiz(log_data.jTorques_data, getJointTorque());
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

    arma::vec q0 = this->getJointPosition();
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
        std::cerr << "An error occured on the robot!\n";
        return false;
      }

      // compute time now
      t += getControlCycle();
      // update trajectory
      arma::mat ref_traj =  get5thOrder(t, q0, qT, duration);
      qref = get5thOrder(t, q0, qT, duration).col(0);
      qref_dot = get5thOrder(t, q0, qT, duration).col(1);

      arma::vec q_dot_cmd = qref_dot + click*(qref-q);

      this->setJointVelocity(q_dot_cmd);

      // waits for the next tick
      waitNextCycle();

      q = this->getJointPosition();
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

  void Robot::setJointPosition(const arma::vec &qd)
  {
    timer_start = true;
    timer.tic();
    this->movej(qd, 1.4, 1.0, this->cycle);
  }

  void Robot::setJointVelocity(const arma::vec &dqd)
  {
    timer_start = true;
    timer.tic();
    this->speedj(dqd, 6.0, this->cycle);
  }

  void Robot::setTaskPose(const arma::mat &pose)
  {
    timer_start = true;
    timer.tic();
    const arma::vec p;
    //convertPose2PosAngles(pose, p);
    this->movel(p, 1.2, 1.0, this->cycle);
  }

  void Robot::setTaskVelocity(const arma::vec &Twist)
  {
    timer_start = true;
    timer.tic();
    this->speedl(Twist, arma::max(arma::abs((Twist-getTaskVelocity()))/this->cycle), this->cycle);
    // this->speedl(Twist, 1.5, this->cycle);
  }

  arma::vec Robot::getTaskWrench() const
  {
    arma::mat T_robot_ee = this->getTaskPose();
    arma::vec wrench(6);

    wrench.subvec(0,2) = T_robot_ee.submat(0,0,2,2)*rSt.wrench.subvec(0,2);
    wrench.subvec(3,5) = T_robot_ee.submat(0,0,2,2)*rSt.wrench.subvec(3,5);

    return wrench;
  }

} // namespace ur10_
