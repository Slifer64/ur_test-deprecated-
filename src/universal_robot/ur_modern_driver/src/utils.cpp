#include <ur_modern_driver/utils.h>

namespace ur_
{

int setThreadPriority(std::thread &thr, int policy, int priority)
{
  struct sched_param sch_param;
  int prev_policy;
  pthread_getschedparam(thr.native_handle(), &prev_policy, &sch_param);
  sch_param.sched_priority = priority;
  int ret_code = pthread_setschedparam(thr.native_handle(), policy, &sch_param);

  return ret_code;
}

int makeThreadRT(std::thread &thr)
{
  return setThreadPriority(thr, SCHED_FIFO, 99);
}

std::string setThreadPriorErrMsg(int error_code)
{
  if (error_code == 0) return "";

  switch (error_code)
  {
    case ESRCH:
      return "No thread with the ID thread could be found.";
    case EINVAL:
      return "Policy is not a recognized policy, or param does not make sense for the policy.";
    case EPERM:
      return "The caller does not have appropriate privileges to set the specified scheduling policy and parameters.";
    case ENOTSUP:
      return "Attempt was made to set the policy or scheduling parameters to an unsupported value.";
    default:
      return "Unknown error code.";
  }
}

void PRINT_INFO_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[34m" << "[INFO]: " << msg << "\033[0m";
}

void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out)
{
  std::cout << "\033[1m" << "\033[32m" << "[INFO]: " << msg << "\033[0m";
}

void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out)
{
  std::cout << "\033[1m" << "\033[33m" << "[WARNING]: " << msg << "\033[0m";
}

void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out)
{
  std::cout << "\033[1m" << "\033[31m" << "[ERROR]: " << msg << "\033[0m";
}



JointStatePublisher::JointStatePublisher()
{
  run = false;
}

JointStatePublisher::~JointStatePublisher()
{
  stop();
}

void JointStatePublisher::start()
{
  if (run == true) return;

  joint_state_pub = nh.advertise<sensor_msgs::JointState>(pub_topic, 1);
  run = true;
  run_thread = std::thread(&JointStatePublisher::publishLoop, this);
}

void JointStatePublisher::stop()
{
  run = false;
  if (run_thread.joinable()) run_thread.join();
  joint_state_pub.shutdown();
}

void JointStatePublisher::addFun(void (*fun)(sensor_msgs::JointState &))
{
  std::unique_lock<std::mutex> lck(mtx);
  add_joint_state_funs.push_back( std::bind(fun, std::placeholders::_1) );
}


void JointStatePublisher::setPublishCycle(double Ts)
{
  this->Ts = Ts*1e9;
}

void JointStatePublisher::setPublishTopic(const std::string &pub_topic)
{
  this->pub_topic = pub_topic;
}


void JointStatePublisher::publishLoop()
{
  while (run)
  {
    std::unique_lock<std::mutex> lck(mtx);

    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    for (int i=0; i<add_joint_state_funs.size(); i++) (add_joint_state_funs[i])(joint_state_msg);

    joint_state_pub.publish(joint_state_msg);

    lck.unlock();

    std::this_thread::sleep_for(std::chrono::nanoseconds(Ts));
  }
}

} // namespace ur_