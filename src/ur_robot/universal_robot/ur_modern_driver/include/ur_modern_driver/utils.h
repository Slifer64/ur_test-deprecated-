#ifndef UR_ROBOT_UNITS_H
#define UR_ROBOT_UNITS_H

#include <mutex>
#include <thread>
#include <condition_variable>

namespace ur_
{

class Semaphore
{
private:
  std::mutex mutex_;
  std::condition_variable condition_;
  // unsigned long count_ = 0; // Initialized as locked.
  bool count_ = false;  // Initialized as locked.

public:
  void notify()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    // ++count_;
    count_ = true;
    condition_.notify_one();
  }

  void wait()
  {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    while(!count_) // Handle spurious wake-ups.
      condition_.wait(lock);
    // --count_;
    count_ = false;
  }

  bool try_wait()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    if(count_)
    {
      // --count_;
      count_ = false;
      return true;
    }
    return false;
  }
};


class Timer
{
public:
  Timer()
  {
    clock_gettime(CLOCK_REALTIME, &beg_);
  }

  long int elapsedNanoSec()
  {
    clock_gettime(CLOCK_REALTIME, &end_);
    return (end_.tv_sec - beg_.tv_sec)*1000000000 + (end_.tv_nsec - beg_.tv_nsec);
  }

  double elapsedMicroSec()
  {
    return elapsedNanoSec()/1000.0;
  }

  double elapsedMilliSec()
  {
    return elapsedNanoSec()/1000000.0;
  }

  double elapsedSec()
  {
    return elapsedNanoSec()/1000000000.0;
  }

  void start() { clock_gettime(CLOCK_REALTIME, &beg_); }

private:
  timespec beg_, end_;
};


} // namespace ur_

#endif // UR_ROBOT_UNITS_H
