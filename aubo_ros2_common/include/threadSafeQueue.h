#ifndef THREAD_SAFE_QUEUE_H_
#define THREAD_SAFE_QUEUE_H_

#include <queue>
#include <mutex>
#include <condition_variable>

template<typename T>
class ThreadSafeQueue
{
public:
  ThreadSafeQueue() {}

  void push(const T& value)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push(value);
    lock.unlock();
    cond_.notify_one();
  }

  bool pop(T& value)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (queue_.empty())
    {
      cond_.wait(lock);
    }
    
    value = queue_.front();
    queue_.pop();
    lock.unlock();
    return true;
  }

  int size()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    int size = queue_.size();
    lock.unlock();
    return size;
  }

  bool clear()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while(!queue_.empty())
    {
      queue_.pop();
    }
    lock.unlock();
    return true;
  }

private:
  std::queue<T> queue_;
  std::mutex mutex_;
  std::condition_variable cond_;
};

#endif
