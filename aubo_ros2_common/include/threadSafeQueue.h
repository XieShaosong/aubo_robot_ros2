// Copyright 2023 Xie Shaosong
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
