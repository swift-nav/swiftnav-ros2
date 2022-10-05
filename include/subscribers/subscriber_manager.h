#pragma once

#include <subscribers/dummy_subscriber.h>
#include <list>

class SubscriberManager {
 public:
  void add(const SubscriberPtr& subscriber) {
    subscribers_.push_back(subscriber);
  }

 private:
  std::list<SubscriberPtr> subscribers_;
};
