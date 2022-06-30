#pragma once

#include <publishers/dummy_publisher.h>
#include <list>

class PublisherManager {
 public:
  void add(const PublisherPtr& publisher) { publishers_.push_back(publisher); }

 private:
  std::list<PublisherPtr> publishers_;
};
