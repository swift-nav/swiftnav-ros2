#pragma once

#include <memory>

/**
 * @brief Class that acts as an empty interface
 */
class DummySubscriber {
 public:
  virtual ~DummySubscriber() {}
};

using SubscriberPtr = std::shared_ptr<DummySubscriber>;
