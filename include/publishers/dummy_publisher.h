#pragma once

#include <memory>

/**
 * @brief Class that acts as an empty interface
 */
class DummyPublisher {
 public:
  virtual ~DummyPublisher() {}
};

using PublisherPtr = std::shared_ptr<DummyPublisher>;
