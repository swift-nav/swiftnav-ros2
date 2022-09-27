#include <subscribers/subscriber_factory.h>

#include <subscribers/imu_subscriber.h>
#include <subscribers/odometry_subscriber.h>

SubscriberPtr subscriberFactory(const Subscribers sub_type, sbp::State* state,
                                const std::string& topic_name,
                                rclcpp::Node* node, const LoggerPtr& logger) {
  SubscriberPtr sub;

  switch (sub_type) {
    case Subscribers::Imu:
      sub = std::make_shared<IMUSubscriber>(node, state, topic_name, logger);
      break;

    case Subscribers::Odometry:
      sub =
          std::make_shared<OdometrySubscriber>(node, state, topic_name, logger);
      break;

    default:
      LOG_ERROR(logger, "Subscriber id: %d isn't valid",
                static_cast<int>(sub_type));
      break;
  }

  return sub;
}
