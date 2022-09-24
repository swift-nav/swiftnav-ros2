#include <publishers/publisher_factory.h>

#include <publishers/angular_rate_publisher.h>
#include <publishers/baseline_heading_publisher.h>
#include <publishers/gnss_time_offset_publisher.h>
#include <publishers/gpsfix_publisher.h>
#include <publishers/imu_aux_publisher.h>
#include <publishers/imu_raw_publisher.h>
#include <publishers/navsatfix_publisher.h>
#include <publishers/odometry_publisher.h>
#include <publishers/orient_euler_publisher.h>
#include <publishers/orient_quat_publisher.h>
#include <publishers/posestamped_publisher.h>
#include <publishers/timereference_publisher.h>
#include <publishers/wheeltick_publisher.h>

PublisherPtr publisherFactory(const Publishers pub_type, sbp::State* state,
                              const std::string& topic_name, rclcpp::Node* node,
                              const LoggerPtr& logger,
                              const std::string& frame) {
  PublisherPtr pub;

  switch (pub_type) {
    case Publishers::AngularRate:
      pub = std::make_shared<AngularRatePublisher>(state, topic_name, node,
                                                   logger, frame);
      break;

    case Publishers::BaselineHeading:
      pub = std::make_shared<BaselineHeadingPublisher>(state, topic_name, node,
                                                       logger, frame);
      break;

    case Publishers::GnssTimeOffset:
      pub = std::make_shared<GnssTimeOffsetPublisher>(state, topic_name, node,
                                                      logger, frame);
      break;

    case Publishers::GpsFix:
      pub = std::make_shared<GPSFixPublisher>(state, topic_name, node, logger,
                                              frame);
      break;

    case Publishers::ImuAux:
      pub = std::make_shared<ImuAuxPublisher>(state, topic_name, node, logger,
                                              frame);
      break;

    case Publishers::ImuRaw:
      pub = std::make_shared<ImuRawPublisher>(state, topic_name, node, logger,
                                              frame);
      break;

    case Publishers::NavSatFix:
      pub = std::make_shared<NavSatFixPublisher>(state, topic_name, node,
                                                 logger, frame);
      break;

    case Publishers::Odometry:
      pub = std::make_shared<OdometryPublisher>(state, topic_name, node, logger,
                                                frame);
      break;

    case Publishers::OrientEuler:
      pub = std::make_shared<OrientEulerPublisher>(state, topic_name, node,
                                                   logger, frame);
      break;

    case Publishers::OrientQuat:
      pub = std::make_shared<OrientQuatPublisher>(state, topic_name, node,
                                                  logger, frame);
      break;

    case Publishers::TimeReference:
      pub = std::make_shared<TimeReferencePublisher>(state, topic_name, node,
                                                     logger, frame);
      break;

    case Publishers::Wheeltick:
      pub = std::make_shared<WheeltickPublisher>(state, topic_name, node,
                                                 logger, frame);
      break;

    case Publishers::PoseStamped:
      pub = std::make_shared<PoseStampedPublisher>(state, topic_name, node,
                                                   logger, frame);
      break;

    default:
      LOG_ERROR(logger, "Publisher id: %d isn't valid",
                static_cast<int>(pub_type));
      break;
  }

  return pub;
}
