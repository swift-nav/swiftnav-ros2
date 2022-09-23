#include <publishers/publisher_factory.h>

#include <publishers/GPSFixPublisher.h>
#include <publishers/NavSatFixPublisher.h>
#include <publishers/TimeReferencePublisher.h>
#include <publishers/angular_rate_publisher.h>
#include <publishers/baseline_heading_publisher.h>
#include <publishers/gnss_time_offset_publisher.h>
#include <publishers/imu_aux_publisher.h>
#include <publishers/imu_raw_publisher.h>
#include <publishers/odometry_publisher.h>
#include <publishers/orient_euler_publisher.h>
#include <publishers/orient_quat_publisher.h>
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

      ImuAux,             //  5
          ImuRaw,         //  6
          NavSatFix,      //  7
          Odometry,       //  8
          OrientEuler,    //  9
          OrientQuat,     // 10
          TimeReference,  // 11
          Wheeltick       // 12
  }

  return pub;
}
