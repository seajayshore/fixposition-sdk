/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: ROS2 bag writer
 */

/* LIBC/STL */
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

/* EXTERNAL */
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include "fpsdk_ros2/ext/msgs.hpp"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#if defined(FPSDK_HAVE_OPENCV)
#  include <opencv2/core/mat.hpp>
#  include <opencv2/imgcodecs.hpp>
#  include <opencv2/imgproc.hpp>
#endif
#if defined(FPSDK_HAVE_FOXGLOVE_MSGS)
#  include <foxglove_msgs/msg/location_fix.hpp>
#endif

/* Fixposition SDK */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/ros1.hpp>
#include <fpsdk_common/trafo.hpp>

/* PACKAGE */
#include "fpsdk_ros2/bagwriter.hpp"
#include "fpsdk_ros2/ros1.hpp"
#include "fpsdk_ros2/utils.hpp"

namespace fpsdk {
namespace ros2 {
namespace bagwriter {
/* ****************************************************************************************************************** */

BagWriter::BagWriter()
{
}

BagWriter::~BagWriter()
{
    Close();
}

void BagWriter::SetImageExportOptions(const ImageExportFormat format, const int jpeg_quality)
{
    image_export_format_ = format;
    image_jpeg_quality_ = std::clamp(jpeg_quality, 1, 100);
}

// ---------------------------------------------------------------------------------------------------------------------

bool BagWriter::Open(const std::string& path, const int compress)
{
    Close();
    bag_ = std::make_unique<rosbag2_cpp::Writer>();
    rosbag2_storage::StorageOptions opts;
    opts.uri = path;
    if (compress > 0) {
        opts.storage_id = "mcap";  // apt install ros-${ROS_DISTRO}-rosbag2-storage-mcap
        opts.storage_preset_profile = (compress > 1 ? "zstd_small" : "zstd_fast");
    } else {
        opts.storage_id = "sqlite3";
    }
    try {
        bag_->open(opts);
    } catch (const std::exception& ex) {
        WARNING("BagWriter: open fail %s: %s. Maybe %s storage plugin is not installed?", path.c_str(), ex.what(),
            opts.storage_id.c_str());
        bag_.reset();
        return false;
    }
    DEBUG("BagWriter: %s", path.c_str());

    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

void BagWriter::Close()
{
    if (bag_) {
        bag_->close();
        bag_.reset();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void BagWriter::AddMsgDef(const common::fpl::RosMsgDef& rosmsgdef)
{
    if (rosmsgdef.valid_ && (defs_.find(rosmsgdef.topic_name_) == defs_.end())) {
        DEBUG("BagWriter: %s", rosmsgdef.info_.c_str());
        defs_[rosmsgdef.topic_name_] = rosmsgdef;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

// Forward declare helper for NavSatFix derivation from ECEF odometry
static bool DeriveNavSatFix(const nav_msgs::msg::Odometry& odom, const std::string& src_topic,
                           sensor_msgs::msg::NavSatFix& fix, std::string& fix_topic);

// Forward declare helper for WGS84 pose + ENU quaternion derivation from ECEF odometry
static bool DeriveWgs84Pose(const nav_msgs::msg::Odometry& odom, const std::string& src_topic,
    geometry_msgs::msg::PoseStamped& pose, std::string& pose_topic);

// Forward declare helper for LocationFix derivation from PoseStamped and NavSatFix
#if defined(FPSDK_HAVE_FOXGLOVE_MSGS)
static bool DeriveLocationFix(const std::string& src_topic, const geometry_msgs::msg::PoseStamped& pose,
    const sensor_msgs::msg::NavSatFix& navsatfix, foxglove_msgs::msg::LocationFix& locationfix,
    std::string& locationfix_topic);
#endif

template <typename Ros1MsgT, typename Ros2MsgT>
inline bool WriteMessageEx(
    const common::fpl::RosMsgDef& rosmsgdef, const common::fpl::RosMsgBin& rosmsgbin, ros2::bagwriter::BagWriter& bag)
{
    if (rosmsgdef.msg_name_ == ros::message_traits::datatype<Ros1MsgT>()) {
        Ros1MsgT ros1;
        common::ros1::DeserializeMessage(rosmsgbin.msg_data_, ros1);
        Ros2MsgT ros2;
        ros2::ros1::Ros1ToRos2(ros1, ros2);
        bag.WriteMessage(ros2, rosmsgbin.topic_name_, rosmsgbin.rec_time_);
        return true;
    } else {
        return false;
    }
}

static std::string MakeCompressedImageTopic(const std::string& src_topic)
{
    return src_topic + "_compressed";
}

#if defined(FPSDK_HAVE_OPENCV)
static bool EncodeJpegImage(
    const sensor_msgs::msg::Image& image, const int jpeg_quality, sensor_msgs::msg::CompressedImage& compressed)
{
    int cv_type = -1;
    int color_code = -1;
    if (image.encoding == sensor_msgs::image_encodings::MONO8) {
        cv_type = CV_8UC1;
    } else if (image.encoding == sensor_msgs::image_encodings::MONO16) {
        cv_type = CV_16UC1;
    } else if (image.encoding == sensor_msgs::image_encodings::BGR8) {
        cv_type = CV_8UC3;
    } else if (image.encoding == sensor_msgs::image_encodings::BGRA8) {
        cv_type = CV_8UC4;
    } else if (image.encoding == sensor_msgs::image_encodings::RGB8) {
        cv_type = CV_8UC3;
        color_code = cv::COLOR_RGB2BGR;
    } else if (image.encoding == sensor_msgs::image_encodings::RGBA8) {
        cv_type = CV_8UC4;
        color_code = cv::COLOR_RGBA2BGR;
    } else {
        return false;
    }

    const std::size_t expected_size = static_cast<std::size_t>(image.height) * static_cast<std::size_t>(image.step);
    if (image.data.size() < expected_size) {
        return false;
    }

    const cv::Mat src(static_cast<int>(image.height), static_cast<int>(image.width), cv_type,
        const_cast<unsigned char*>(image.data.data()), static_cast<std::size_t>(image.step));
    cv::Mat encode_input;
    if (color_code >= 0) {
        cv::cvtColor(src, encode_input, color_code);
    } else {
        encode_input = src;
    }

    compressed.header = image.header;
    compressed.format = "jpeg";
    const std::vector<int> params{ cv::IMWRITE_JPEG_QUALITY, std::clamp(jpeg_quality, 1, 100) };
    return cv::imencode(".jpg", encode_input, compressed.data, params);
}
#endif

bool WriteImageMessage(const common::fpl::RosMsgDef& rosmsgdef, const common::fpl::RosMsgBin& rosmsgbin,
    ros2::bagwriter::BagWriter& bag, const BagWriter::ImageExportFormat image_export_format, const int jpeg_quality)
{
    if (rosmsgdef.msg_name_ != ros::message_traits::datatype<sensor_msgs::Image>()) {
        return false;
    }

    sensor_msgs::Image ros1_image;
    common::ros1::DeserializeMessage(rosmsgbin.msg_data_, ros1_image);
    sensor_msgs::msg::Image ros2_image;
    ros2::ros1::Ros1ToRos2(ros1_image, ros2_image);

    if (image_export_format == BagWriter::ImageExportFormat::RAW) {
        bag.WriteMessage(ros2_image, rosmsgbin.topic_name_, rosmsgbin.rec_time_);
        return true;
    }

#if defined(FPSDK_HAVE_OPENCV)
    sensor_msgs::msg::CompressedImage compressed_image;
    if (EncodeJpegImage(ros2_image, jpeg_quality, compressed_image)) {
        bag.WriteMessage(compressed_image, MakeCompressedImageTopic(rosmsgbin.topic_name_), rosmsgbin.rec_time_);
        return true;
    }
    WARNING_THR(1000, "BagWriter: failed to JPEG-encode image on %s, writing raw image", rosmsgbin.topic_name_.c_str());
#else
    WARNING_THR(1000, "BagWriter: JPEG image export requested for %s but OpenCV support is unavailable, writing raw image",
        rosmsgbin.topic_name_.c_str());
#endif

    bag.WriteMessage(ros2_image, rosmsgbin.topic_name_, rosmsgbin.rec_time_);
    return true;
}

// Derive NavSatFix (WGS84 lat/lon/alt) from ECEF odometry message
static bool DeriveNavSatFix(const nav_msgs::msg::Odometry& odom, const std::string& src_topic,
                           sensor_msgs::msg::NavSatFix& fix, std::string& fix_topic)
{
    using namespace fpsdk::common::trafo;

    // Extract position from odometry (ECEF coordinates, in meters)
    const double x = odom.pose.pose.position.x;
    const double y = odom.pose.pose.position.y;
    const double z = odom.pose.pose.position.z;
    const double pos_mag_sq = x * x + y * y + z * z;

    // ECEF positions are > 6.3e6 m from origin; ENU positions are near origin
    if (pos_mag_sq < 1e12) {  // sqrt(1e12) = 1e6 meters: not ECEF
        return false;
    }

    // Convert ECEF to WGS84 LLH (latitude, longitude in radians; height in meters)
    const Eigen::Vector3d ecef_pos(x, y, z);
    const Eigen::Vector3d llh_rad = TfWgs84LlhEcef(ecef_pos);
    const Eigen::Vector3d llh_deg = LlhRadToDeg(llh_rad);

    // Build NavSatFix header (copy from odometry)
    fix.header = odom.header;

    // Set latitude, longitude, altitude
    fix.latitude = llh_deg(0);
    fix.longitude = llh_deg(1);
    fix.altitude = llh_deg(2);

    // Status: set to STATUS_FIX (RTK quality not encoded in nav_msgs::Odometry)
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;  // GNSS-based

    // Covariance: rotate from ECEF to ENU frame
    // Extract 3x3 position covariance block from odometry (top-left of pose.covariance)
    Eigen::Matrix3d cov_ecef;
    cov_ecef(0, 0) = odom.pose.covariance[0];    // XX
    cov_ecef(1, 1) = odom.pose.covariance[7];    // YY
    cov_ecef(2, 2) = odom.pose.covariance[14];   // ZZ
    cov_ecef(0, 1) = cov_ecef(1, 0) = odom.pose.covariance[1];  // XY
    cov_ecef(0, 2) = cov_ecef(2, 0) = odom.pose.covariance[2];  // XZ
    cov_ecef(1, 2) = cov_ecef(2, 1) = odom.pose.covariance[8];  // YZ

    // Rotation matrix from ECEF to ENU
    const Eigen::Matrix3d R_enu_ecef = RotEnuEcef(ecef_pos);

    // Rotate covariance: C_enu = R * C_ecef * R^T
    const Eigen::Matrix3d cov_enu = R_enu_ecef * cov_ecef * R_enu_ecef.transpose();

    // Fill NavSatFix covariance (row-major: E, N, U order)
    fix.position_covariance[0] = cov_enu(0, 0);  // EE
    fix.position_covariance[1] = cov_enu(0, 1);  // EN
    fix.position_covariance[2] = cov_enu(0, 2);  // EU
    fix.position_covariance[3] = cov_enu(1, 0);  // NE
    fix.position_covariance[4] = cov_enu(1, 1);  // NN
    fix.position_covariance[5] = cov_enu(1, 2);  // NU
    fix.position_covariance[6] = cov_enu(2, 0);  // UE
    fix.position_covariance[7] = cov_enu(2, 1);  // UN
    fix.position_covariance[8] = cov_enu(2, 2);  // UU
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;

    // Topic naming: require "_odometry" suffix and replace it with "_navsatfix".
    // This guarantees the derived NavSatFix is never written onto the original odometry topic.
    const std::string odometry_suffix = "_odometry";
    if (src_topic.length() <= odometry_suffix.length() ||
        src_topic.compare(src_topic.length() - odometry_suffix.length(), odometry_suffix.length(), odometry_suffix) !=
            0) {
        return false;
    }
    fix_topic = src_topic;
    fix_topic.replace(fix_topic.length() - odometry_suffix.length(), odometry_suffix.length(), "_navsatfix");

    return true;
}

// Derive WGS84 position (lat/lon/alt) and ENU quaternion orientation from ECEF odometry message
static bool DeriveWgs84Pose(const nav_msgs::msg::Odometry& odom, const std::string& src_topic,
    geometry_msgs::msg::PoseStamped& pose, std::string& pose_topic)
{
    using namespace fpsdk::common::trafo;

    const double x = odom.pose.pose.position.x;
    const double y = odom.pose.pose.position.y;
    const double z = odom.pose.pose.position.z;
    const double pos_mag_sq = x * x + y * y + z * z;
    if (pos_mag_sq < 1e12) {  // sqrt(1e12) = 1e6 meters: not ECEF
        return false;
    }

    const std::string odometry_suffix = "_odometry";
    if (src_topic.length() <= odometry_suffix.length() ||
        src_topic.compare(src_topic.length() - odometry_suffix.length(), odometry_suffix.length(), odometry_suffix) !=
            0) {
        return false;
    }
    pose_topic = src_topic;
    pose_topic.replace(pose_topic.length() - odometry_suffix.length(), odometry_suffix.length(), "_pose");

    const Eigen::Vector3d ecef_pos(x, y, z);
    const Eigen::Vector3d llh_deg = LlhRadToDeg(TfWgs84LlhEcef(ecef_pos));

    pose.header = odom.header;
    // Temporary representation requested by plan: x=lat [deg], y=lon [deg], z=alt [m].
    pose.pose.position.x = llh_deg.x();
    pose.pose.position.y = llh_deg.y();
    pose.pose.position.z = llh_deg.z();

    const Eigen::Quaterniond q_ecef_body(
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
    if (q_ecef_body.norm() <= std::numeric_limits<double>::epsilon()) {
        return false;
    }
    const Eigen::Matrix3d ecef_rot_matrix = q_ecef_body.normalized().toRotationMatrix();
    const Eigen::Matrix3d rot_enu_ecef = RotEnuEcef(ecef_pos);
    const Eigen::Matrix3d rot_enu_body = rot_enu_ecef * ecef_rot_matrix;
    const Eigen::Quaterniond q_enu_body(rot_enu_body);

    pose.pose.orientation.x = q_enu_body.x();
    pose.pose.orientation.y = q_enu_body.y();
    pose.pose.orientation.z = q_enu_body.z();
    pose.pose.orientation.w = q_enu_body.w();

    return true;
}

// Derive LocationFix from already-derived PoseStamped (orientation) and NavSatFix (position/covariance)
#if defined(FPSDK_HAVE_FOXGLOVE_MSGS)
static bool DeriveLocationFix(const std::string& src_topic, const geometry_msgs::msg::PoseStamped& pose,
    const sensor_msgs::msg::NavSatFix& navsatfix, foxglove_msgs::msg::LocationFix& locationfix,
    std::string& locationfix_topic)
{
    const std::string odometry_suffix = "_odometry";
    if (src_topic.length() <= odometry_suffix.length() ||
        src_topic.compare(src_topic.length() - odometry_suffix.length(), odometry_suffix.length(), odometry_suffix) !=
            0) {
        return false;
    }
    locationfix_topic = src_topic;
    locationfix_topic.replace(
        locationfix_topic.length() - odometry_suffix.length(), odometry_suffix.length(), "_locationfix");

    locationfix.timestamp = pose.header.stamp;
    locationfix.frame_id = pose.header.frame_id;

    // Position matches NavSatFix (WGS84 lat/lon/alt).
    locationfix.latitude = navsatfix.latitude;
    locationfix.longitude = navsatfix.longitude;
    locationfix.altitude = navsatfix.altitude;

    for (int i = 0; i < 9; i++) {
        locationfix.position_covariance[i] = navsatfix.position_covariance[i];
    }
    locationfix.position_covariance_type = navsatfix.position_covariance_type;

    // Extract ENU yaw from quaternion, then convert to compass heading.
    const auto& q = pose.pose.orientation;
    const double yaw_enu = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    double heading = M_PI_2 - yaw_enu;
    while (heading < 0.0) {
        heading += 2.0 * M_PI;
    }
    while (heading >= 2.0 * M_PI) {
        heading -= 2.0 * M_PI;
    }
    locationfix.heading = heading;

    return true;
}
#endif

bool BagWriter::WriteMessage(const common::fpl::RosMsgBin& rosmsgbin)
{
    if (!rosmsgbin.valid_) {
        return false;
    }

    const auto& entry = defs_.find(rosmsgbin.topic_name_);
    if (entry == defs_.end()) {
        WARNING("BagWriter: missing message definition for %s", rosmsgbin.topic_name_.c_str());

        WARNING_THR(1000, "Missing ROSMSGDEF for ROSMSGBIN %s", rosmsgbin.info_.c_str());
        return false;
    }

    const auto& rosmsgdef = entry->second;

    // For ROS2 we have to instantiate the ROS1 message in order to be able to convert it to the corresponding ROS2
    // message type, which includes the message meta data and definition. This can then be written to the bag. Only some
    // conversions are implemented.
    try {
        if (WriteImageMessage(rosmsgdef, rosmsgbin, *this, image_export_format_, image_jpeg_quality_) ||
            WriteMessageEx<sensor_msgs::Imu, sensor_msgs::msg::Imu>(rosmsgdef, rosmsgbin, *this) ||
            WriteMessageEx<sensor_msgs::Temperature, sensor_msgs::msg::Temperature>(rosmsgdef, rosmsgbin, *this) ||
            WriteMessageEx<nav_msgs::Odometry, nav_msgs::msg::Odometry>(rosmsgdef, rosmsgbin, *this) ||
            WriteMessageEx<tf2_msgs::TFMessage, tf2_msgs::msg::TFMessage>(rosmsgdef, rosmsgbin, *this)) {
            // After writing Odometry, also derive NavSatFix if possible (Phase 2 plan)
            if (rosmsgdef.msg_name_ == ros::message_traits::datatype<nav_msgs::Odometry>()) {
                nav_msgs::Odometry ros1_odom;
                common::ros1::DeserializeMessage(rosmsgbin.msg_data_, ros1_odom);
                nav_msgs::msg::Odometry ros2_odom;
                ros2::ros1::Ros1ToRos2(ros1_odom, ros2_odom);

                sensor_msgs::msg::NavSatFix navsatfix;
                std::string navsatfix_topic;
                if (DeriveNavSatFix(ros2_odom, rosmsgbin.topic_name_, navsatfix, navsatfix_topic) &&
                    (navsatfix_topic != rosmsgbin.topic_name_)) {
                    WriteMessage(navsatfix, navsatfix_topic, rosmsgbin.rec_time_);
                }

                geometry_msgs::msg::PoseStamped wgs84_pose;
                std::string wgs84_pose_topic;
                if (DeriveWgs84Pose(ros2_odom, rosmsgbin.topic_name_, wgs84_pose, wgs84_pose_topic) &&
                    (wgs84_pose_topic != rosmsgbin.topic_name_)) {
                    WriteMessage(wgs84_pose, wgs84_pose_topic, rosmsgbin.rec_time_);

#if defined(FPSDK_HAVE_FOXGLOVE_MSGS)
                    foxglove_msgs::msg::LocationFix locationfix;
                    std::string locationfix_topic;
                    if (DeriveLocationFix(rosmsgbin.topic_name_, wgs84_pose, navsatfix, locationfix, locationfix_topic) &&
                        (locationfix_topic != rosmsgbin.topic_name_)) {
                        WriteMessage(locationfix, locationfix_topic, rosmsgbin.rec_time_);
                    }
#endif
                }
            }
        } else {
            throw std::runtime_error("conversion not implemented");
        }
    } catch (std::exception& ex) {
        WARNING("BagWriter: write fail: %s %s", rosmsgdef.msg_name_.c_str(), ex.what());
        return false;
    }

    return true;
}

/* ****************************************************************************************************************** */
}  // namespace bagwriter
}  // namespace ros2
}  // namespace fpsdk
