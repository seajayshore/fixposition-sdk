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
 * @brief Fixposition SDK: tests for fpsdk::ros2::bagwriter
 */

/* LIBC/STL */
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/fpl.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/ros1.hpp>
#include <fpsdk_ros2/bagwriter.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::ros2::bagwriter;

std::string Trim(const std::string& str)
{
    const auto first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
}

template <typename MsgT>
std::vector<uint8_t> SerializeRos1Message(const MsgT& msg)
{
    const uint32_t msg_size = ros::serialization::serializationLength(msg);
    std::vector<uint8_t> msg_data(msg_size);
    ros::serialization::OStream stream(msg_data.data(), msg_size);
    ros::serialization::serialize(stream, msg);
    return msg_data;
}

struct TopicInfo
{
    std::string type;
    int message_count = 0;
};

std::map<std::string, TopicInfo> ParseBagMetadata(const std::string& bag_path)
{
    std::ifstream in(bag_path + "/metadata.yaml");
    if (!in.is_open()) {
        return {};
    }

    std::map<std::string, TopicInfo> topics;
    std::string pending_topic;
    std::string line;
    while (std::getline(in, line)) {
        const auto trimmed = Trim(line);

        if (trimmed.rfind("name:", 0) == 0) {
            pending_topic = Trim(trimmed.substr(5));
            continue;
        }

        if (!pending_topic.empty() && (trimmed.rfind("type:", 0) == 0)) {
            topics[pending_topic].type = Trim(trimmed.substr(5));
            continue;
        }

        if (!pending_topic.empty() && (trimmed.rfind("message_count:", 0) == 0)) {
            std::istringstream iss(Trim(trimmed.substr(14)));
            iss >> topics[pending_topic].message_count;
            pending_topic.clear();
        }
    }

    return topics;
}

std::string MakeTempBagPath()
{
    std::string templ = "/tmp/fpsdk_ros2_bagwriter_test_XXXXXX";
    std::vector<char> buf(templ.begin(), templ.end());
    buf.push_back('\0');
    const char* path = mkdtemp(buf.data());
    if (path == nullptr) {
        return "";
    }
    return std::string(path);
}

bool WriteSingleOdometryToBag(
    const std::string& bag_path, const std::string& topic_name, const nav_msgs::Odometry& ros1_odom)
{
    BagWriter bag;
    if (!bag.Open(bag_path, 0)) {
        return false;
    }

    fpsdk::common::fpl::RosMsgDef rosmsgdef;
    rosmsgdef.valid_ = true;
    rosmsgdef.topic_name_ = topic_name;
    rosmsgdef.msg_name_ = ros::message_traits::datatype<nav_msgs::Odometry>();
    bag.AddMsgDef(rosmsgdef);

    fpsdk::common::fpl::RosMsgBin rosmsgbin;
    rosmsgbin.valid_ = true;
    rosmsgbin.topic_name_ = topic_name;
    rosmsgbin.rec_time_ = fpsdk::common::time::RosTime(1777674228, 880539595);
    rosmsgbin.msg_data_ = SerializeRos1Message(ros1_odom);

    const bool ok = bag.WriteMessage(rosmsgbin);
    bag.Close();
    return ok;
}

TEST(BagwriterTest, DerivesNavSatFixForEcefOdometry)
{
    const std::string bag_path = MakeTempBagPath();
    ASSERT_FALSE(bag_path.empty());

    nav_msgs::Odometry odom;
    odom.header.frame_id = "ecef";
    odom.header.stamp.sec = 1777674228;
    odom.header.stamp.nsec = 880539595;
    odom.pose.pose.position.x = 4281234.5;
    odom.pose.pose.position.y = 634567.8;
    odom.pose.pose.position.z = 4678912.3;
    odom.pose.covariance[0] = 1.0;
    odom.pose.covariance[1] = 0.1;
    odom.pose.covariance[2] = 0.2;
    odom.pose.covariance[7] = 2.0;
    odom.pose.covariance[8] = 0.3;
    odom.pose.covariance[14] = 3.0;

    ASSERT_TRUE(WriteSingleOdometryToBag(bag_path, "/user_io/out/poi_odometry", odom));

    const auto topics = ParseBagMetadata(bag_path);
    const auto odom_it = topics.find("/user_io/out/poi_odometry");
    ASSERT_NE(odom_it, topics.end());
    EXPECT_EQ(odom_it->second.type, "nav_msgs/msg/Odometry");
    EXPECT_EQ(odom_it->second.message_count, 1);

    const auto fix_it = topics.find("/user_io/out/poi_navsatfix");
    ASSERT_NE(fix_it, topics.end());
    EXPECT_EQ(fix_it->second.type, "sensor_msgs/msg/NavSatFix");
    EXPECT_EQ(fix_it->second.message_count, 1);

    const auto pose_it = topics.find("/user_io/out/poi_pose");
    ASSERT_NE(pose_it, topics.end());
    EXPECT_EQ(pose_it->second.type, "geometry_msgs/msg/PoseStamped");
    EXPECT_EQ(pose_it->second.message_count, 1);

#if defined(FPSDK_HAVE_FOXGLOVE_MSGS)
    const auto locationfix_it = topics.find("/user_io/out/poi_locationfix");
    ASSERT_NE(locationfix_it, topics.end());
    EXPECT_EQ(locationfix_it->second.type, "foxglove_msgs/msg/LocationFix");
    EXPECT_EQ(locationfix_it->second.message_count, 1);
#endif

    std::filesystem::remove_all(bag_path);
}

TEST(BagwriterTest, DoesNotDeriveNavSatFixForNonEcefOdometry)
{
    const std::string bag_path = MakeTempBagPath();
    ASSERT_FALSE(bag_path.empty());

    nav_msgs::Odometry odom;
    odom.header.frame_id = "enu";
    odom.header.stamp.sec = 1777674228;
    odom.header.stamp.nsec = 880539595;
    odom.pose.pose.position.x = 10.0;
    odom.pose.pose.position.y = -5.0;
    odom.pose.pose.position.z = 2.0;

    ASSERT_TRUE(WriteSingleOdometryToBag(bag_path, "/user_io/out/poi_odometry", odom));

    const auto topics = ParseBagMetadata(bag_path);
    const auto odom_it = topics.find("/user_io/out/poi_odometry");
    ASSERT_NE(odom_it, topics.end());
    EXPECT_EQ(odom_it->second.type, "nav_msgs/msg/Odometry");
    EXPECT_EQ(odom_it->second.message_count, 1);

    const auto fix_it = topics.find("/user_io/out/poi_navsatfix");
    EXPECT_EQ(fix_it, topics.end());

    const auto pose_it = topics.find("/user_io/out/poi_pose");
    EXPECT_EQ(pose_it, topics.end());

#if defined(FPSDK_HAVE_FOXGLOVE_MSGS)
    const auto locationfix_it = topics.find("/user_io/out/poi_locationfix");
    EXPECT_EQ(locationfix_it, topics.end());
#endif

    std::filesystem::remove_all(bag_path);
}

/* ****************************************************************************************************************** */
}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto level = fpsdk::common::logging::LoggingLevel::WARNING;
    for (int ix = 0; ix < argc; ix++) {
        if ((argv[ix][0] == '-') && argv[ix][1] == 'v') {
            level++;
        }
    }
    fpsdk::common::logging::LoggingSetParams(level);
    return RUN_ALL_TESTS();
}
