#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <thread>
#include "laz_writer.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"

struct TrajectoryPose
{
    double timestamp_ns;
    double x_m;
    double y_m;
    double z_m;
    double qw;
    double qx;
    double qy;
    double qz;
    Eigen::Affine3d pose;
};

namespace fs = std::filesystem;

bool save_poses(const std::string file_name, std::vector<Eigen::Affine3d> m_poses, std::vector<std::string> filenames)
{
    std::ofstream outfile;
    outfile.open(file_name);
    if (!outfile.good())
    {
        std::cout << "can not save file: '" << file_name << "'" << std::endl;
        std::cout << "if You can see only '' it means there is no filename assigned to poses, please read manual or "
                     "contact me januszbedkowski@gmail.com"
                  << std::endl;
        std::cout
            << "To assign filename to poses please use following two buttons in multi_view_tls_registration_step_2"
            << std::endl;
        std::cout << "1: update initial poses from RESSO file" << std::endl;
        std::cout << "2: update poses from RESSO file" << std::endl;
        return false;
    }

    outfile << m_poses.size() << std::endl;
    for (size_t i = 0; i < m_poses.size(); i++)
    {
        outfile << filenames[i] << std::endl;
        outfile << m_poses[i](0, 0) << " " << m_poses[i](0, 1) << " " << m_poses[i](0, 2) << " " << m_poses[i](0, 3)
                << std::endl;
        outfile << m_poses[i](1, 0) << " " << m_poses[i](1, 1) << " " << m_poses[i](1, 2) << " " << m_poses[i](1, 3)
                << std::endl;
        outfile << m_poses[i](2, 0) << " " << m_poses[i](2, 1) << " " << m_poses[i](2, 2) << " " << m_poses[i](2, 3)
                << std::endl;
        outfile << "0 0 0 1" << std::endl;
    }
    outfile.close();

    return true;
}

template <typename T>
bool be_verbose(const T idx)
{
    if (idx % 100 == 0)
    {
        return true;
    }
    return false;
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " <input_bag> <output_directory>" << std::endl;
        return 1;
    }

    const auto& me = rclcpp::get_logger(argv[0]);

    const fs::path input_bag = argv[1];
    const fs::path output_directory = argv[2];
    const fs::path path_session = output_directory / "session.json";
    const fs::path path_poses = output_directory / "glim_poses.reg";

    RCLCPP_INFO(me, "Processing bag:  x=%s", input_bag.c_str());
    rosbag2_cpp::Reader bag;
    bag.open(input_bag);

    using LocalCloud = std::vector<Point3Di>;
    using Clouds = std::vector<LocalCloud>;
    using Trajectory = std::vector<TrajectoryPose>;

    Clouds clouds;
    Trajectory trajectory;

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_point_cloud2;
    rclcpp::Serialization<nav_msgs::msg::Odometry> serialization_odom;
    int cnt_poses = 0, cnt_clouds = 0;
    while (bag.has_next())
    {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = bag.read_next();

        if (msg->topic_name == "/glim_ros/points_corrected")
        {
            if (be_verbose(cnt_clouds))
            {
                RCLCPP_INFO(rclcpp::get_logger("GlimFrame"), "Received message on topic: /glim_ros/points");
            }

            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            const auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            serialization_point_cloud2.deserialize_message(&serialized_msg, cloud_msg.get());

            if (!cloud_msg || cloud_msg->data.empty())
            {
                RCLCPP_WARN(rclcpp::get_logger("GlimFrame"), "Error: Empty PointCloud2 message!");
                continue;
            }
            const size_t num_points = cloud_msg->width * cloud_msg->height;

            if (num_points == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("GlimFrame"), "Processing %zu points", num_points);
                continue;
            }
            sensor_msgs::PointCloud2ConstIterator<const float> iter_x(*cloud_msg, "x");
            sensor_msgs::PointCloud2ConstIterator<const float> iter_y(*cloud_msg, "y");
            sensor_msgs::PointCloud2ConstIterator<const float> iter_z(*cloud_msg, "z");

            LocalCloud& local_cloud = clouds.emplace_back();
            for (size_t pt_idx = 0; pt_idx < num_points; ++pt_idx, ++iter_x, ++iter_y, ++iter_z)
            {
                const uint64_t sec_in_ms = static_cast<uint64_t>(cloud_msg->header.stamp.sec) * 1000ULL;
                const uint64_t ns_in_ms = static_cast<uint64_t>(cloud_msg->header.stamp.nanosec) / 1'000'000ULL;

                Point3Di& point_global = local_cloud.emplace_back();
                point_global.timestamp = sec_in_ms + ns_in_ms;
                point_global.point.x() = *iter_x;
                point_global.point.y() = *iter_y;
                point_global.point.z() = *iter_z;
                point_global.intensity = 0;
                point_global.index_pose = static_cast<int>(pt_idx);
                point_global.lidarid = 0;
                point_global.index_point = static_cast<int>(pt_idx);
            }

            if (be_verbose(cnt_clouds++))
            {
                RCLCPP_INFO(rclcpp::get_logger("GlimFrame"), "Processed %zu points!", num_points);
            }
        }

        if (msg->topic_name == "/glim_ros/odom_corrected")
        {
            if (be_verbose(cnt_poses))
            {
                RCLCPP_INFO(rclcpp::get_logger("GlimOdometry"), "Received message on topic: /glim_ros/odom");
            }

            const auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            serialization_odom.deserialize_message(&serialized_msg, odom_msg.get());

            if (!odom_msg)
            {
                RCLCPP_WARN(rclcpp::get_logger("GlimOdometry"), "Odometry message deserialization error!");
                continue;
            }
            const double x = odom_msg->pose.pose.position.x;
            const double y = odom_msg->pose.pose.position.y;
            const double z = odom_msg->pose.pose.position.z;
            const double qx = odom_msg->pose.pose.orientation.x;
            const double qy = odom_msg->pose.pose.orientation.y;
            const double qz = odom_msg->pose.pose.orientation.z;
            const double qw = odom_msg->pose.pose.orientation.w;
            const uint64_t sec_in_ms = static_cast<uint64_t>(odom_msg->header.stamp.sec) * 1000ULL;
            const uint64_t ns_in_ms = static_cast<uint64_t>(odom_msg->header.stamp.nanosec) / 1'000'000ULL;

            TrajectoryPose& pose = trajectory.emplace_back();
            pose.timestamp_ns = sec_in_ms + ns_in_ms;
            pose.x_m = x;
            pose.y_m = y;
            pose.z_m = z;
            pose.qw = qw;
            pose.qx = qx;
            pose.qy = qy;
            pose.qz = qz;
            pose.pose = Eigen::Affine3d::Identity();
            pose.pose.translation() = Eigen::Vector3d(x, y, z);
            pose.pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();

            if (be_verbose(cnt_poses++))
            {
                RCLCPP_INFO(rclcpp::get_logger("GlimOdometry"), "Added position to trajectory: x=%.3f, y=%.3f, z=%.3f",
                            x, y, z);
            }
        }
    }

    /// if found both cloud and pose then emplace back
    if (clouds.size() != trajectory.size())
    {
        RCLCPP_ERROR(me, "Different number of cloud and odometry topics!");
        return 1;
    }

    ///////////////////////// (apparently glim outputs them in local CS already)
    // std::cout << "start transforming clouds to local coordinate system" << std::endl;
    // for (int i = 0; i < clouds.size(); i++)
    // {
    //     if (i % 100 == 0){
    //         std::cout << "computing [" << i + 1 << "] of: " << clouds.size() << std::endl;
    //     }

    //     Eigen::Vector3d trans(trajectory[i].x_m, trajectory[i].y_m, trajectory[i].z_m);
    //     Eigen::Quaterniond q(trajectory[i].qw, trajectory[i].qx, trajectory[i].qy, trajectory[i].qz);

    //     Eigen::Affine3d first_affine = Eigen::Affine3d::Identity();
    //     first_affine.translation() = trans;
    //     first_affine.linear() = q.toRotationMatrix();

    //     Eigen::Affine3d first_affine_inv = first_affine.inverse();

    //     for (auto &p : clouds[i])
    //     {
    //         p.point = first_affine_inv * p.point;
    //         // std::cout << p.point << std::endl;
    //     }
    // }

    // save clouds aggregate poses
    Eigen::Vector3d trajectory_center(0, 0, 0);
    std::vector<Eigen::Affine3d> m_poses;
    std::vector<std::string> file_names;
    fs::create_directories(output_directory);
    for (size_t cloud_idx = 0; cloud_idx < clouds.size(); cloud_idx++)
    {
        if (clouds[cloud_idx].empty())
        {
            continue;
        }
        const std::string filename = ("scan_glim_" + std::to_string(cloud_idx) + ".laz");
        const fs::path filepath = output_directory / filename;
        if (be_verbose(cloud_idx))
        {
            std::cout << "saving to: " << filepath << " number of points: " << clouds[cloud_idx].size() << std::endl;
        }
        laz::saveLaz(filepath, clouds[cloud_idx], be_verbose(cloud_idx));
        file_names.push_back(filename);
        trajectory_center += trajectory[cloud_idx].pose.translation();
        m_poses.emplace_back(trajectory[cloud_idx].pose);
    }

    trajectory_center /= m_poses.size();
    for (auto& m : m_poses)
    {
        m.translation() -= trajectory_center;
    }
    save_poses(path_poses.string(), m_poses, file_names);

    std::cout << "saving file: '" << path_session << "'" << std::endl;

    nlohmann::json session_json;
    nlohmann::json& j = session_json["Session Settings"];
    j["offset_x"] = 0.0;
    j["offset_y"] = 0.0;
    j["offset_z"] = 0.0;
    j["folder_name"] = output_directory;
    j["out_folder_name"] = output_directory;
    j["poses_file_name"] = path_poses.string();
    j["initial_poses_file_name"] = path_poses.string();
    j["out_poses_file_name"] = path_poses.string();
    j["lidar_odometry_version"] = "HdMap";

    nlohmann::json& laz_file_names_json = session_json["laz_file_names"];
    for (const auto& filename : file_names)
    {
        const fs::path path = output_directory / filename;
        nlohmann::json fn_json{{"file_name", path.string()}};
        laz_file_names_json.push_back(fn_json);
    }

    std::ofstream fs(path_session.string());
    fs << session_json.dump(2);
    fs.close();

    return 0;
}
