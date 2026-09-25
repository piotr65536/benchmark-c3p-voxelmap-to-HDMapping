#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <nav_msgs/Odometry.h>
#include "pcl/conversions.h"
#include <laszip/laszip_api.h>
#include <fstream> 
#include <iostream>
#include <string>
#include <thread> 
#include <filesystem>
#include <nlohmann/json.hpp>
#include "rosbag/bag.h"
#include "ros/ros.h"
#include <rosbag/view.h>
#include <pcl_ros/point_cloud.h>
#include <rosgraph_msgs/Clock.h>
#include "laz_writer.hpp"
#include "bag_time_translation.hpp"


struct TrajectoryPose
{
    uint64_t timestamp_ns;
    double x_m;
    double y_m;
    double z_m;
    double qw;
    double qx;
    double qy;
    double qz;
    Eigen::Affine3d pose;
    double om_rad;  // Roll (omega)
    double fi_rad;  // Pitch (phi)
    double ka_rad;  // Yaw (kappa)>
};

struct TaitBryanPose
{
    double px;
    double py;
    double pz;
    double om;
    double fi;
    double ka;
};

inline double deg2rad(double deg) {
	return (deg * M_PI) / 180.0;
}

inline double rad2deg(double rad) {
	return (rad * 180.0) / M_PI;
}

inline TaitBryanPose pose_tait_bryan_from_affine_matrix(Eigen::Affine3d m){
	TaitBryanPose pose;

	pose.px = m(0,3);
	pose.py = m(1,3);
	pose.pz = m(2,3);

	if (m(0,2) < 1) {
		if (m(0,2) > -1) {
			//case 1
			pose.fi = asin(m(0,2));
			pose.om = atan2(-m(1,2), m(2,2));
			pose.ka = atan2(-m(0,1), m(0,0));

			return pose;
		}
		else //r02 = −1
		{
			//case 2
			// not a unique solution: thetaz − thetax = atan2 ( r10 , r11 )
			pose.fi = -M_PI / 2.0;
			pose.om = -atan2(m(1,0), m(1,1));
			pose.ka = 0;
			return pose;
		}
	}
	else {
		//case 3
		// r02 = +1
		// not a unique solution: thetaz + thetax = atan2 ( r10 , r11 )
		pose.fi = M_PI / 2.0;
		pose.om = atan2(m(1,0), m(1,1));
		pose.ka = 0.0;
		return pose;
	}

	return pose;
}

namespace fs = std::filesystem;
std::vector<Point3Di> points_global;

std::vector<TrajectoryPose> trajectory;
std::vector<std::vector<TrajectoryPose>> chunks_trajectory;

// void saveOdometryDataToFile(const std::string &filename,
//                              double x, double y, double z,
//                              double qx, double qy, double qz, double qw)
// {
//     std::ofstream file;
//     file.open(filename, std::ios::app); 

//     if (!file.is_open())
//     {
//         std::cerr << "Błąd otwierania pliku!" << std::endl;
//         return;
//     }

//     file << x << "," << y << "," << z << ","
//          << qx << "," << qy << "," << qz << "," << qw << "\n";

//     file.close();
//     std::cout << "Dane zapisane do pliku " << filename << std::endl;
// }


bool save_poses(const std::string file_name, std::vector<Eigen::Affine3d> m_poses, std::vector<std::string> filenames)
{
    std::ofstream outfile;
    outfile.open(file_name);
    if (!outfile.good())
    {
        std::cout << "can not save file: '" << file_name << "'" << std::endl;
        std::cout << "if You can see only '' it means there is no filename assigned to poses, please read manual or contact me januszbedkowski@gmail.com" << std::endl;
        std::cout << "To assign filename to poses please use following two buttons in multi_view_tls_registration_step_2" << std::endl;
        std::cout << "1: update initial poses from RESSO file" << std::endl;
        std::cout << "2: update poses from RESSO file" << std::endl;
        return false;
    }

    outfile << m_poses.size() << std::endl;
    for (size_t i = 0; i < m_poses.size(); i++)
    {
        outfile << filenames[i] << std::endl;
        outfile << m_poses[i](0, 0) << " " << m_poses[i](0, 1) << " " << m_poses[i](0, 2) << " " << m_poses[i](0, 3) << std::endl;
        outfile << m_poses[i](1, 0) << " " << m_poses[i](1, 1) << " " << m_poses[i](1, 2) << " " << m_poses[i](1, 3) << std::endl;
        outfile << m_poses[i](2, 0) << " " << m_poses[i](2, 1) << " " << m_poses[i](2, 2) << " " << m_poses[i](2, 3) << std::endl;
        outfile << "0 0 0 1" << std::endl;
    }
    outfile.close();

    return true;
}

int main(int argc, char **argv)
{

    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " <input_bag> <output_directory>" << std::endl;
        return 1;
    }
    
    const std::string input_bag = argv[1];
    const std::string output_directory = argv[2];
    ros::serialization::Serializer<sensor_msgs::PointCloud2> serializationPointCloud2;
    ros::serialization::Serializer<nav_msgs::Odometry> serializationOdom;

    std::cout << "Processing bag: " << input_bag << std::endl;

    rosbag::Bag bag;
    bag.open(input_bag, rosbag::bagmode::Read);  
    rosbag::View view(bag);

    // Inputs for translating the result into bag time (see bag_time_translation.hpp).
    std::vector<int64_t> clock_wall_minus_bag_ns;  // per /clock message: receive time - payload
    std::vector<int64_t> odom_stamp_minus_receive_ns;  // per odometry message: header stamp - receive time

    for (const rosbag::MessageInstance& m : view) {
        if (m.getTopic() == "/clock") {
            rosgraph_msgs::Clock::ConstPtr clock_msg = m.instantiate<rosgraph_msgs::Clock>();
            if (clock_msg) {
                clock_wall_minus_bag_ns.push_back(static_cast<int64_t>(m.getTime().toNSec()) -
                                                  static_cast<int64_t>(clock_msg->clock.toNSec()));
            }
            continue;
        }

        if (m.getTopic() == "/cloud_registered") {
            ROS_INFO("Received message on topic: /cloud_registered");

            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();

            if (cloud_msg) {
                pcl::PointCloud<pcl::PointXYZ> cloud;
                pcl::fromROSMsg(*cloud_msg, cloud);

                size_t num_points = cloud.points.size();
                if (num_points == 0) {
                    ROS_ERROR("Error: Empty PointCloud2 message!");
                    return 1;
                }

                ROS_INFO("Processing %zu points", num_points);

                sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
                sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
                sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");
    
                for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z)
                {
                    pcl::PointXYZ point;
                    point.x = *iter_x;
                    point.y = *iter_y;
                    point.z = *iter_z;
                
                    cloud.points.push_back(point);
                
                    Point3Di point_global;
                
                    if (cloud_msg->header.stamp.sec != 0 || cloud_msg->header.stamp.nsec != 0)
                    {
                        uint64_t sec_in_ms = static_cast<uint64_t>(cloud_msg->header.stamp.sec) * 1'000'000'000ULL;
                        uint64_t ns_in_ms = static_cast<uint64_t>(cloud_msg->header.stamp.nsec);
                        point_global.timestamp = sec_in_ms + ns_in_ms;
                    }
                
                    point_global.point = Eigen::Vector3d(point.x, point.y, point.z);
                    point_global.intensity = 0;  
                    point_global.index_pose = static_cast<int>(i);
                    point_global.lidarid = 0;
                    point_global.index_point = static_cast<int>(i);
                
                    points_global.push_back(point_global);
                }
                
                ROS_INFO("Processed %zu points!", cloud.points.size());
            }
        }
       
        if (m.getTopic() == "/aft_mapped_to_init") {
            ROS_INFO("Received message on topic: /aft_mapped_to_init");

            nav_msgs::Odometry::ConstPtr odom_msg = m.instantiate<nav_msgs::Odometry>();

            if (!odom_msg) {
                ROS_ERROR("Odometry message deserialization error!");
                return 1;
            }

            double x = odom_msg->pose.pose.position.x;
            double y = odom_msg->pose.pose.position.y;
            double z = odom_msg->pose.pose.position.z;

            double qx = odom_msg->pose.pose.orientation.x;
            double qy = odom_msg->pose.pose.orientation.y;
            double qz = odom_msg->pose.pose.orientation.z;
            double qw = odom_msg->pose.pose.orientation.w;

            TrajectoryPose pose;
            uint64_t sec_in_ms = static_cast<uint64_t>(odom_msg->header.stamp.sec) * 1'000'000'000ULL;
            uint64_t ns_in_ms = static_cast<uint64_t>(odom_msg->header.stamp.nsec);
            pose.timestamp_ns = sec_in_ms + ns_in_ms;
            odom_stamp_minus_receive_ns.push_back(static_cast<int64_t>(pose.timestamp_ns) -
                                                  static_cast<int64_t>(m.getTime().toNSec()));

            pose.x_m = x;
            pose.y_m = y;
            pose.z_m = z;
            pose.qw = qw;
            pose.qx = qx;
            pose.qy = qy;
            pose.qz = qz;

            pose.pose = Eigen::Affine3d::Identity();

            Eigen::Vector3d trans(x, y, z);
            Eigen::Quaterniond q(qw, qx, qy, qz);

            pose.pose.translation() = trans;
            pose.pose.linear() = q.toRotationMatrix();

            // Calculate Tait-Bryan angles using the original function
            TaitBryanPose tb = pose_tait_bryan_from_affine_matrix(pose.pose);
            pose.om_rad = tb.om;
            pose.fi_rad = tb.fi;
            pose.ka_rad = tb.ka;
            // Dodanie pozycji do trajektorii

            trajectory.push_back(pose);

            ROS_INFO("Added position to trajectory: x=%.3f, y=%.3f, z=%.3f", x, y, z);
        }
    }

    // C3P-VoxelMap stamps its outputs with wall-clock time. Its clock is left
    // alone; the result is translated into bag time here, so it can be
    // aligned with the ground truth. Poses and points are shifted by the same
    // constant, which keeps their association below intact.
    const BagTimeShift shift = decide_bag_time_shift(clock_wall_minus_bag_ns, odom_stamp_minus_receive_ns);
    if (shift.apply) {
        for (auto &p : trajectory) {
            p.timestamp_ns = static_cast<uint64_t>(static_cast<int64_t>(p.timestamp_ns) - shift.offset_ns);
        }
        for (auto &pt : points_global) {
            if (pt.timestamp != 0) {
                pt.timestamp -= static_cast<double>(shift.offset_ns);
            }
        }
        std::cout << "Bag time: " << shift.reason << ", shifted by " << shift.offset_ns
                  << " ns (median of " << clock_wall_minus_bag_ns.size() << " /clock messages)" << std::endl;
    } else {
        std::cout << "Bag time: " << shift.reason << std::endl;
    }

    // std::cout << "start transforming point to global coordinate system" << std::endl;

    // for(int i = 0; i < points_global.size(); i++){
    //     //auto lower = std::lower_bound(trajectory.begin(), trajectory.end(), points_global[i].timestamp);
    //     //auto index_pose = std::distance(trajectory.begin(), lower);

    //     if(i % 1000000 == 0){
    //         std::cout << "computed: " << i << " of " << points_global.size() << std::endl;
    //     }
        
    //     auto lower = std::lower_bound(trajectory.begin(), trajectory.end(), points_global[i].timestamp,
    //                                           [](TrajectoryPose lhs, double rhs) -> bool
    //                                           { return lhs.timestamp_ns < rhs; });


    //                                           //std::vector<TrajectoryPose>
    //     int index_pose = std::distance(trajectory.begin(), lower) - 1;
    //     if (index_pose >= 0 && index_pose < trajectory.size())
    //     {
    //         points_global[i].point = trajectory[index_pose].pose * points_global[i].point;
    //     }
    // }
    
                   
    // std::cout << "transforming point to global coordinate system finished" << std::endl;

    std::cout << "start loading pc" << std::endl;
    std::cout << "loading pc finished" << std::endl;

    std::vector<std::vector<Point3Di>> chunks_pc;

    int counter = 0;
    std::vector<Point3Di> chunk;

    for (int i = 0; i < points_global.size(); i++)
    {
        chunk.push_back(points_global[i]);

        if (chunk.size() > 2000000)
        {
            counter++;
            chunks_pc.push_back(chunk);
            chunk.clear();
            std::cout << "adding chunk [" << counter << "]" << std::endl;
        }
    }

    // remaining pc
    std::cout << "reamaining points: " << chunk.size() << std::endl;

    // Keep the remaining points as a final chunk whenever there are any. The
    // old threshold (> 1,000,000) assumed dense per-scan clouds: C3P-VoxelMap
    // publishes a downsampled cloud of every fifth point, so a whole run can
    // stay below it, which left no chunk at all and therefore no trajectory.
    // Same fix as in benchmark-SR-LIO-to-HDMapping.
    if (!chunk.empty())
    {
        chunks_pc.push_back(chunk);
    }

    std::cout << "cleaning points" << std::endl;
    points_global.clear();
    std::cout << "points cleaned" << std::endl;

    /////////////////////////
    std::cout << "start indexing chunks_trajectory" << std::endl;
    chunks_trajectory.resize(chunks_pc.size());

    for (int i = 0; i < trajectory.size(); i++)
    {
        if(i % 1000 == 0){
            std::cout << "computing [" << i + 1 << "] of: " << trajectory.size() << std::endl;
        }
        for (int j = 0; j < chunks_pc.size(); j++)
        {
            if (chunks_pc[j].size() > 0)
            {
                if (trajectory[i].timestamp_ns >= chunks_pc[j][0].timestamp &&
                    trajectory[i].timestamp_ns < chunks_pc[j][chunks_pc[j].size() - 1].timestamp)
                {
                    chunks_trajectory[j].push_back(trajectory[i]);
                }
            }
        }
    }

    for (const auto &trj : chunks_trajectory)
    {
        std::cout << "number of trajectory elements: " << trj.size() << std::endl;
    }

    /////////////////////////
    std::cout << "start transforming chunks_pc to local coordinate system" << std::endl;
    for (int i = 0; i < chunks_pc.size(); i++)
    {
        std::cout << "computing [" << i + 1 << "] of: " << chunks_pc.size() << std::endl;
        // auto m_inv = chunks_trajectory[i][0].
        if (chunks_trajectory[i].size() == 0){
            continue;
        }

        Eigen::Vector3d trans(chunks_trajectory[i][0].x_m, chunks_trajectory[i][0].y_m, chunks_trajectory[i][0].z_m);
        Eigen::Quaterniond q(chunks_trajectory[i][0].qw, chunks_trajectory[i][0].qx, chunks_trajectory[i][0].qy, chunks_trajectory[i][0].qz);

        Eigen::Affine3d first_affine = Eigen::Affine3d::Identity();
        first_affine.translation() = trans;
        first_affine.linear() = q.toRotationMatrix();

        Eigen::Affine3d first_affine_inv = first_affine.inverse();

        for (auto &p : chunks_pc[i])
        {
            p.point = first_affine_inv * p.point;
            // std::cout << p.point << std::endl;
        }
    }

    ////////////////////////
    if (fs::exists(output_directory)) {
        std::cout << "Directory already exists." << std::endl;
    } else {
    
        try {
            if (fs::create_directory(output_directory)) {
                std::cout << "Directory has been created." << std::endl;
            } else {
                std::cerr << "Failed to create directory " << std::endl;
                return 1; 
            }
        } catch (const fs::filesystem_error& e) {
            std::cerr << "Error creating directory: " << e.what() << std::endl;
            return 1; 
        }
    }
    
    fs::path outwd = output_directory;

    Eigen::Vector3d offset(0, 0, 0); // --obliczyc
    int cc = 0;
    for (int i = 0; i < chunks_trajectory.size(); i++)
    {
        for (int j = 0; j < chunks_trajectory[i].size(); j++)
        {
            Eigen::Vector3d trans_curr(chunks_trajectory[i][j].x_m, chunks_trajectory[i][j].y_m, chunks_trajectory[i][j].z_m);
            offset += trans_curr;
            cc++;
        }
    }
    offset /= cc;

    std::vector<Eigen::Affine3d>
        m_poses;
    std::vector<std::string> file_names;

    for (int i = 0; i < chunks_pc.size(); i++)
    {
        if (chunks_pc[i].size() == 0){
            continue;
        }
        if (chunks_trajectory[i].size() == 0)
        {
            continue;
        }

        fs::path path(outwd);
        std::string filename = ("scan_lio_" + std::to_string(i) + ".laz");
        path /= filename;
        std::cout << "saving to: " << path << " number of points: " << chunks_pc[i].size() << std::endl;
        saveLaz(path.string(), chunks_pc[i]);
        file_names.push_back(filename);

        std::string trajectory_filename = ("trajectory_lio_" + std::to_string(i) + ".csv");
        fs::path pathtrj(outwd);
        pathtrj /= trajectory_filename;
        std::cout << "saving to: " << pathtrj << std::endl;

        std::ofstream outfile;
        outfile.open(pathtrj);
        if (!outfile.good())
        {
            std::cout << "can not save file: " << pathtrj << std::endl;
            return 1;
        }


        outfile << "timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 timestampUnix_nanoseconds om_rad fi_rad ka_rad" << std::endl;

        Eigen::Vector3d trans(chunks_trajectory[i][0].x_m, chunks_trajectory[i][0].y_m, chunks_trajectory[i][0].z_m);
        Eigen::Quaterniond q(chunks_trajectory[i][0].qw, chunks_trajectory[i][0].qx, chunks_trajectory[i][0].qy, chunks_trajectory[i][0].qz);

        Eigen::Affine3d first_affine = Eigen::Affine3d::Identity();
        first_affine.translation() = trans;
        first_affine.linear() = q.toRotationMatrix();

        Eigen::Affine3d first_affine_inv = first_affine.inverse();
        m_poses.push_back(first_affine);

        for (int j = 0; j < chunks_trajectory[i].size(); j++)
        {
            Eigen::Vector3d trans_curr(chunks_trajectory[i][j].x_m, chunks_trajectory[i][j].y_m, chunks_trajectory[i][j].z_m);
            Eigen::Quaterniond q_curr(chunks_trajectory[i][j].qw, chunks_trajectory[i][j].qx, chunks_trajectory[i][j].qy, chunks_trajectory[i][j].qz);

            Eigen::Affine3d first_affine_curr = Eigen::Affine3d::Identity();
            first_affine_curr.translation() = trans_curr;
            first_affine_curr.linear() = q_curr.toRotationMatrix();

            auto pose = first_affine_inv * first_affine_curr;
            // auto pose = worker_data_concatenated[i].intermediate_trajectory[0].inverse() * worker_data_concatenated[i].intermediate_trajectory[j];
            outfile
                << chunks_trajectory[i][j].timestamp_ns << " " << std::setprecision(10)

                << pose(0, 0) << " "
                << pose(0, 1) << " "
                << pose(0, 2) << " "
                << pose(0, 3) << " "
                << pose(1, 0) << " "
                << pose(1, 1) << " "
                << pose(1, 2) << " "
                << pose(1, 3) << " "
                << pose(2, 0) << " "
                << pose(2, 1) << " "
                << pose(2, 2) << " "
                << pose(2, 3) << " "
                // Position (x, y, z)
                // << chunks_trajectory[i][j].x_m << " "  // x
                // << chunks_trajectory[i][j].y_m << " "  // y
                // << chunks_trajectory[i][j].z_m << " "  // z
                // << chunks_trajectory[i][j].qw << " "   // qw
                // << chunks_trajectory[i][j].qx << " "   // qx
                // << chunks_trajectory[i][j].qy << " "   // qy
                // << chunks_trajectory[i][j].qz << " "   // qz
                << chunks_trajectory[i][j].timestamp_ns << " "
                << std::setprecision(20)
                << chunks_trajectory[i][j].om_rad << " "
                << chunks_trajectory[i][j].fi_rad << " "
                << chunks_trajectory[i][j].ka_rad << " "
                << std::endl;
        }
        outfile.close();
    }

    for (auto &m : m_poses)
    {
        m.translation() -= offset;
    }

    fs::path path(outwd);
    path /= "lio_initial_poses.reg";
    save_poses(path.string(), m_poses, file_names);
    fs::path path2(outwd);
    path2 /= "poses.reg";
    save_poses(path2.string(), m_poses, file_names);

    fs::path path3(outwd);
    path3 /= "session.json";

    // save session file
    std::cout << "saving file: '" << path3 << "'" << std::endl;

    nlohmann::json jj;
    nlohmann::json j;
    j["offset_x"] = 0.0;
    j["offset_y"] = 0.0;
    j["offset_z"] = 0.0;
    j["folder_name"] = outwd;
    j["out_folder_name"] = outwd;
    j["poses_file_name"] = path2.string();
    j["initial_poses_file_name"] = path.string();
    j["out_poses_file_name"] = path2.string();
    j["lidar_odometry_version"] = "HdMap";

    jj["Session Settings"] = j;

    nlohmann::json jlaz_file_names;
    for (int i = 0; i < chunks_pc.size(); i++)
    {
        if (chunks_pc[i].size() == 0)
        {
            continue;
        }
        if (chunks_trajectory[i].size() == 0)
        {
            continue;
        }
        
        fs::path path(outwd);
        std::string filename = ("scan_lio_" + std::to_string(i) + ".laz");
        path /= filename;
        std::cout << "adding file: " << path << std::endl;

        nlohmann::json jfn{
            {"file_name", path.string()}};
        jlaz_file_names.push_back(jfn);
    }
    jj["laz_file_names"] = jlaz_file_names;

    std::ofstream fs(path3.string());
    fs << jj.dump(2);
    fs.close();

return 0;
}
