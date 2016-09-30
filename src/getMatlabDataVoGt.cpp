#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "msgs_septentrio/LrmGps.h"
#include "msgs_raw_imu/RawIMU.h"

#include <signal.h>

#include "writeMatlabFile.h"

std::ofstream output_vo, output_gt, output_imu;
bool first_vo, first_gt, first_imu;

void mySigintHandler(int sig);
void callback_gpsVO(const msgs_septentrio::LrmGps::ConstPtr &msg);
void callback_odometryVO(const nav_msgs::Odometry::ConstPtr &msg);
void callback_poseStampedVO(const geometry_msgs::PoseStamped::ConstPtr &msg);
void callback_gpsGT(const msgs_septentrio::LrmGps::ConstPtr &msg);
void callback_odometryGT(const nav_msgs::Odometry::ConstPtr &msg);
void callback_poseStampedGT(const geometry_msgs::PoseStamped::ConstPtr &msg);
void callback_IMU(const msgs_raw_imu::RawIMU::ConstPtr &msg);

int main(int argc, char **argv)
{
    if(argc < 6)
    {
        ROS_ERROR("Invalid number of arguments. Please specify:\n\t- Output file path\n\t- Experience name\n\t- Visual odometry topic type\n\t- Ground truth topic type\n\tAcceptable topic types:\n\t\t- gps\n\t\t- odometry\n\t\t- pose");
        return -1;
    }
    
    ros::init(argc, argv, "get_vogt_node");
    ros::NodeHandle node;
    
    ros::Subscriber sub_vo, sub_gt, sub_imu;

    ros::NodeHandle nh("~");
    std::string topicname_vo, topicname_gt, topicname_imu;
    nh.param<std::string>("topicname_vo", topicname_vo, "get_data/imu");
    nh.param<std::string>("topicname_gps", topicname_gt, "get_data/gps");
    nh.param<std::string>("topicname_imu", topicname_imu, "get_data/imu");

    std::string topictype_vo(argv[3]);
    std::string topictype_gt(argv[4]);

    if(topictype_vo == "gps")
    {
        sub_vo = node.subscribe(topicname_vo, 1, callback_gpsVO);
    }
    else if(topictype_vo == "odometry")
    {
        sub_vo = node.subscribe(topicname_vo, 1, callback_odometryVO);
    }
    else if(topictype_vo == "pose")
    {
        sub_vo = node.subscribe(topicname_vo, 1, callback_poseStampedVO);
    }
    else
    {
        ROS_ERROR("Invalid visual odometry topic type!\n\tAcceptable topic types:\n\t\t- gps\n\t\t- odometry\n\t\t- pose");
        return -1;
    }
    
    if(topictype_gt == "gps")
    {
        sub_gt = node.subscribe(topicname_gt, 1, callback_gpsGT);
    }
    else if(topictype_gt == "odometry")
    {
        sub_gt = node.subscribe(topicname_gt, 1, callback_odometryGT);
    }
    else if(topictype_gt == "pose")
    {
        sub_gt = node.subscribe(topicname_gt, 1, callback_poseStampedGT);
    }
    else
    {
        ROS_ERROR("Invalid ground truth topic type!\n\tAcceptable topic types:\n\t\t- gps\n\t\t- odometry\n\t\t- pose");
        return -1;
    }

    sub_imu = node.subscribe(topicname_imu, 1, callback_IMU);
    
    std::string output_path(argv[1]);
    
    std::string full_output_path_vo = output_path + "/vo_" + std::string(argv[2]) + "_data.m";
    std::string full_output_path_gt = output_path + "/gt_" + std::string(argv[2]) + "_data.m";
    std::string full_output_path_imu = output_path + "/imu_" + std::string(argv[2]) + "_data.m";
    output_vo.open(full_output_path_vo.c_str());
    output_gt.open(full_output_path_gt.c_str());
    output_imu.open(full_output_path_imu.c_str());
    
    std::cout << output_path+"/vo.m" << std::endl;
    
    beginFile(&output_vo, "vo_" + std::string(argv[2]), first_vo);
    beginFile(&output_gt, "gt_" + std::string(argv[2]), first_gt);
    beginFile(&output_imu, "imu_" + std::string(argv[2]), first_imu);
    
    signal(SIGINT, mySigintHandler);
    
    ros::spin();    
    return 0; 
}

void mySigintHandler(int sig)
{
    endFile(&output_vo);
    endFile(&output_gt);
    endFile(&output_imu);
    
    ros::shutdown();
}

void callback_gpsVO(const msgs_septentrio::LrmGps::ConstPtr &msg)
{
    writeGPS(&output_vo, first_vo, msg);
}

void callback_odometryVO(const nav_msgs::Odometry::ConstPtr &msg)
{
    writeOdometry(&output_vo, first_vo, msg);
}

void callback_poseStampedVO(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    writePoseStamped(&output_vo, first_vo, msg);
}

void callback_gpsGT(const msgs_septentrio::LrmGps::ConstPtr &msg)
{
    writeGPS(&output_gt, first_gt, msg);
}

void callback_odometryGT(const nav_msgs::Odometry::ConstPtr &msg)
{
    writeOdometry(&output_gt, first_gt, msg);
}

void callback_poseStampedGT(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    writePoseStamped(&output_gt, first_gt, msg);
}

void callback_IMU(const msgs_raw_imu::RawIMU::ConstPtr &msg)
{
    writeIMU(&output_imu, first_imu, msg);
}

