#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "msgs_septentrio/LrmGps.h"

#include <signal.h>

#include "writeMatlabFile.h"

std::ofstream output_vo;
bool first_vo;

void mySigintHandler(int sig);
void callback_gpsVO(const msgs_septentrio::LrmGps::ConstPtr &msg);
void callback_odometryVO(const nav_msgs::Odometry::ConstPtr &msg);
void callback_poseStampedVO(const geometry_msgs::PoseStamped::ConstPtr &msg);
void callback_poseCovarStampedVO(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        ROS_ERROR("Invalid number of arguments. Please specify:\n\t- Output file path (including file name)\n\t- Variable name\n\t\n\t- Topic type.\n\tAcceptable topic types:\n\t\t- gps\n\t\t- odometry\n\t\t- pose\n\t\t- poseCovar");
        return -1;
    }
    
    ros::init(argc, argv, "get_vo_node");
    ros::NodeHandle node;
    
    ros::Subscriber sub_vo;
    
    std::string topicname_vo = "get_data/visual_odometry";
    
    std::string topictype_vo(argv[3]);

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
    else if(topictype_vo == "poseCovar")
    {
        sub_vo = node.subscribe(topicname_vo, 1, callback_poseCovarStampedVO);
    }
    else
    {
        ROS_ERROR("Invalid visual odometry topic type!\n\tAcceptable topic types:\n\t\t- gps\n\t\t- odometry\n\t\t- pose");
        return -1;
    }
    
    std::string output_path(argv[1]);    
    output_vo.open(output_path);
    
    beginFile(&output_vo, std::string(argv[2]), first_vo);
    
    signal(SIGINT, mySigintHandler);
    
    ros::spin();    
    return 0; 
}

void mySigintHandler(int sig)
{
    endFile(&output_vo);
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

void callback_poseCovarStampedVO(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    std::cout << "received pose msg" << std::endl;
    writePoseCovarStamped(&output_vo, first_vo, msg);
}