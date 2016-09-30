
#include "writeMatlabFile.h"

void beginFile(std::ofstream *fd, std::string variableName, bool &firstRow)
{
    (*fd) << variableName << " = [" << std::endl;
    firstRow = true;
}

void endFile(std::ofstream *fd)
{
    (*fd) << "\n\t];" << std::endl;
    (*fd).close();
}

void writeOdometry(std::ofstream *fd, bool &firstRow, const nav_msgs::Odometry::ConstPtr &msg)
{
    std::cout.precision(std::numeric_limits<double>::max_digits10);    
    
    double val_tx, val_ty, val_tz, val_rx, val_ry, val_rz;
    getOdometryData(msg, val_tx, val_ty, val_tz, val_rx, val_ry, val_rz);

    double timestamp = msg->header.stamp.toSec();
    
    int camIndex = -1;
    if((msg->child_frame_id).length() == 14)
    {
		if((msg->child_frame_id)[12] >= '0' && (msg->child_frame_id)[12] <= '9' && (msg->child_frame_id)[13] >= '0' && (msg->child_frame_id)[13] <= '9')
		{
			char camNumber[3];
			camNumber[0] = (msg->child_frame_id)[12];
			camNumber[1] = (msg->child_frame_id)[13];
			camNumber[2] = '\0';
			camIndex = atoi(camNumber);            
		}
	}

    if(!firstRow)
    {
        (*fd) << ";" << std::endl;
               
    }
    else
    {
        firstRow = false; 
    }

    (*fd) << "\t" << camIndex << ", " << std::fixed << timestamp << ", " << std::fixed << val_tx << ", " << std::fixed << val_ty << ", " << std::fixed << val_tz << ", " << std::fixed << val_rx << ", " << std::fixed << val_ry << ", " << std::fixed << val_rz;
    (*fd).flush();    
}

void writePoseStamped(std::ofstream *fd, bool &firstRow, const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    std::cout.precision(std::numeric_limits<double>::max_digits10);
    
    double val_tx, val_ty, val_tz, val_rx, val_ry, val_rz;
    getPoseStampedData(msg, val_tx, val_ty, val_tz, val_rx, val_ry, val_rz);
    
    double timestamp = msg->header.stamp.toSec();
    
    if(!firstRow)
    {
        (*fd) << ";" << std::endl;
    }
    else
    {
        firstRow = false;  
    }
    
    (*fd) << "\t" << std::fixed << timestamp << ", " << std::fixed << val_tx << ", " << std::fixed << val_ty << ", " << std::fixed << val_tz << ", " << std::fixed << val_rx << ", " << std::fixed << val_ry << ", " << std::fixed << val_rz;
    (*fd).flush();    
}

void writeGPS(std::ofstream *fd, bool &firstRow, const msgs_septentrio::LrmGps::ConstPtr &msg)
{
    std::cout.precision(std::numeric_limits<double>::max_digits10);
    
    double val_tx, val_ty, val_tz, val_rx, val_ry, val_rz;
    getGPSData(msg, val_tx, val_ty, val_tz, val_rx, val_ry, val_rz);
    
    double timestamp = msg->header.stamp.toSec();
    
    if(!firstRow)
    {
        (*fd) << ";" << std::endl;
    }
    else
    {
        firstRow = false; 
    }
    
    (*fd) << "\t" << std::fixed << timestamp << ", " << std::fixed << val_tx << ", " << std::fixed << val_ty << ", " << std::fixed << val_tz << ", " << std::fixed << val_rx << ", " << std::fixed << val_ry << ", " << std::fixed << val_rz;
    (*fd).flush(); 
}

void writePoseCovarStamped(std::ofstream *fd, bool &firstRow, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    std::cout.precision(std::numeric_limits<double>::max_digits10);
    
    double val_tx, val_ty, val_tz, val_rx, val_ry, val_rz;
    getPoseCovarStampedData(msg, val_tx, val_ty, val_tz, val_rx, val_ry, val_rz);
    
    double timestamp = msg->header.stamp.toSec();
    
    if(!firstRow)
    {
        (*fd) << ";" << std::endl;
    }
    else
    {
        firstRow = false; 
    }
    
    (*fd) << "\t" << std::fixed << timestamp << ", " << std::fixed << val_tx << ", " << std::fixed << val_ty << ", " << std::fixed << val_tz << ", " << std::fixed << val_rx << ", " << std::fixed << val_ry << ", " << std::fixed << val_rz;
    (*fd).flush(); 
}

void writeIMU(std::ofstream *fd, bool &firstRow, const msgs_raw_imu::RawIMU::ConstPtr &msg)
{
    std::cout.precision(std::numeric_limits<double>::max_digits10);
    double val_rx, val_ry, val_rz;
    getIMUData(msg, val_rx, val_ry, val_rz);

    double timestamp = msg->header.stamp.toSec();

    if(!firstRow)
    {
        (*fd) << ";" << std::endl;
    }
    else
    {
        firstRow = false; 
    }

    (*fd) << "\t" << std::fixed << timestamp << ", " << std::fixed << std::fixed << val_rx << ", " << std::fixed << val_ry << ", " << std::fixed << val_rz;
    (*fd).flush(); 
}
 
void getOdometryData(const nav_msgs::Odometry::ConstPtr &msg, double &tx, double &ty, double &tz, double &rx, double &ry, double &rz)
{
    tx = msg->pose.pose.position.x;
    ty = msg->pose.pose.position.y;
    tz = msg->pose.pose.position.z;
            
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(rx, ry, rz); 
}

void getPoseStampedData(const geometry_msgs::PoseStamped::ConstPtr &msg, double &tx, double &ty, double &tz, double &rx, double &ry, double &rz)
{
    tx = msg->pose.position.x;
    ty = msg->pose.position.y;
    tz = msg->pose.position.z;
            
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3(q).getRPY(rx, ry, rz);   
}

void getGPSData(const msgs_septentrio::LrmGps::ConstPtr &msg, double &tx, double &ty, double &tz, double &rx, double &ry, double &rz)
{
    geographic_msgs::GeoPose geo_pose;
    geo_pose.position.latitude = msg->latitude;
    geo_pose.position.longitude = msg->longitude;
    geo_pose.position.altitude = msg->altitude;
    geo_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(msg->orientation.x, msg->orientation.y, msg->orientation.z);
    
    geodesy::UTMPose utm_pose;
    geodesy::fromMsg(geo_pose, utm_pose);    

    tx = utm_pose.position.easting;
    ty = utm_pose.position.northing;
    tz = utm_pose.position.altitude;
    
    tf::Quaternion q(utm_pose.orientation.x, utm_pose.orientation.y, utm_pose.orientation.z, utm_pose.orientation.w);
    tf::Matrix3x3(q).getRPY(rx, ry, rz);    
}

void getPoseCovarStampedData(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg, double &tx, double &ty, double &tz, double &rx, double &ry, double &rz)
{
    tx = msg->pose.pose.position.x;
    ty = msg->pose.pose.position.y;
    tz = msg->pose.pose.position.z;
            
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(rx, ry, rz); 
}

void getIMUData(const msgs_raw_imu::RawIMU::ConstPtr &msg, double &rx, double &ry, double &rz)
{
    rx = msg->magnetic_field.x;
    ry = msg->magnetic_field.y;
    rz = msg->magnetic_field.z;
}

