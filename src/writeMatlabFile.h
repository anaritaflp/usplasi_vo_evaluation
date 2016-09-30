#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "msgs_septentrio/LrmGps.h"
#include "msgs_raw_imu/RawIMU.h"
#include <geographic_msgs/GeoPose.h>
#include "geodesy/utm.h"
#include <tf/tf.h>

#include <iostream>
#include <string>
#include <fstream>
#include <limits>

void beginFile(std::ofstream *fd, std::string variableName, bool &firstRow);
void endFile(std::ofstream *fd);
void writeOdometry(std::ofstream *fd, bool &firstRow, const nav_msgs::Odometry::ConstPtr &msg);
void writePoseStamped(std::ofstream *fd, bool &firstRow, const geometry_msgs::PoseStamped::ConstPtr &msg);
void writeGPS(std::ofstream *fd, bool &firstRow, const msgs_septentrio::LrmGps::ConstPtr &msg);
void writePoseCovarStamped(std::ofstream *fd, bool &firstRow, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
void writeIMU(std::ofstream *fd, bool &firstRow, const msgs_raw_imu::RawIMU::ConstPtr &msg);

void getOdometryData(const nav_msgs::Odometry::ConstPtr &msg, double &tx, double &ty, double &tz, double &rx, double &ry, double &rz);
void getPoseStampedData(const geometry_msgs::PoseStamped::ConstPtr &msg, double &tx, double &ty, double &tz, double &rx, double &ry, double &rz);
void getGPSData(const msgs_septentrio::LrmGps::ConstPtr &msg, double &tx, double &ty, double &tz, double &rx, double &ry, double &rz);
void getPoseCovarStampedData(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg, double &tx, double &ty, double &tz, double &rx, double &ry, double &rz);
void getIMUData(const msgs_raw_imu::RawIMU::ConstPtr &msg, double &rx, double &ry, double &rz);