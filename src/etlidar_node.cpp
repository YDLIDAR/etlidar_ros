/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 *
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include <LIDAR/LIDARDevice.h>

#ifndef M_PI
#define M_PI 3.1415926
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace ydlidar;

#define ROSVerision "1.0.0"
#define FOV 300
ros::Publisher scan_pub;
std::string frame_id = "laser_frame";
double angle_max = 150, angle_min = -150;
double max_range = 64, min_range = 0.035;
std::vector<float> ignore_array;


std::vector<float> split(const std::string &s, char delim) {
  std::vector<float> elems;
  std::stringstream ss(s);
  std::string number;

  while (std::getline(ss, number, delim)) {
    elems.push_back(atof(number.c_str()));
  }

  return elems;
}


void ETLidarScanCallBack(const ydlidar::lidarData& data) {
  size_t all_nodes_counts = data.data.size();
  double each_angle = FOV*1.0 / (all_nodes_counts - 1);
  ydlidar::lidarData compensate_data;
  compensate_data.data.resize(all_nodes_counts);
  unsigned int i = 0;
  for (; i < all_nodes_counts; i++) {
    float angle = data.data[i].angle;
    angle = angle + 180;
    if (angle >= 360) {
      angle = angle - 360;
    }
    int inter = (int)(angle / each_angle);
    float angle_pre = angle - inter * each_angle;
    float angle_next = (inter + 1) * each_angle - angle;

    if (angle_pre < angle_next) {
      if (inter < all_nodes_counts) {
        compensate_data.data[inter] = data.data[i];
      }
    } else {
      if (inter < all_nodes_counts - 1) {
        compensate_data.data[inter + 1] = data.data[i];
      }
    }
  }

      


  int counts = all_nodes_counts * ((angle_max - angle_min) /FOV);
  int angle_start = 150 + angle_min;
  int node_start = all_nodes_counts * (angle_start*1.0 / FOV);
  sensor_msgs::LaserScan scan_msg;
  ros::Time start_scan_time;
  start_scan_time.sec = data.system_timestamp / 1000000000ul;
  start_scan_time.nsec = data.system_timestamp % 1000000000ul;
  scan_msg.header.stamp = start_scan_time;
  scan_msg.header.frame_id = frame_id;
  scan_msg.angle_min = DEG2RAD(angle_min);
  scan_msg.angle_max = DEG2RAD(angle_max);
  scan_msg.angle_increment =(scan_msg.angle_max - scan_msg.angle_min)/(all_nodes_counts - 1);
  scan_msg.scan_time = data.scan_time*1.0/1e9;
  scan_msg.time_increment = scan_msg.scan_time/(all_nodes_counts - 1);
  scan_msg.range_min = min_range;
  scan_msg.range_max = max_range;
  scan_msg.ranges.resize(counts);
  scan_msg.intensities.resize(counts);
  int index = 0;
  float range = 0.0;
  float intensity = 0.0;
  for (size_t i = 0; i < all_nodes_counts; i++) {
    range = compensate_data.data[i].range;
    if (i < all_nodes_counts / 2) {
      index = all_nodes_counts / 2 - 1 - i;
    } else {
      index = all_nodes_counts - 1 - (i - all_nodes_counts / 2);
    }
    if (range > max_range || range < min_range) {
      range = 0.0;
    }
    intensity = compensate_data.data[i].intensity;
    int pos = index - node_start ;

    if (0 <= pos && pos < counts) {
      scan_msg.ranges[pos] =  range;
      scan_msg.intensities[pos] = intensity;
    }
  }


  scan_pub.publish(scan_msg);
}

int main(int argc, char *argv[]) {

  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);
  ros::init(argc, argv, "etlidar_node");

  std::string ip;
  int port = 9000;
  std::string list;
 

  ros::NodeHandle nh;
  scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
  ros::NodeHandle nh_private("~");
  nh_private.param<std::string>("ip", ip, "192.168.0.11");
  nh_private.param<int>("port", port, 9000);
  nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
  nh_private.param<double>("angle_max", angle_max, 150);
  nh_private.param<double>("angle_min", angle_min, -150);
  nh_private.param<double>("range_max", max_range, 64.0);
  nh_private.param<double>("range_min", min_range, 0.0035);
  nh_private.param<std::string>("ignore_array", list, "");

  ignore_array = split(list, ',');

  if (ignore_array.size() % 2) {
    ROS_ERROR_STREAM("ignore array is odd need be even");
  }

  for (uint16_t i = 0 ; i < ignore_array.size(); i++) {
    if (ignore_array[i] < -150 && ignore_array[i] > 150) {
      ROS_ERROR_STREAM("ignore array should be between -150 and 150");
    }
  }


  if (angle_max < angle_min) {
    double temp = angle_max;
    angle_max = angle_min;
    angle_min = temp;
  }
  if(angle_max > 150) {
    angle_max = 150;
  }
  if(angle_min < -150) {
    angle_min = -150;
  }

  ros::Rate rate(30);
  ROS_INFO("[YDLIDAR INFO] Now ETLIDAR ROS SDK VERSION:%s .......", ROSVerision);

  try {
    ydlidar::LIDAR etlidar(ip.c_str(), port);   
    etlidar.RegisterLIDARDataCallback(&ETLidarScanCallBack);
    while (ros::ok()) {
      rate.sleep();
      ros::spinOnce();
    } 

  }catch(ydlidar::DeviceException& e) {
      std::cout << e.what()<< std::endl;
  }catch(...) {
      std::cout <<"Unkown error" << std::endl;
  }
  return 0;
}