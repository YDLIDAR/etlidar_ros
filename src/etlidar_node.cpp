/*
 *  ETLIDAR SYSTEM
 *  ETLIDAR ROS Node Client
 *
 *  Copyright 2015 - 2019 EAI TEAM
 *  http://www.ydlidar.com
 *
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include "ETLidarDriver.h"

#ifndef M_PI
#define M_PI 3.1415926
#endif

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*M_PI/180.)
#endif

using namespace ydlidar;

#define ROSVerision "1.0.1"

std::vector<float> split(const std::string &s, char delim) {
  std::vector<float> elems;
  std::stringstream ss(s);
  std::string number;

  while (std::getline(ss, number, delim)) {
    elems.push_back(atof(number.c_str()));
  }

  return elems;
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
  std::string list;
  std::string frame_id = "laser_frame";
  double angle_max = 150, angle_min = -150;
  double max_range = 64, min_range = 0.035;
  std::vector<float> ignore_array;
  int scan_frequency = 20;
  int port = 8000;
 

  ros::NodeHandle nh;
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
  ros::NodeHandle nh_private("~");
  nh_private.param<std::string>("ip", ip, "192.168.0.11");
  nh_private.param<int>("port", port, 8000);
  nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
  nh_private.param<double>("angle_max", angle_max, 150);
  nh_private.param<double>("angle_min", angle_min, -150);
  nh_private.param<double>("range_max", max_range, 64.0);
  nh_private.param<double>("range_min", min_range, 0.0035);
  nh_private.param<int>("scan_frequency", scan_frequency, 20);
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

  if(scan_frequency > 50) {
    scan_frequency = 50;
  } else if(scan_frequency < 5) {
    scan_frequency = 20;
  }

  ros::Rate rate(scan_frequency);
  ROS_INFO("[YDLIDAR INFO] Now ETLIDAR ROS SDK VERSION:%s", ROSVerision);

  ydlidar::ETLidarDriver lidar;
  lidarConfig config;

  if (!lidar.getScanCfg(config, ip)) {
    ROS_ERROR("Failed to get lidar config...");
    return 0;
  }

  
  config.dataRecvPort = port;
  config.motor_rpm = scan_frequency*60;
  lidar.updateScanCfg(config);

  result_t ans = lidar.connect(ip, config.dataRecvPort);

  if (!IS_OK(ans)) {
    ROS_ERROR("Failed to connecting lidar...");
    return 0;
  }

  config = lidar.getFinishedScanCfg();
  int m_sampleRate = 1000 / config.laserScanFrequency * 1000;

  int start_angle = config.fov_start;
  start_angle = (360 - start_angle) + 180;
  if (start_angle > 180) {
    start_angle -= 360;
  }  

  int end_angle = config.fov_end;
  end_angle = (360 - end_angle) + 180;
  if (end_angle > 180) {
    end_angle -= 360;
  }
  if(start_angle > end_angle) {
    int tmp = end_angle;
    end_angle = start_angle;
    start_angle = tmp;
  }
  if(angle_max > end_angle) {
    angle_max = end_angle;
  }

  if(angle_min < start_angle) {
    angle_min = start_angle;
  }

  bool rs = lidar.turnOn();
  while (rs && ros::ok()) {
    lidarData scan;
    ans = lidar.grabScanData(scan);
    if(IS_OK(ans)) {
      size_t scan_size = scan.data.size();
      int all_nodes_counts = m_sampleRate/(config.motor_rpm/60);
      double each_angle = 360.0/all_nodes_counts;
      int counts = all_nodes_counts * ((angle_max - angle_min) /360);
      sensor_msgs::LaserScan scan_msg;
      ros::Time start_scan_time;
      start_scan_time.sec = scan.system_timestamp / 1000000000ul;
      start_scan_time.nsec = scan.system_timestamp % 1000000000ul;
      scan_msg.header.stamp = start_scan_time;
      scan_msg.header.frame_id = frame_id;
      scan_msg.angle_min = DEG2RAD(angle_min);
      scan_msg.angle_max = DEG2RAD(angle_max);
      scan_msg.angle_increment = DEG2RAD(each_angle);
      scan_msg.scan_time = scan.scan_time*1.0/1e9;
      scan_msg.time_increment = scan_msg.scan_time/(all_nodes_counts - 1);
      scan_msg.range_min = min_range;
      scan_msg.range_max = max_range;
      scan_msg.ranges.resize(counts);
      scan_msg.intensities.resize(counts);
      int index = 0;
      float range = 0.0;
      float intensity = 0.0;


      for (int i = 0; i < scan_size; i++) {
        float angle = scan.data[i].angle;
        angle = (360 - angle) + 180;
        if (angle > 180) {
          angle -= 360;
        }
        range = scan.data[i].range;
        intensity = scan.data[i].intensity;
        if (ignore_array.size() != 0) {
          for (uint16_t j = 0; j < ignore_array.size(); j = j + 2) {
            if ((ignore_array[j] < angle) && (angle <= ignore_array[j + 1])) {
              range = 0.0;
              break;
            }
          }
        }

        if (range > max_range || range < min_range) {
          range = 0.0;
          intensity = 0.0;
        }
        index = int((angle - angle_min) / each_angle + 0.5);
        if(index>0 && index < counts) {
          scan_msg.ranges[index] =  range;
          scan_msg.intensities[index] = intensity;
        }


      }

      scan_pub.publish(scan_msg);
    } else {
      ROS_WARN("Failed to get scan data");
    }
    rate.sleep();
    ros::spinOnce();
  }

  fprintf(stdout, "disconnecting...\n");
  fflush(stdout);
  lidar.turnOff();
  lidar.disconnect();
  return 0;
}
