// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <vector>
#include <tuple>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

using namespace std;
//using namespace std::chrono_literals;
using std::placeholders::_1;

const float LOOKAHEAD_DISTANCE = 0.3;
const float VELOCITY = 2;
const float MAX_STEERING_ANGLE = 0.4;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Pure_Pursuit_Node : public rclcpp::Node
{
public:
  int nearest_waypoint_index;
  vector<tuple<double, double>> path_points_2d;
  bool go_drive;
  double steering_angle;

  //sichergehen das der Vector zu beginn leer ist
  std::vector<double> last_visited_waypoints;

  Pure_Pursuit_Node()
  : Node("Pure_Pursuit")
  {
    marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("csv_point", 10);
    goal_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("current_goal_point", 10);
    drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    control_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/control", 10);

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom",10, std::bind(&Pure_Pursuit_Node::odom_callback, this, std::placeholders::_1));
    
    last_visited_waypoints.clear();
    steering_angle = 0;

    string file_name = "/home/emelies/ros_mpc_env/ba_ws_com/maps/Austin_map_race_line.csv";
    ifstream Raceline_CSV;
    Raceline_CSV.open(file_name);

    //import waypoints aus csv file zur not marker in WayPOints
    string line, str_x, str_y;
    tuple<double, double> point;
  
    while (getline(Raceline_CSV, line)){
      //RCLCPP_INFO(this->get_logger(), "line= %s", line.c_str());
    //Todo
      if(line.empty()){
        continue;
      }
      else{
        stringstream stream_line;
        stream_line << line;
        getline(stream_line, str_x, ',');
        getline(stream_line, str_y, ',');
        int size_x = str_x.length();

        //possible problem with the e+01 or e-01 for parsing
        //printf("StrX=", str_x);
        //RCLCPP_INFO(this->get_logger(), "STR_X= %s", str_x.c_str());
        //cout << str_x;
        //cout << str_y;

        double x = stod(str_x);
        double y = stod(str_y);

        point = std::make_tuple(x, y);
        path_points_2d.push_back(point);
      }
    }
    //initialisierung damit noch kein index vorliegt
    nearest_waypoint_index = -1;
    
    char x;
    cout<< "Start car? [y/n]";
    cin>>x;
    go_drive =  (x =='y');
  }

  
//yaw is the relevant angle
//in rad
  tuple<double,double,double> euler_from_quaternion(double x, double y, double z, double w){
      tuple<double, double, double> euler_angles;
      tf2::Quaternion q;
      q.setValue(x,y,z,w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      euler_angles= make_tuple(roll, pitch, yaw);
    return euler_angles;
  }

  double calc_euc_dist(tuple<double,double> point1, tuple<double,double> point2){
    double distance_x = get<0>(point1)-get<0>(point2);
    double distance_y= get<1>(point1)-get<1>(point2);
    double distance = sqrt(distance_x*distance_x + distance_y*distance_y);
  return distance;
  } 

  double calc_turning_angle(tuple<double, double> car_position, tuple<double, double, double> car_orientation, tuple<double, double> next_waypoint){
   double curvature;

   tuple<double, double> transformed_waypoint = transform_waypoint(next_waypoint, car_position, car_orientation);
   double x = get<0>(transformed_waypoint); 
   double l = calc_euc_dist(car_position, next_waypoint);
   curvature= (2*x)/(l*l);
   //woher kommt der atan? und woher die 0.324? degrees(atan(curvature * WHEEL_BASE))
   //curvature = atan(0.324 * curvature); 

   return -curvature;
  }

  int find_nearest_waypoint(tuple<double, double> car_position, vector<tuple<double, double>> path_points_2d){
    double smallest_dist = INFINITY;
    int index = 0;
    double dist;
    nearest_waypoint_index;

    for(tuple<double, double> way_point : path_points_2d){
      
      dist = calc_euc_dist(car_position, way_point);
      if( dist<smallest_dist and dist>LOOKAHEAD_DISTANCE){
        smallest_dist = dist;
        nearest_waypoint_index = index;
      }
      index++;
    }
    //warum genau +1? damit es vor dem auto liegt
      return nearest_waypoint_index+1;
  } 

  int find_next_waypoint(int current_waypoint, tuple<double, double> car_position, vector<tuple<double, double>> path_points_2d){
    int new_waypoint;
    if(calc_euc_dist(path_points_2d.at(current_waypoint), car_position) < LOOKAHEAD_DISTANCE){
      new_waypoint = current_waypoint + 1;
    }
    else{
      return current_waypoint;
    }

    if(new_waypoint == path_points_2d.size()){
      return 0;
    } 

    return new_waypoint;  
  }

  tuple<double, double> transform_waypoint(tuple<double, double> waypoint, tuple<double, double> car_pos, tuple<double, double, double> car_orientation){
    tuple<double, double> transformed_waypoint;
    tuple<double, double> waypoint_t;
    tuple<double, double> waypoint_r;
    
    double theta = M_PI/2 - get<2>(car_orientation); //yaw from odom msg

    get<0>(waypoint_t) = get<0>(waypoint) - get<0>(car_pos);
    get<1>(waypoint_t) = get<1>(waypoint) - get<1>(car_pos);

    get<0>(waypoint_r) = get<0>(waypoint_t)*cos(theta) - get<1>(waypoint_t)*sin(theta);
    get<1>(waypoint_r) = get<0>(waypoint_t)*sin(theta) + get<1>(waypoint_t)*cos(theta);

    //woher kommt das minus
    //get<1>(waypoint) = -get<0>(waypoint)*sin(theta) + get<1>(waypoint)*cos(theta);

    //transformed_waypoint = waypoint_r;
    return waypoint_r;
  }





private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_pub;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

  void publish_single_point(double x, double y){
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = rclcpp::Clock().now();
    
    marker.ns= "goal_point";
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.id = 9999;

    marker.scale.x =0.3;
    marker.scale.y =0.3;
    marker.scale.z =0.3;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    goal_marker_pub->publish(marker);
  }

  void publish_points(){
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    
    //array in vector for schleife und datentypen komisch
    for (tuple<double, double> p : path_points_2d){
      visualization_msgs::msg::Marker marker;
      
      marker.header.frame_id = "/map";
      marker.header.stamp = rclcpp::Clock().now();
      marker.type = 2;
      marker.id = marker_id,
      marker_id +=1;
      marker.scale.x =0.2; 
      marker.scale.y =0.2;
      marker.scale.z =0.2;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      marker.pose.position.x = get<0>(p);
      marker.pose.position.y = get<1>(p);
      //da pathpoints_2d keine z_pos haben 
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      //push_back adds element to the end of the vector
      marker_array.markers.push_back(marker);
    }     
      marker_pub->publish(marker_array);

  }
  

  //still todo
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    tuple<double, double> car_position_2d;
    tuple<double, double, double> car_orientation;
    car_position_2d = make_tuple(msg->pose.pose.position.x, msg->pose.pose.position.y);
    car_orientation= euler_from_quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w );
    
    if (nearest_waypoint_index ==-1){
      nearest_waypoint_index = find_nearest_waypoint(car_position_2d, path_points_2d);
    }
    else{
      nearest_waypoint_index = find_next_waypoint(nearest_waypoint_index, car_position_2d, path_points_2d);
    }

    tuple<double, double> next_goalpoint = path_points_2d.at(nearest_waypoint_index);
    //tuple<double, double> transformed_waypoint = transform_waypoint(nearest_waypoint, car_position_2d, car_orientation);

    publish_points();
    publish_single_point(get<0>(next_goalpoint), get<1>(next_goalpoint));
    
    double turning_angle = calc_turning_angle(car_position_2d, car_orientation, next_goalpoint);

    if(turning_angle < -MAX_STEERING_ANGLE){
      steering_angle = -MAX_STEERING_ANGLE;
    }
    else if (turning_angle > MAX_STEERING_ANGLE){
      steering_angle = MAX_STEERING_ANGLE;
    }
    else{
      steering_angle = turning_angle;
    }

    //cout<<"Steering_angle: " << steering_angle;




    //if(go_drive){

      ackermann_msgs::msg::AckermannDriveStamped drive_msg;
      ackermann_msgs::msg::AckermannDrive drive;
      drive.speed = VELOCITY;
      drive.steering_angle = steering_angle;
      drive_msg.drive = drive;

      control_pub->publish(drive_msg);
      //drive_pub->publish(drive_msg);
      //todo
      //add velocitiy and angle later
      //cout<<"driving";

      
    //}
    

  }
  
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pure_Pursuit_Node>());
  rclcpp::shutdown();
  return 0;
}