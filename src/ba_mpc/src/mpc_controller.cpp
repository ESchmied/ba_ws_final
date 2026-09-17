#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

extern "C" {
#include "grampc.h"
#include "time.h"
#include "my_cpp_py_pkg/userparam.h"
}

#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>
#include <cmath>
#include <limits>
#include <algorithm>


using namespace std;

// Waypoint file, adjust as needed 
const std::string waypoint_file = "/home/emelies/ros_mpc_env/ba_ws_com/maps/Austin_map_race_line.csv"; //raceline funktioniert nicht, weil dann die constraint berechnung nicht mehr funktioniert
const std::string centerline_file = "/home/emelies/ros_mpc_env/ba_ws_com/maps/Austin_map_centerline.csv";
const std::string inner_border_file = "/home/emelies/ros_mpc_env/ba_ws_com/maps/Austin_map_inner_border.csv";
const std::string outer_border_file = "/home/emelies/ros_mpc_env/ba_ws_com/maps/Austin_map_outer_border.csv";

// Vehicle Parameters
constexpr typeRNum L = 0.33;        //0.33 /0.58[m] (Länge)
constexpr typeRNum W = 0.31;
constexpr typeRNum V_MAX = 3;     // [m/s] ursprünglich 2
constexpr typeRNum M = 3.74;
constexpr typeRNum LF = L/2;
constexpr typeRNum LR = L/2;
constexpr typeRNum C_AF = 4.718;
constexpr typeRNum C_AR = 5.4562;
constexpr typeRNum IZ = 0.04712;

constexpr typeRNum YAW_MIN = -0.4;  // Steering angle in rad
constexpr typeRNum YAW_MAX = 0.4;

constexpr typeRNum A_MIN = -1.5;      // Acceleration  ursprünglich -1/1
constexpr typeRNum A_MAX = 1.5;

// OCP Parameters dt*(Nhor-1) = Thor
constexpr typeRNum DT = 0.25;  //0.05 ursprünglich 0.01 je höher desto weniger oszilliert das auto
constexpr typeRNum NHOR = 11; //51
constexpr typeRNum THOR = 2.5; //2.5

constexpr typeInt NX = 4; //x,y,yaw,v
constexpr typeInt NU = 2; // steer, a

// Cost Weights
constexpr typeRNum Q_POS = 0.5; //0.5
constexpr typeRNum Q_THETA = 0.3; //0.3
constexpr typeRNum Q_VEL = 0.1; //0.1
constexpr typeRNum R_STEER = 0.1; //0.02
constexpr typeRNum R_ACCEL = 0.02;



// In your header or class definition:  
class MPCNode : public rclcpp::Node {
public:
  MPCNode() : Node("mpc_node") {
    RCLCPP_INFO(this->get_logger(), "MPCNode initialized");

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 3), qos_profile);


    // Load the reference path from CSV into a flat vector of doubles.
    flat_path_points_ = load_flat_pathpoints(waypoint_file);
    flat_center_points_ = load_flat_pathpoints(centerline_file);
    flat_inner_border_points_ = load_flat_pathpoints(inner_border_file);
    //for ease of programming in probfct
    flat_inner_border_points_3d = convertPointsToTrajectory(flat_inner_border_points_);
    flat_outer_border_points_ = load_flat_pathpoints(outer_border_file);
    flat_outer_border_points_3d = convertPointsToTrajectory(flat_outer_border_points_);
    

    // Set user parameters 
    user_param_.dt = DT;
    user_param_.Q_pos = Q_POS;
    user_param_.Q_theta = Q_THETA;
    user_param_.Q_vel = Q_VEL;
    user_param_.R_steer = R_STEER;
    user_param_.R_accel = R_ACCEL;
 
    user_param_.wheelbase = L; //abstand vorderachse und hinter achse
    user_param_.width = W;
    user_param_.max_velocity = V_MAX;
    
    // Log the loaded waypoints and trajectory
    auto &pts = flat_path_points_;          //welche bedeutung hat das & (reference?)
    size_t num_waypoints = pts.size()/2;
    RCLCPP_INFO(this->get_logger(), "Loaded %zu raw values(%zu waypoints)", pts.size(), num_waypoints); //loaded pts.size() raw values (size/2 waypoints)
    
    //why tho? just for logging?
    for(size_t i = 0; i< num_waypoints; i++){ //for each in waypoints 
      double x = pts[2*i];
      double y = pts[2*i + 1];
      RCLCPP_INFO(this->get_logger(), "waypoint %zu: x=%.6f, y=%.6f", i, x, y);
    }

    auto test = convertPointsToTrajectory(flat_path_points_);
    for (int i = 0; i < test.size()/3; i++)
    {
      RCLCPP_INFO(this->get_logger(), "Traj %.d: x=%.3f, y=%.3f, yaw=%.3f", i,test[3*i],test[3*i +1],test[3*i +2]);
    }
    

    // Subscribe to odometry.
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", qos_sensor, 
      std::bind(&MPCNode::odom_callback, this, std::placeholders::_1));

    // Other publishers…
    drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    trajectory_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mpc_trajectory", 10);
    active_ref_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mpc_ref_traj", 10);
    reference_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("reference_path", 10);
    border_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("border_points", 10);
    border_line_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("border_line", 10);
    
    //to avoid overload of rviz not published
    //publish_reference_path(); //load in the reference path in RViz
    single_point_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("next_point", 1);

    // Initialize GRAMPC
    init_grampc();
  }

  //what? zum manuellen starten den autos?
  ~MPCNode() {
    auto stop_msg = ackermann_msgs::msg::AckermannDriveStamped();
    stop_msg.drive.speed = 0.0;
    if (drive_publisher_) {
      drive_publisher_->publish(stop_msg);
    }
  }

private:
  // Publishers/subscribers...
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr active_ref_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr border_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr reference_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr border_line_publisher_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr single_point_publisher_;
  

  // GRAMPC pointer
  TYPE_GRAMPC_POINTER(grampc)

  // Flat reference path (each waypoint stored as [x, y])
  vector<double> flat_path_points_;
  vector<double> flat_center_points_;

  //Flat border points for (each point stored as [x,y])
  vector<double> flat_inner_border_points_;
  vector<double> flat_outer_border_points_;

  vector<double> flat_inner_border_points_3d;
  vector<double> flat_outer_border_points_3d;


  // This member holds the computed reference trajectory (flattened), i.e. for each prediction step: [x_ref, y_ref, yaw_ref]
  vector<double> ref_traj_;

  int infeasible_counter = 0;

  // Store the user parameters to pass to grampc
  UserParam user_param_;

  // --------------------------
  // Load CSV file into a flat vector of doubles.
  std::vector<double> load_flat_pathpoints(std::string file_path) {

    vector<double> points;
    ifstream file(file_path);
    string line;

    while (getline(file, line)){
      if (line.empty())
        continue;

      stringstream stream_line(line);
      string x_str, y_str;
      if (getline(stream_line, x_str, ',') && getline(stream_line, y_str, ',')) {
        double x = stod(x_str);
        double y = stod(y_str);
        points.push_back(x);
        points.push_back(y);
      }
    }

    //not needed for my map i think
    //points.pop_back(); // In my_map_ref.csv the first and last point are the same causing some trouble later (division by zero leading to nan values)
    return points;
    
  } 

  // --------------------------
  // Compute a reference trajectory over the horizon.
  // Here, we compute a vector with Nhor * 3 elements: for each step, [x_ref, y_ref, yaw_ref].
 
  
  //überladene fkt, tut exakt das gleiche nur schöner i guess
  std::vector<double> computeReferenceTrajectory(const std::vector<double>& flat_points, int nearest_idx, int num_points_ahead) {
    int num_points = flat_points.size() / 2;
    std::vector<double> traj;

    for (int i = 0; i < num_points_ahead; ++i) {
        int idx = (nearest_idx + i) % num_points; //sorgt dafür das die idx immer auf punkte in der liste verweist

        double x_ref = flat_points[2 * idx];
        double y_ref = flat_points[2 * idx + 1];

        int next_idx = (idx + 1) % num_points; //if abfrage wird unnötig da modulo 
        double x_next = flat_points[2 * next_idx];
        double y_next = flat_points[2 * next_idx + 1];
        //gibt pos yaw ref von pos x-Achse richtung pos Y-Achse 
        //atan2(y,x)
        double yaw_ref = atan2(y_next - y_ref, x_next - x_ref);

        traj.push_back(x_ref);
        traj.push_back(y_ref);
        traj.push_back(yaw_ref);

        //printf("idx: %d, next_idx: %d, xn: %f, xr: %f, yn: %f, yr: %f \n", idx, next_idx, x_next, x_ref, y_next, y_ref);
    }

    return traj;
  }

  //---------------------------
  //convert points to traj by adding yaw_ref 
  vector<double> convertPointsToTrajectory(const vector<double>& flat_points){
    int num_points = flat_points.size()/2;
    vector<double> traj; // Will contain [x_ref, y_ref, yaw_ref] for each step.

    //for each waypoint
    for (int i = 0; i < num_points; ++i) {
      double x_ref = flat_points[2 * i];
      double y_ref = flat_points[2 * i + 1];

      int next_idx = (i + 1)%num_points; // Loop around if last point

      double x_next = flat_points[2 * next_idx];
      double y_next = flat_points[2 * next_idx + 1];
      double yaw_ref = atan2(y_next - y_ref, x_next - x_ref); //atan2 berechnet globales yaw 
      //cout << "Yaw_ref:" << yaw_ref;
      //RCLCPP_INFO(this->get_logger(), "Yaw_ref: yaw=%.2f", yaw_ref);


      traj.push_back(x_ref);
      traj.push_back(y_ref);
      traj.push_back(yaw_ref);
    }
    return traj;
  }

  // --------------------------
  // Get the index of the closest path point to the current position 
  //for flat lists [x,y] 
  typeInt getNearestIndex(double current_x, double current_y, const vector<double>& flat_points){
    int num_points = flat_points.size() / 2;
    int nearest_idx = 0; // or -1 to make sure its not on the list already
    double min_dist = numeric_limits<double>::max();
    for (int i = 0; i < num_points; i++) {
      double x = flat_points[2 * i];
      double y = flat_points[2 * i + 1];
      double d = sqrt((x - current_x) * (x - current_x) + (y - current_y) * (y - current_y)); //eucl distance
      if (d < min_dist) {
        min_dist = d;
        nearest_idx = i;
      }
    }

    return nearest_idx;
  }


  // --------------------------
  // GRAMPC initialization (set parameters, dt, horizon, etc.)
  void init_grampc() {
    // Init grampc
    typeUSERPARAM *userparam = NULL;
    grampc_init(&grampc, userparam);

    // Set initial state and control limits
    ctypeRNum x0[NX] = { 0.0, 0.0, 0.0 , 0.0};
    ctypeRNum umin[NU] = {YAW_MIN, A_MIN};
    ctypeRNum umax[NU] = {YAW_MAX, A_MAX};

    grampc_setparam_real_vector(grampc, "x0", x0);
    grampc_setparam_real_vector(grampc, "umin", umin);
    grampc_setparam_real_vector(grampc, "umax", umax);

    grampc_setparam_real(grampc, "dt", DT);
    grampc_setparam_real(grampc, "t0", 0.0);

    grampc_setopt_int(grampc, "Nhor", NHOR);
    grampc_setparam_real(grampc, "Thor", THOR);

    //Important!! Without it the car drives serpentine-like 
    //works without too, but is not as smooth
    grampc_setopt_string(grampc, "ShiftControl", "on");  //on

    //maby only in v2.3
    //grampc_setopt_string(grampc, "Integrator", "discrete");

    // Set number of gradient iterations (example) not to high or else the calculations take too long and the mpc lags behind the real car and starts over compensating
    grampc_setopt_int(grampc, "MaxGradIter", 4);  //7 //4 DEFAULT 2 //inner loop 
    grampc_setopt_int(grampc, "MaxMultIter", 2); //2 //2 DEFAULT 1 //outer loop

    //penalty for contraints 
    grampc_setopt_string(grampc, "InequalityConstraints", "on");
    grampc_setopt_real(grampc, "PenaltyIncreaseFactor", 1.05); //works with 1.05
    grampc_setopt_real(grampc, "PenaltyMin", 1); //works with 1



    //tolerance for the constraints,
    //all constraints are satisfied within the tolerance defined by ConstraintsAbsTol
    ctypeRNum ConstraintsAbsTol[1] = { 0 }; //1e-2 works with 0
    grampc_setopt_real_vector(grampc, "ConstraintsAbsTol", ConstraintsAbsTol);
  }


  
  // --------------------------
  // Odom_callback function: 
  // Update state and reference trajectory in userparam.
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract state from odometry.
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double v = msg->twist.twist.linear.x; //maby nicht in vicon msgs enthalten 

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double roll, pitch, yaw;
    tf2::Quaternion q(qx, qy, qz, qw);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw); //warum hat die Matrix keinen Namen?

    RCLCPP_INFO(this->get_logger(), "Odom received: x=%.6f, y=%.6f, yaw=%.2f, v=%.2f", x, y, yaw, v);
    
    // Compute the reference trajectory using the flat path points.
    int nearest_idx = getNearestIndex(x,y,flat_path_points_);
    int nearest_center_idx = getNearestIndex(x,y, flat_center_points_);
    //int nearest_inner_border_idx = getNearestIndex(x,y, flat_inner_border_points_);
    //int nearest_outer_border_idx = getNearestIndex(x,y, flat_outer_border_points_);

    auto ref_traj_ = computeReferenceTrajectory(flat_path_points_, nearest_idx, NHOR+50); // TODO: How many points ahead are necessary?
    auto center_traj_ = computeReferenceTrajectory(flat_center_points_, nearest_center_idx, NHOR+50);

    //publish_single_point("nearest_inner_border", flat_inner_border_points_[nearest_inner_border_idx*2], flat_inner_border_points_[nearest_inner_border_idx*2 +1]);
    //publish_single_point("nearest_outer_border", flat_outer_border_points_[nearest_outer_border_idx*2], flat_outer_border_points_[nearest_outer_border_idx*2 +1]);
    
    //dont use! it makes the car drive weird, lots of swerving 
    //for (int i = 0; i < ref_traj_.size()/3; i++)
    //{
    //  RCLCPP_INFO(this->get_logger(), "Traj: x=%.3f, y=%.3f, yaw=%.3f", ref_traj_[3*i],ref_traj_[3*i +1],ref_traj_[3*i +2]);
    //}
    

    // Update reference trajectory of user parameters
    user_param_.ref_traj = ref_traj_.data();
    user_param_.ref_length = (int)ref_traj_.size()/3;

    user_param_.center_traj = center_traj_.data();
    user_param_.center_traj_len = center_traj_.size()/3;

    
    //update border of user param
    //auto outer_border = load_flat_pathpoints(outer_border_file);
    //auto inner_border = load_flat_pathpoints(inner_border_file);
    
    user_param_.outer_border = flat_outer_border_points_3d.data();
    user_param_.outer_border_len = (int)flat_outer_border_points_3d.size()/3;

    user_param_.inner_border = flat_inner_border_points_3d.data();
    user_param_.inner_border_len = (int)flat_inner_border_points_3d.size()/3;

    grampc->userparam = static_cast<void*>(&user_param_);

    // Update current state.
    ctypeRNum x0[NX] = { x, y, yaw, v};
    grampc_setparam_real_vector(grampc, "x0", x0);

    // typeRNum t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    // grampc_setparam_real(grampc, "t0", t);

    // Run GRAMPC.
    //RCLCPP_INFO(this->get_logger(), "Starting GRAMPC run...");
    grampc_run(grampc);
    RCLCPP_INFO(this->get_logger(), "Finished GRAMPC run. Status %d", grampc->sol->status);
    grampc_printstatus(grampc->sol->status, STATUS_LEVEL_DEBUG);


    bool infeasible_flag = grampc->sol->status & 256; // 256 is the bitmask for STATUS_INFEASIBLE

    infeasible_counter *= infeasible_flag; // = * true  damit er sich zurücksetzt falls es doch gelöst wurde ist 
    infeasible_counter += infeasible_flag;
    printf("infeasible_counter: %d \n", infeasible_counter);

    bool infeasible = infeasible_counter > 10; //>20 works for driving Only set if the flag was active for multiple runs

 
    //publish after Grampc run to avoid interfering with the data update
    //publish_single_point("proj_center_point", user_param_.proj_center_point_x, user_param_.proj_center_point_y);  
    //publish_single_point("proj_border_point", user_param_.proj_border_point_x, user_param_.proj_border_point_y);
    //publish_single_point("next_point", user_param_.car_pos_x, user_param_.car_pos_y);



    // Extract control command.
    double steering_angle = grampc->sol->unext[0]; //Extract the solution for k+1 from Grampc for the correct steering angle
    double acceleration = grampc->sol->unext[1];
    double v_next = grampc->sol->xnext[3]; // Extract the velocity state of the next solution step
    RCLCPP_INFO(this->get_logger(), "Published: Steering=%.2f, Speed=%.2f, Acceleration=%.2f", steering_angle, v_next, acceleration);

    // for (int i = 0; i < NHOR; ++i)
    // {
    //   double x_pred = grampc->rws->x[i * NX];
    //   double y_pred = grampc->rws->x[i * NX + 1];
    //   double yaw_pred = grampc->rws->x[i * NX + 2];
    //   double v_pred = grampc->rws->x[i * NX + 3];
    //   double dist = sqrt(POW(x_pred-x,2) + POW(y_pred-y,2));

    //   RCLCPP_INFO(this->get_logger(), "Step %d: x=%.3f, y=%.3f, yaw=%.2f, v=%.3f, dist=%.3f", i, x_pred, y_pred, yaw_pred, v_pred, dist);
    // }
    
    if (isnan(v_next) || isnan(steering_angle) || infeasible){ //what does is nan? nan ^= not-a-number
      auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
      drive_msg.drive.speed = 0.0;
      drive_msg.drive.steering_angle = 0.0;
      drive_publisher_->publish(drive_msg);

      RCLCPP_INFO(this->get_logger(), "Invalid MPC calculations. Stopping car and shutting down...");
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      rclcpp::shutdown();
    }

    // Publish control command.
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.speed = v_next;
    drive_msg.drive.steering_angle = steering_angle;
    drive_publisher_->publish(drive_msg);

    // Optionally publish predicted trajectory markers.
    publish_current_ref_trajectory();
    publish_border_points("inner_border", flat_inner_border_points_); //why two different variables for userparam and publishing
    publish_border_points("outer_border", flat_outer_border_points_); //seems to be the same in simluation 

    publish_single_point("next_point", flat_path_points_[(nearest_idx+1)*2], flat_path_points_[(nearest_idx+1)*2 +1]);
    publish_single_point("next_point", x, y);

    publish_border("inner_border_line", flat_inner_border_points_);
    publish_border("outer_border_line", flat_outer_border_points_);
    
    publish_mpc_trajectory();
  }

  // --------------------------
  // Visualize MPC horizon trajectory in RViz.
  void publish_mpc_trajectory() {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker point;
    point.header.frame_id = "/map";
    point.header.stamp = this->now();
    point.ns = "mpc_horizon";
    point.type = visualization_msgs::msg::Marker::SPHERE;
    point.action = visualization_msgs::msg::Marker::ADD;
    //Scale and color of the sphere
    point.scale.x = 0.1; 
    point.scale.y = 0.1;
    point.scale.z = 0.1;

    point.color.r = 1.0;
    point.color.g = 0.0;
    point.color.b = 0.0;
    point.color.a = 1.0;

    for (int i = 0; i < NHOR; i++) {
      point.id = i;
      point.pose.position.x = grampc->rws->x[i * NX];
      point.pose.position.y = grampc->rws->x[i * NX + 1];
      point.pose.position.z = 0.1; //points are floating a bit over ground
      marker_array.markers.push_back(point);
    }
    trajectory_publisher_->publish(marker_array);
  }

  // --------------------------
  // Visualize reference path. (green line)
  //does not get visualized in rviz
  void publish_border(string border_ns , vector<double> flat_border_points) {
    visualization_msgs::msg::Marker border;
    border.header.frame_id = "/map";
    border.header.stamp = this->now();
    border.ns = border_ns;
    border.id = 0;
    border.type = visualization_msgs::msg::Marker::LINE_STRIP;
    border.action = visualization_msgs::msg::Marker::ADD;
    //scale and color of the line
    border.scale.x = 0.1;
    border.color.r = 0.0;
    border.color.g = 1.0;
    border.color.b = 0.0;
    border.color.a = 1.0;
    for (size_t i = 0; i < flat_border_points.size() / 2; i++) {
      geometry_msgs::msg::Point p;
      p.x = flat_border_points[2 * i];
      p.y = flat_border_points[2 * i + 1];
      p.z = 0.1;    //line is floating a bit over ground
      border.points.push_back(p);
    }
    border_line_publisher_->publish(border);
  }

  //publish waypoints warum verschwinden die punkte hinter dem auto wieder
  void publish_current_ref_trajectory(){
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker point;
    point.header.frame_id = "/map";
    point.header.stamp = this->now();
    point.ns = "mpc_ref_traj";
    point.type = visualization_msgs::msg::Marker::SPHERE;
    //point.action = visualization_msgs::msg::Marker::ADD;
    //Scale and color of the sphere
    point.scale.x = 0.1; 
    point.scale.y = 0.1;
    point.scale.z = 0.1;

    point.color.r = 1.0;
    point.color.g = 0.0;
    point.color.b = 0.0;
    point.color.a = 1.0;

    for (int i = 0; i < user_param_.ref_length; i++) {
      point.id = i;
      point.pose.position.x = user_param_.ref_traj[3*i];
      point.pose.position.y = user_param_.ref_traj[3*i + 1];
      point.pose.position.z = 0.1; //points are floating a bit over ground
      marker_array.markers.push_back(point);
    }
    active_ref_publisher_->publish(marker_array);
  }

 
  void publish_border_points(string ns, vector<double> border_points){
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker point;
    point.header.frame_id = "/map";
    point.header.stamp = this->now();
    point.ns = ns;
    point.type = visualization_msgs::msg::Marker::SPHERE;
    //point.action = visualization_msgs::msg::Marker::ADD;
    //Scale and color of the sphere
    point.scale.x = 0.1; 
    point.scale.y = 0.1;
    point.scale.z = 0.1;

    point.color.r = 1.0;
    point.color.g = 0.0;
    point.color.b = 0.0;
    point.color.a = 1.0;

    for (int i = 0; i < border_points.size()/2; i++) {
      point.id = i;
      point.pose.position.x = border_points[2*i];
      point.pose.position.y = border_points[2*i + 1];
      point.pose.position.z = 0.1; //points are floating a bit over ground
      marker_array.markers.push_back(point);
    }
    border_publisher_->publish(marker_array);
  }

  void publish_single_point(string ns , double x, double y){
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = this->now();
    
    marker.ns= ns;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.id = 9999;

    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    single_point_publisher_->publish(marker);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPCNode>());
  rclcpp::shutdown();
  return 0;
}