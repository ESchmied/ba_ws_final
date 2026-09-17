#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "std_msgs/msg/bool.hpp"


extern "C" {
#include "grampc.h"
#include "time.h"
#include "my_cpp_py_pkg/userparam.h"
#include "my_cpp_py_pkg/userparambackup.h"
}

#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>



using namespace std;

// Waypoint file, adjust as needed 
//const std::string waypoint_file = "/home/emelies/ros_mpc_env/ba_ws_com/maps/Spielberg_map_filled_race_line.csv"; 
const std::string waypoint_file = "/home/emelies/ros_mpc_env/ba_ws_com/maps/Austin_map_centerline.csv";
const std::string centerline_file = "/home/emelies/ros_mpc_env/ba_ws_com/maps/Austin_map_centerline.csv";
const std::string inner_border_file = "/home/emelies/ros_mpc_env/ba_ws_com/maps/Austin_map_inner_border_0.3.csv";
const std::string outer_border_file = "/home/emelies/ros_mpc_env/ba_ws_com/maps/Austin_map_outer_border_0.3.csv";

// Vehicle Parameters
constexpr typeRNum L = 0.3302;        //0.33 /0.58[m] (Länge) in xacro: 0.3302
constexpr typeRNum W = 0.2032;      //in xacro 0.2032
constexpr typeRNum V_MAX = 3.0;     // [m/s] ursprünglich 2
constexpr typeRNum M = 3.74;
constexpr typeRNum LF = L/2;
constexpr typeRNum LR = L/2;
constexpr typeRNum C_AF = 4.718;
constexpr typeRNum C_AR = 5.4562;
constexpr typeRNum IZ = 0.04712;

constexpr typeRNum YAW_MIN = -0.4;  // Steering angle in rad
constexpr typeRNum YAW_MAX = 0.4;

constexpr typeRNum A_MIN = -4;      // Acceleration  ursprünglich -1/1
constexpr typeRNum A_MAX = 2;

// OCP Parameters dt*(Nhor-1) = Thor
//wichtig das die Supervisor Punkte vor dem Auto liegen? 
constexpr typeRNum DT = 0.1;  //0.25 ursprünglich 0.01 je größer dt desto weniger oszilliert das auto
constexpr typeRNum NHOR = 21; //11
constexpr typeRNum THOR = 2; //2.5

constexpr typeInt NX = 4; //x,y,yaw,v
constexpr typeInt NU = 2; // steer, a

// Cost Weights
constexpr typeRNum Q_POS = 0.6; //0.5
constexpr typeRNum Q_THETA = 0.4; //0.3
constexpr typeRNum Q_VEL = 0.01; //0.1
constexpr typeRNum R_STEER = 0.1; //0.1
constexpr typeRNum R_ACCEL = 0.02; //0.02

struct my_state{
           double x;
           double y;
           double yaw;
           double v;
       };

struct my_input{
        double speed;
        double a;
        double steer;
    };



// In your header or class definition:  
class MPCNode : public rclcpp::Node {
public:
  MPCNode() : Node("mpc_node2") {
    RCLCPP_INFO(this->get_logger(), "MPCNode2 initialized");

    // Load the reference path from CSV into a flat vector of doubles.
    flat_path_points_ = load_flat_pathpoints(waypoint_file);
    flat_center_points_ = load_flat_pathpoints(centerline_file);
    //for ease of programming in probfct
    flat_inner_border_points_ = load_flat_pathpoints(inner_border_file);
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

   /*  auto test = convertPointsToTrajectory(flat_path_points_);
    for (int i = 0; i < test.size()/3; i++)
    {
      RCLCPP_INFO(this->get_logger(), "Traj %.d: x=%.3f, y=%.3f, yaw=%.3f", i,test[3*i],test[3*i +1],test[3*i +2]);
    } */
    

    // Subscribe to odometry.
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", rclcpp::SensorDataQoS(), 
      std::bind(&MPCNode::odom_callback, this, std::placeholders::_1));

    //subscribe to pure_pursuit control outputs 
    control_subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/control", 10, 
      std::bind(&MPCNode::control_callback, this, std::placeholders::_1));

    // Other publishers…
    drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    trajectory_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mpc_trajectory", 10);
    backup_trajectory_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("backup_mpc_trajectory", 10);
    active_ref_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mpc_ref_traj", 10);
    reference_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("reference_path", 10);
    border_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("border_points", 10);
    border_line_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("border_line", 10);

    collision_flag_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/collision_flag", 10);
    
    //to avoid overload of rviz not published
    //publish_reference_path(); //load in the reference path in RViz
    single_point_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("next_point", 10);

    //printf("vor init: grampc_supervisor memory adress: %p, grampc_backup memory adress %p\n", grampc_supervisor, grampc_backup);

    // Initialize GRAMPC
    //mpc_supervisor = init_grampc_supervisor(6, 2);
    //mpc_backup = init_grampc_backup(6, 2);
    init_grampc_supervisor(8, 3);
    init_grampc_backup(6, 2);


    //printf("nach init: grampc_supervisor memory adress: %p, grampc_backup memory adress %p\n", grampc_supervisor, grampc_backup);

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
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_subscriber_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr backup_trajectory_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr active_ref_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr border_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr reference_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr border_line_publisher_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_flag_publisher_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr single_point_publisher_;
  

  // GRAMPC pointer
  //TYPE_GRAMPC_POINTER(mpc_supervisor)
  //TYPE_GRAMPC_POINTER(mpc_backup)
  TYPE_GRAMPC_POINTER(grampc_supervisor)
  TYPE_GRAMPC_POINTER(grampc_backup)

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
  bool backup_flag = false;
  std::chrono::time_point<std::chrono::steady_clock> start_time_backup_mpc;
  std::chrono::time_point<std::chrono::steady_clock> start_time_control_msg;


  double backup_steering_angle;
  double backup_v;

  // Store the user parameters to pass to grampc
  UserParam user_param_;

  my_state current_state;

  double euclidian_distance(double point1_x, double point1_y, double point2_x, double point2_y){
    double dist_sqrd = POW((point1_x - point2_x),2) + POW((point1_y - point2_y),2);
    return sqrt(dist_sqrd);
  }

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

    //printf("[CPP] num_points: %d, nearest_idx_cpp: %d\n", num_points, nearest_idx);
    return nearest_idx;
  }
/* 
  // ==================== GRAMPC ====================
typeGRAMPC* create_grampc_instance(UserParam* param){
    // === Initialize GRAMPC ===
    TYPE_GRAMPC_POINTER(grampc)
    typeUSERPARAM *userparam = static_cast<void *>(param);
    grampc_init(&grampc, userparam);

    // Set initial state (p_lon, v_lon, p_lat, v_lat)
    double x0[4] = {0.0, 2.0, 0.0, 0.0};
    grampc_setparam_real_vector(grampc, "x0", x0);

    // a_lon, a_lat constraints
    ctypeRNum umin[2] = {-10, -10};
    ctypeRNum umax[2] = {10, 10};
    grampc_setparam_real_vector(grampc, "umin", umin);
    grampc_setparam_real_vector(grampc, "umax", umax);

    // Horizon and step size
    grampc_setparam_real(grampc, "Thor", param->Thor);
    grampc_setparam_real(grampc, "dt", param->dt);

    grampc_setopt_string(grampc, "Integrator", "modeuler"); // feste Schrittgröße

    grampc_setopt_int(grampc, "MaxGradIter", 5); 
    grampc_setopt_int(grampc, "MaxMultIter", 10);

    return grampc;
} */

  // --------------------------
  // GRAMPC initialization (set parameters, dt, horizon, etc.)
  typeGRAMPC* init_grampc_supervisor(int max_grad_iter = 6 , int max_mult_iter = 2) {
    // Init grampc
    //TYPE_GRAMPC_POINTER(grampc_supervisor)
    //typeUSERPARAM *userparam = NULL;
    typeUSERPARAM *userparam = static_cast<void*>(&user_param_); //&userparam == memory address of userparam
    grampc_init(&grampc_supervisor, userparam);

    // Set initial state and control limits
    ctypeRNum x0[NX] = { 0.0, 0.0, 0.0 , 0.0};
    ctypeRNum umin[NU] = {YAW_MIN, A_MIN};
    ctypeRNum umax[NU] = {YAW_MAX, A_MAX};

    grampc_setparam_real_vector(grampc_supervisor, "x0", x0);
    grampc_setparam_real_vector(grampc_supervisor, "umin", umin);
    grampc_setparam_real_vector(grampc_supervisor, "umax", umax);

    grampc_setparam_real(grampc_supervisor, "dt", DT);
    grampc_setparam_real(grampc_supervisor, "t0", 0.0);

    grampc_setopt_int(grampc_supervisor, "Nhor", NHOR);
    grampc_setparam_real(grampc_supervisor, "Thor", THOR);

    //Important!! Without it the car drives serpentine-like 
    //works without too, but is not as smooth
    grampc_setopt_string(grampc_supervisor, "ShiftControl", "on");  //off

    //maby only in v2.3
    //grampc_setopt_string(grampc, "Integrator", "discrete");

    // Set number of gradient iterations (example) not to high or else the calculations take too long and the mpc lags behind the real car and starts over compensating
    grampc_setopt_int(grampc_supervisor, "MaxGradIter", max_grad_iter);  //6 //4 DEFAULT 2 //inner loop 
    grampc_setopt_int(grampc_supervisor, "MaxMultIter", max_mult_iter); //2 //2 DEFAULT 1 //outer loop

    //penalty for contraints 
    grampc_setopt_string(grampc_supervisor, "InequalityConstraints", "on");
    grampc_setopt_real(grampc_supervisor, "PenaltyIncreaseFactor", 1.0); //works with 1.0
    grampc_setopt_real(grampc_supervisor, "PenaltyDecreaseFactor", 1.0); //works with 1.0
    grampc_setopt_real(grampc_supervisor, "PenaltyMin", 1.5); //works with 1



    //tolerance for the constraints,
    //all constraints are satisfied within the tolerance defined by ConstraintsAbsTol
    ctypeRNum ConstraintsAbsTol[1] = {0}; //1e-2 works with 0
    grampc_setopt_real_vector(grampc_supervisor, "ConstraintsAbsTol", ConstraintsAbsTol);

    return grampc_supervisor;
  }

  // --------------------------
  // GRAMPC initialization (set parameters, dt, horizon, etc.)
  typeGRAMPC* init_grampc_backup(int max_grad_iter = 6 , int max_mult_iter = 2) {
    // Init grampc
    //TYPE_GRAMPC_POINTER(grampc_backup)
    //typeUSERPARAM *userparam = NULL;
    typeUSERPARAM *userparam = static_cast<void*>(&user_param_); //&userparam == memory address of userparam
    grampc_init(&grampc_backup, userparam);

    // Set initial state and control limits
    ctypeRNum x0[NX] = { 0.0, 0.0, 0.0 , 0.0};
    ctypeRNum umin[NU] = {YAW_MIN, A_MIN};
    ctypeRNum umax[NU] = {YAW_MAX, A_MAX};

    grampc_setparam_real_vector(grampc_backup, "x0", x0);
    grampc_setparam_real_vector(grampc_backup, "umin", umin);
    grampc_setparam_real_vector(grampc_backup, "umax", umax);

    grampc_setparam_real(grampc_backup, "dt", DT);
    grampc_setparam_real(grampc_backup, "t0", 0.0);

    grampc_setopt_int(grampc_backup, "Nhor", NHOR);
    grampc_setparam_real(grampc_backup, "Thor", THOR);

    //Important!! Without it the car drives serpentine-like 
    //works without too, but is not as smooth
    grampc_setopt_string(grampc_backup, "ShiftControl", "on");  //off

    //maby only in v2.3
    //grampc_setopt_string(grampc, "Integrator", "discrete");

    // Set number of gradient iterations (example) not to high or else the calculations take too long and the mpc lags behind the real car and starts over compensating
    grampc_setopt_int(grampc_backup, "MaxGradIter", max_grad_iter);  //6 //4 DEFAULT 2 //inner loop 
    grampc_setopt_int(grampc_backup, "MaxMultIter", max_mult_iter); //2 //2 DEFAULT 1 //outer loop

    //penalty for contraints 
    grampc_setopt_string(grampc_backup, "InequalityConstraints", "on");
    grampc_setopt_real(grampc_backup, "PenaltyIncreaseFactor", 1.0); //works with 1.0
    grampc_setopt_real(grampc_backup, "PenaltyDecreaseFactor", 1.0); //works with 1.0
    grampc_setopt_real(grampc_backup, "PenaltyMin", 1.5); //works with 1



    //tolerance for the constraints,
    //all constraints are satisfied within the tolerance defined by ConstraintsAbsTol
    ctypeRNum ConstraintsAbsTol[1] = {0}; //1e-2 works with 0
    grampc_setopt_real_vector(grampc_backup, "ConstraintsAbsTol", ConstraintsAbsTol);

    return grampc_backup;
  }

  my_state get_next_state(my_state state0, my_input input0, double t){
    my_state state_t;
    
    //double beta = atan(LR/(L)* tan(input0.steer));

    //dx/dt
    double dx = state0.v* cos(state0.yaw); //+beta
    //double dx = state0.v* cos(state0.yaw + beta); 
    state_t.x = state0.x + dx*t;

    //yt = y0 + y' *t;
    double dy = state0.v* sin(state0.yaw); 
    //double dy = state0.v* sin(state0.yaw + beta); 
    state_t.y = state0.y + dy *t;

    //yawt = yaw0 + yaw' *t;
    double dyaw = state0.v * tan(input0.steer)/L;
    //double dyaw = state0.v/L * tan(input0.steer)*cos(beta);
    state_t.yaw = state0.yaw + dyaw *t;

    //vt = v0 + a *t;
    /* if (input0.speed ==  NAN){
      state_t.v = state0.v + input0.a *t;
    }
    else{
      state_t.v = input0.speed;
    } */
    state_t.v = input0.speed;


    return state_t;
  }

  double wrapToPi(double number){
    double wrappedNumber = fmod(number + M_PI, 2.0*M_PI);
    if (wrappedNumber < 0) wrappedNumber += 2.0 * M_PI;
    wrappedNumber -= M_PI;

    return wrappedNumber;
}
  
  // --------------------------
  // Odom_callback function: 
  // Update state and reference trajectory in userparam.
  //TODO: Abbruch Bedingung backupMPC
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract state from odometry.
    //my_state current_state;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double roll, pitch, yaw;
    tf2::Quaternion q(qx, qy, qz, qw);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw); //warum hat die Matrix keinen Namen?

    current_state.yaw = yaw;
    current_state.v = msg->twist.twist.linear.x; //maby nicht in vicon msgs enthalten current_state.

    current_state.x = msg->pose.pose.position.x;
    current_state.y = msg->pose.pose.position.y;

    /* //move odom pos from back of car to center
    //---------------------------------------------------------------------------------
    double msg_x = msg->pose.pose.position.x;
    double msg_y = msg->pose.pose.position.y;
    double msg_xr = msg_x*cos(current_state.yaw) - msg_y*sin(current_state.yaw);
    double msg_yr = msg_x*sin(current_state.yaw) + msg_y*cos(current_state.yaw);
    double msg_yt = msg_yr - 0.5*L ;
    current_state.x = msg_xr*cos(-current_state.yaw) - msg_yt*sin(-current_state.yaw);
    current_state.y = msg_xr*sin(-current_state.yaw) + msg_yt*cos(-current_state.yaw);
   //----------------------------------------------------------------------------------- */
    auto current_time = std::chrono::steady_clock::now();
    auto duration_since_last_control_msg = std::chrono::duration_cast<chrono::milliseconds>(current_time - start_time_control_msg);
    if(backup_flag==false && duration_since_last_control_msg > std::chrono::milliseconds(750)){

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = 0.0;
        drive_msg.drive.steering_angle = 0.0;
        drive_publisher_->publish(drive_msg);

        RCLCPP_INFO(this->get_logger(), "Time since last Control msg: %d, Stopping car.", duration_since_last_control_msg.count());
    }

    //RCLCPP_INFO(this->get_logger(), "Odom received: x=%.6f, y=%.6f, yaw=%.2f, v=%.2f", current_state.x, current_state.y, current_state.yaw, current_state.v);
    
    // Compute the reference trajectory using the flat path points.
    int nearest_idx = getNearestIndex(current_state.x, current_state.y, flat_path_points_);
    int nearest_center_idx = getNearestIndex(current_state.x, current_state.y, flat_center_points_);
    //int nearest_inner_border_idx = getNearestIndex(x,y, flat_inner_border_points_);
    //int nearest_outer_border_idx = getNearestIndex(x,y, flat_outer_border_points_);
    int start_idx = nearest_idx -1;
    //auto ref_traj_ = computeReferenceTrajectory(flat_path_points_, nearest_idx, NHOR+50); // TODO: How many points ahead are necessary?

    auto center_traj_ = computeReferenceTrajectory(flat_center_points_, start_idx, NHOR+25);
    auto ref_traj_ = center_traj_;

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
    //printf("ref_traj_data: first_pt: %f, %f, second_pt: %f, %f\n", ref_traj_[0], ref_traj_[1], ref_traj_[2], ref_traj_[3]);

    user_param_.center_traj = center_traj_.data();
    user_param_.center_traj_len = (int)center_traj_.size()/3;

    
    //update border of user param    
    user_param_.outer_border = flat_outer_border_points_3d.data();
    user_param_.outer_border_len = (int)flat_outer_border_points_3d.size()/3;

    user_param_.inner_border = flat_inner_border_points_3d.data();
    user_param_.inner_border_len = (int)flat_inner_border_points_3d.size()/3;

    //to avoid both MPC running at the same time
    //overloading in Userparam results in NAN sol

    //start backup grampc
    ctypeRNum x0[NX] = {current_state.x, current_state.y, current_state.yaw, current_state.v};
    grampc_setparam_real_vector(grampc_backup, "x0", x0);
    grampc_run(grampc_backup);
    //RCLCPP_INFO(this->get_logger(), "Finished GRAMPC_backup run. Status %d", grampc_backup->sol->status);
    //grampc_printstatus(grampc_supervisor->sol->status, STATUS_LEVEL_DEBUG);
    
    // Extract backup control command.
    backup_steering_angle = grampc_backup->sol->unext[0]; //Extract the solution for k from Grampc for the correct steering angle
    backup_v = grampc_backup->sol->xnext[3]; // Extract the velocity state of the next solution ste */
    //printf("backup_flag == true \n");
    if (backup_flag){
    if(!isnan(backup_v) && !isnan(backup_steering_angle)){
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = backup_v;
        drive_msg.drive.steering_angle = backup_steering_angle;
        drive_publisher_->publish(drive_msg);

        //publish_collision_flag();
        RCLCPP_INFO(this->get_logger(), "Backup MPC still driving");
      }
      else{
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = 0.0;
        drive_msg.drive.steering_angle = 0.0;
        drive_publisher_->publish(drive_msg);

        RCLCPP_INFO(this->get_logger(), "Invalid Backup MPC calculations. Stopping car and shutting down...%f,  %f", backup_v, backup_steering_angle);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        rclcpp::shutdown();
      }

      double nearest_center_pt_x = center_traj_[3]; //use nearest point in traj should equal first point of traj
      double nearest_center_pt_y = center_traj_[4];
      double nearest_center_pt_yaw = center_traj_[5];

      //timer so RL has time to receive collisin flag
      
      auto duration = std::chrono::duration_cast<chrono::milliseconds>(current_time - start_time_backup_mpc);
         
      //printf("Distance error = %f, Heading error = %f \n", euclidian_distance(current_state.x, current_state.y, nearest_center_pt_x, nearest_center_pt_y), abs(wrapToPi(current_state.yaw- nearest_center_pt_yaw)));
      if (duration > std::chrono::milliseconds(100) && euclidian_distance(current_state.x, current_state.y, nearest_center_pt_x, nearest_center_pt_y) < 0.4 && abs((current_state.yaw- nearest_center_pt_yaw) < 0.8)){
        backup_flag = false;
        publish_collision_flag();
        //printf("safe state reached!");
      }
    }

    
   

    // Optionally publish predicted trajectory markers.
    //printf("finished publish drive msg");
    publish_current_ref_trajectory();
    publish_border_points("inner_border", flat_inner_border_points_); //why two different variables for userparam and publishing
    publish_border_points("outer_border", flat_outer_border_points_); //seems to be the same in simluation 
    
    publish_single_point("next_point", flat_path_points_[(nearest_idx+1)*2], flat_path_points_[(nearest_idx+1)*2 +1]);
    
    publish_border("inner_border_line", flat_inner_border_points_);
    publish_border("outer_border_line", flat_outer_border_points_);
    
    publish_mpc_trajectory();
    publish_backup_mpc_trajectory();
  }

  /* 
  --------------------------------------------------------------
  takes control input from PPC and checks if it leads to a feasible solutin in the next time step
  if yes apply PP input 
  if no: start backup mpc and wait till the car has returned to a safe starting point  */
  void control_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg){
    start_time_control_msg = std::chrono::steady_clock::now();
    my_input input_pp;
    input_pp.steer= msg->drive.steering_angle;
    input_pp.speed = msg->drive.speed;
    
    //to avoid both mpc running at the same time
    my_state state_t = get_next_state(current_state, input_pp, DT);
    //grampc->userparam = static_cast<void*>(&user_param_); //jetzt direkt in init_grampc()
    
    
    // Update next state.
    ctypeRNum xt[NX] = {state_t.x, state_t.y, state_t.yaw, state_t.v};
    grampc_setparam_real_vector(grampc_supervisor, "x0", xt);
    //printf("xt: state_t.x: %f, state_t.y: %f, state_t.yaw: %f, state_t.v: %f \n", state_t.x, state_t.y, state_t.yaw, state_t.v);
    // typeRNum t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    // grampc_setparam_real(grampc, "t0", t);
    
    // Run GRAMPC.
    //RCLCPP_INFO(this->get_logger(), "Starting GRAMPC run...");
    grampc_run(grampc_supervisor);
    //RCLCPP_INFO(this->get_logger(), "Finished GRAMPC_supervisor run. Status %d", grampc_supervisor->sol->status);
    //grampc_printstatus(grampc_supervisor->sol->status, STATUS_LEVEL_DEBUG);
    
    bool infeasible_flag = grampc_supervisor->sol->status & 256; // 256 is the bitmask for STATUS_INFEASIBLE
    infeasible_counter *= infeasible_flag; // = * true  damit er sich zurücksetzt falls es doch gelöst wurde 
    infeasible_counter += infeasible_flag;
    //printf("infeasible_counter: %d \n", infeasible_counter);
    bool infeasible = infeasible_counter > 0; //>20 works for driving Only set if the flag was active for multiple runs
    
    
    // Extract control command.
    double steering_angle = grampc_supervisor->sol->unext[0]; //Extract the solution for k+1 from Grampc for the correct steering angle
    double acceleration = grampc_supervisor->sol->unext[1];
    double v_next = grampc_supervisor->sol->xnext[3]; // Extract the velocity state of the next solution step
    
    //RCLCPP_INFO(this->get_logger(), "Published: Steering=%.2f, Speed=%.2f, Acceleration=%.2f", steering_angle, v_next, acceleration);
    // for (int i = 0; i < NHOR; ++i)
    // {
      //   double x_pred = grampc->rws->x[i * NX];
      //   double y_pred = grampc->rws->x[i * NX + 1];
      //   double yaw_pred = grampc->rws->x[i * NX + 2];
      //   double v_pred = grampc->rws->x[i * NX + 3];
      //   double dist = sqrt(POW(x_pred-x,2) + POW(y_pred-y,2));
      //   RCLCPP_INFO(this->get_logger(), "Step %d: x=%.3f, y=%.3f, yaw=%.2f, v=%.3f, dist=%.3f", i, x_pred, y_pred, yaw_pred, v_pred, dist);
      // }
      
    
    double sup_backup_steering_angle;
    double sup_backup_v;
      
      
    if(backup_flag == false){
      //printf("backup_flag == false \n");
      //found a feasible solution
      if (!isnan(v_next) && !isnan(steering_angle) && !infeasible){ //if feasible and we have a sol 
        sup_backup_steering_angle = steering_angle;
        sup_backup_v = v_next;
 
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = input_pp.speed;
        drive_msg.drive.steering_angle = input_pp.steer;
        drive_publisher_->publish(drive_msg);

        RCLCPP_INFO(this->get_logger(), "Feasible Input! PurePursuit Driving");
        //rclcpp::sleep_for(std::chrono::milliseconds(500));
        //rclcpp::shutdown();
      }
      //found a soolution but is infeasible
      else if (!isnan(v_next) && !isnan(steering_angle) && infeasible){

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = sup_backup_v;
        drive_msg.drive.steering_angle = sup_backup_steering_angle;
        drive_publisher_->publish(drive_msg);

        RCLCPP_INFO(this->get_logger(), "Infeasible solution! Backup MPC now driving");
        backup_flag = true;
        publish_collision_flag();
        //rclcpp::sleep_for(std::chrono::milliseconds(500));
        //rclcpp::shutdown();
        start_time_backup_mpc = std::chrono::steady_clock::now();

      }
      //found no solution
      else{
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = 0.0;
        drive_msg.drive.steering_angle = 0.0;
        drive_publisher_->publish(drive_msg);

        RCLCPP_INFO(this->get_logger(), "Invalid MPC calculations. Stopping car and shutting down...%f,  %f", v_next, steering_angle);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        rclcpp::shutdown();
      }

    }
   
    

  }

  void publish_collision_flag(){
    std_msgs::msg::Bool collision_flag;
    collision_flag.data = backup_flag;
    collision_flag_publisher_->publish(collision_flag);
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

    point.color.r = 0.5;
    point.color.g = 0.0;
    point.color.b = 0.5;
    point.color.a = 1.0;

    for (int i = 0; i < NHOR; i++) {
      point.id = i;
      point.pose.position.x = grampc_supervisor->rws->x[i * NX];
      point.pose.position.y = grampc_supervisor->rws->x[i * NX + 1];
      point.pose.position.z = 0.1; //points are floating a bit over ground
      marker_array.markers.push_back(point);
    }
    trajectory_publisher_->publish(marker_array);
  }
 
   // --------------------------
  // Visualize backup MPC horizon trajectory in RViz.
  void publish_backup_mpc_trajectory() {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker point;
    point.header.frame_id = "/map";
    point.header.stamp = this->now();
    point.ns = "backup_mpc_horizon";
    point.type = visualization_msgs::msg::Marker::SPHERE;
    point.action = visualization_msgs::msg::Marker::ADD;
    //Scale and color of the sphere
    point.scale.x = 0.1; 
    point.scale.y = 0.1;
    point.scale.z = 0.1;

    point.color.r = 0.0;
    point.color.g = 0.5;
    point.color.b = 0.5;
    point.color.a = 1.0;

    for (int i = 0; i < NHOR; i++) {
      point.id = i;
      point.pose.position.x = grampc_backup->rws->x[i * NX];
      point.pose.position.y = grampc_backup->rws->x[i * NX + 1];
      point.pose.position.z = 0.1; //points are floating a bit over ground
      marker_array.markers.push_back(point);
    }
    backup_trajectory_publisher_->publish(marker_array);
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

    point.color.r = 0.0;
    point.color.g = 1.0;
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

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

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
  rclcpp::Node::SharedPtr node = std::make_shared<MPCNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  //rclcpp::spin(std::make_shared<MPCNode>());
  rclcpp::shutdown();
  return 0;
}