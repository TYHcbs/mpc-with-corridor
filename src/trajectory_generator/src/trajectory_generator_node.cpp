#include <algorithm>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <random>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Useful customized headers
#include "Astar_searcher.h"
#include "backward.hpp"
#include "trajectory_generator_waypoint.h"

#include "decomp_ros_utils/data_ros_utils.h"
#include "decomp_util/ellipsoid_decomp.h"
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>


using namespace std;
using namespace Eigen;

TrajectoryGeneratorWaypoint *_trajGene = new TrajectoryGeneratorWaypoint();
AstarPathFinder *_astar_path_finder = new AstarPathFinder();

// Set the obstacle map
double _resolution, _inv_resolution, _path_resolution;
double _x_size, _y_size, _z_size;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

bool has_global_map(false); 

// Param from launch file
double _vis_traj_width;
double _Vel, _Acc;
int _dev_order, _min_order;

// ros related
ros::Subscriber _map_sub, _pts_sub, _odom_sub, _glb_pc_sub;
ros::Publisher _traj_vis_pub, _traj_pub, _path_vis_pub,_astar_vis_pub;
ros::Publisher corridor_pub;
ros::Publisher corridor_pub2;
ros::Publisher subpointclouds_pub;

// for planning
Vector3d odom_pt, odom_vel, start_pt, target_pt, start_vel;
int _poly_num1D;
MatrixXd _polyCoeff;
VectorXd _polyTime;
double time_duration;
ros::Time time_traj_start;
bool has_odom = false;
bool has_target = false;
pcl::PointCloud<pcl::PointXYZ> global_cloud;
vec_Vec3f waypointsf;
vec_Vec3f observations;


// for replanning
enum STATE {
  INIT,
  WAIT_TARGET,
  GEN_NEW_TRAJ,
  EXEC_TRAJ,
  REPLAN_TRAJ
} exec_state = STATE::INIT;
double no_replan_thresh, replan_thresh;
ros::Timer _exec_timer;
void execCallback(const ros::TimerEvent &e);

// declare
void changeState(STATE new_state, string pos_call);
void printState();
void visTrajectory(MatrixXd polyCoeff, VectorXd time);
void visPath(MatrixXd nodes);
void visPath_astar(MatrixXd nodes);
void trajOptimization(Eigen::MatrixXd path, vector<Vector3d> grid_path, vec_Vec3f grid_pathf);
void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &odom);
void rcvWaypointsCallback(const nav_msgs::Path &wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map);
void trajPublish(MatrixXd polyCoeff, VectorXd time);
bool trajGeneration();
VectorXd timeAllocation(MatrixXd Path);
Vector3d getPos(double t_cur);
Vector3d getVel(double t_cur);
vec_Vec3f cloud_to_vec(const pcl::PointCloud<pcl::PointXYZ> &cloud);
std::vector<Eigen::Matrix<double, 6, -1>> polyhTypeConverter(vec_E<Polyhedron<3>> vs);
vec_Vec3f getSubPointClouds(const pcl::PointCloud<pcl::PointXYZ> &pc, const vec_Vec3f &wp);
void visualizeCorridors(const std::vector<Eigen::Matrix<double, 6, -1>>& corridors);
void visualizeCorridors2(const std::vector<Eigen::Matrix<double, 6, -1>> hPolys); //hPolys,corridors samesconst std::vector<Eigen::Matrix<double, 6, -1>> hPolys
void visualizeSubpointcloud(vec_Vec3f& points);
void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map);

// change the state to the new state
void changeState(STATE new_state, string pos_call) {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ"};
  int pre_s = int(exec_state);
  exec_state = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " +
              state_str[int(new_state)]
       << endl;
}

void printState() {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ"};
  cout << "[Clock]: state: " + state_str[int(exec_state)] << endl;
}

void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  odom_pt(0) = odom->pose.pose.position.x;
  odom_pt(1) = odom->pose.pose.position.y;
  odom_pt(2) = odom->pose.pose.position.z;

  odom_vel(0) = odom->twist.twist.linear.x;
  odom_vel(1) = odom->twist.twist.linear.y;
  odom_vel(2) = odom->twist.twist.linear.z;

  has_odom = true;
}

// Control the State changes
void execCallback(const ros::TimerEvent &e) {
  static int num = 0;
  num++;
  if (num == 100) {
    printState();
    if (!has_odom)
      cout << "no odom." << endl;
    if (!has_target)
      cout << "wait for goal." << endl;
    num = 0;
  }

  switch (exec_state) {
  case INIT: {
    if (!has_odom)
      return;
    if (!has_target)
      return;
    changeState(WAIT_TARGET, "STATE");
    break;
  }

  case WAIT_TARGET: {
    if (!has_target)
      return;
    else
      changeState(GEN_NEW_TRAJ, "STATE");
    break;
  }

  case GEN_NEW_TRAJ: {
    bool success = trajGeneration();
    if (success)
      changeState(EXEC_TRAJ, "STATE");
    else
      changeState(GEN_NEW_TRAJ, "STATE");
    break;
  }

  case EXEC_TRAJ: {
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - time_traj_start).toSec();
    double t_replan = ros::Duration(100000, 0).toSec();
    t_cur = min(time_duration, t_cur); // time_duration = _polyTime.sum(); in trajGen

    if (t_cur > time_duration - 1e-2) {
      has_target = false;
      changeState(WAIT_TARGET, "STATE");
      return;
    } else if ((target_pt - odom_pt).norm() < no_replan_thresh) { //???
      return;
    } else if ((start_pt - odom_pt).norm() < replan_thresh) {
      return;
    } else if (t_cur < t_replan) {
      return;
    } else {
      changeState(REPLAN_TRAJ, "STATE");
    }
    break;
  }
  case REPLAN_TRAJ: {
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - time_traj_start).toSec();
    double t_delta = ros::Duration(0, 50).toSec(); //???
    t_cur = t_delta + t_cur;
    start_pt = getPos(t_cur);// get pos from t???
    start_vel = getVel(t_cur);
    bool success = trajGeneration();
    if (success)
      changeState(EXEC_TRAJ, "STATE");
    else
      changeState(GEN_NEW_TRAJ, "STATE");
    break;
  }
  }
}

vec_Vec3f cloud_to_vec(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  vec_Vec3f pts;
  pts.resize(cloud.points.size());
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    pts[i](0) = cloud.points[i].x;
    pts[i](1) = cloud.points[i].y;
    pts[i](2) = cloud.points[i].z;
  }
  return pts;
}

std::vector<Eigen::Matrix<double, 6, -1>> polyhTypeConverter(
    vec_E<Polyhedron<3>> vs) {
  std::vector<Eigen::Matrix<double, 6, -1>> polys;
  polys.reserve(vs.size());//!
  for (const auto& v : vs) {
    Eigen::MatrixXd poly;
    poly.resize(6, v.hyperplanes().size());
    int i = 0;
    for (const auto& p : v.hyperplanes()) {
      poly.col(i).head<3>() = p.p_; //modified: tial<->head
      poly.col(i).tail<3>() = p.n_;
      i++;
    }
    polys.push_back(poly);
  }
  return polys;
}

void rcvWaypointsCallBack(const nav_msgs::Path &wp) {
  if (wp.poses[0].pose.position.z < 0.0)
    return;
  target_pt << wp.poses[0].pose.position.x, wp.poses[0].pose.position.y,
      wp.poses[0].pose.position.z; 
  ROS_INFO("[node] receive the planning target");
  start_pt = odom_pt;
  start_vel = odom_vel;
  has_target = true;
  std::cout<<"start_pt: "<<start_pt(0)<<","<<start_pt(1)<<","<<start_pt(2)<<std::endl;
  std::cout<<"target_pt: "<<target_pt(0)<<","<<target_pt(1)<<","<<target_pt(2)<<std::endl;
  if (exec_state == WAIT_TARGET)
    changeState(GEN_NEW_TRAJ, "STATE");
  else if (exec_state == EXEC_TRAJ)
    changeState(REPLAN_TRAJ, "STATE");

  for (int k = 0; k < (int)wp.poses.size(); k++) {
    Eigen::Vector3d pt(wp.poses[k].pose.position.x, wp.poses[k].pose.position.y,
                      wp.poses[k].pose.position.z);
    Vec3f ptf(wp.poses[k].pose.position.x, wp.poses[k].pose.position.y,
          wp.poses[k].pose.position.z);
    waypointsf.push_back(ptf);
  }

}

vec_Vec3f getSubPointClouds(const pcl::PointCloud<pcl::PointXYZ> &pc, const vec_Vec3f &wp) { //vec_Vec3f const vector<Vector3d>
  
  // Define conpare function，for deduplication in set
  struct Vec3fCompare {
      bool operator()(const Vec3f& a, const Vec3f& b) const {
          // Set a small threshold to prevent floating-point accuracy issues
          const float epsilon = 1e-5;
          if (std::abs(a[0] - b[0]) > epsilon) return a[0] < b[0];
          if (std::abs(a[1] - b[1]) > epsilon) return a[1] < b[1];
          if (std::abs(a[2] - b[2]) > epsilon) return a[2] < b[2];
          return false;
      }
  };

  // restore unique_points
  std::set<Vec3f, Vec3fCompare> unique_points;
  
  vec_Vec3f pc_out;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr new_pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr old_pc(new pcl::PointCloud<pcl::PointXYZ>);
  *old_pc = pc;
  std::cout << "Old Point Cloud Size: " << pc.size() << std::endl;
  int n = wp.size();
  std::cout << "n Size: " << n << std::endl;
  for (int i = 1; i < n; i++) {
    std::cout<<"in loop, i= "<<i<<std::endl;
    Vec3f p_now = wp[i];// modified, origin wp[i]
    Vec3f p_prev = wp[i - 1];
    double xmin, xmax, ymin, ymax, zmin, zmax;
  
    xmin = std::min(p_now[0], p_prev[0]) - 3; //modified origin: 0.5
    xmax = std::max(p_now[0], p_prev[0]) + 3;
    ymin = std::min(p_now[1], p_prev[1]) - 3;
    ymax = std::max(p_now[1], p_prev[1]) + 3;
    zmin = std::min(p_now[2], p_prev[2]) - 3;
    zmax = std::max(p_now[2], p_prev[2]) + 3;

    pcl::PointCloud<pcl::PointXYZ>::Ptr new_pc(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x .setInputCloud(old_pc);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(xmin, xmax);
    pass_x.filter(*new_pc);
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y .setInputCloud(new_pc);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(ymin, ymax);
    pass_y.filter(*new_pc);
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z .setInputCloud(new_pc);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(zmin, zmax);
    pass_z.filter(*new_pc);

    vec_Vec3f new_vec = cloud_to_vec(*new_pc);
    std::cout << "new_vecd Size: " << new_vec.size() << std::endl;
    for (const auto& point : new_vec) {
        unique_points.insert(point);
    }
  }

  // 将set中的点转换回vector
  pc_out.assign(unique_points.begin(), unique_points.end());
  std::cout << "New Point Cloud Size: " << pc_out.size() << std::endl;
  return pc_out;
}

void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map){
  if (has_global_map)
    return;
  pcl::fromROSMsg(pointcloud_map, global_cloud);
  has_global_map = true;
  ROS_WARN("In traj_gen_node Global Pointcloud received..");
  std::cout<<"Global Pointcloud Size: "<<global_cloud.size()<<std::endl; 

  return;
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map) {

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_vis;
  sensor_msgs::PointCloud2 map_vis;

  pcl::fromROSMsg(pointcloud_map, cloud); // modified origin no * // in hw6: map_ptr_->initFromPointCloud(msg);

  if ((int)cloud.points.size() == 0)
    return;

  pcl::PointXYZ pt;
  for (int idx = 0; idx < (int)cloud.points.size(); idx++) {
    pt = cloud.points[idx];
    // set obstalces into grid map for path planning
    _astar_path_finder->setObs(pt.x, pt.y, pt.z);
  }
}

// trajectory generation: front-end + back-end
// front-end : A* search method
// back-end  : Minimum snap trajectory generation
bool trajGeneration() {
  /**
   *
   * STEP 1:  search the path and get the path
   *
   * **/
  cout <<"***start trajGeneration***"<< endl; // for test
  _astar_path_finder->AstarGraphSearch(start_pt, target_pt);
  auto grid_path = _astar_path_finder->getPath(); //vector<Vector3d> path;

  // Reset map for next call

  /**
   *
   * STEP 2:  Simplify the path: use the RDP algorithm
   *
   * **/
  auto simp_grid_path = _astar_path_finder->pathSimplify(grid_path, _path_resolution); //selfadd:vector<Vector3d> subPath;
  cout <<"***after pathSimplify***"<< endl; // for test

  MatrixXd path(int(simp_grid_path.size()), 3);
  for (int k = 0; k < int(simp_grid_path.size()); k++) {
    path.row(k) = simp_grid_path[k]; // ??? why this? // modify for test, simp_grid_path-> grid_path
  }

  vec_Vec3f grid_pathf;
  for(int k= 0; k< simp_grid_path.size(); k++){
      Vec3f ptf(simp_grid_path[k](0), simp_grid_path[k](1),
              simp_grid_path[k](2));
      grid_pathf.push_back(ptf);        
  }

  MatrixXd path_astar(int(grid_path.size()), 3);
  for (int k = 0; k < int(grid_path.size()); k++) {
    path_astar.row(k) = grid_path[k]; //
  }
  try{
    visPath_astar(path_astar);
  } catch (const std::exception& e) {
    cout<<"[Debug] vispath_astar failed"<< endl;
  }

  /**
   *
   * STEP 3:  Trajectory optimization
   *
   * **/
  trajOptimization(path,simp_grid_path,grid_pathf);
  cout <<"***after trajOptimization***"<< endl; // for test
  time_duration = _polyTime.sum();
  cout <<"***after time_duration calculation***"<< endl; // for test
  // Publish the trajectory
  trajPublish(_polyCoeff, _polyTime);
  cout <<"***after trajPublish***"<< endl; // for test
  // record the trajectory start time
  time_traj_start = ros::Time::now();
  // return if the trajectory generation successes
  if (_polyCoeff.rows() > 0)
    return true;
  else
    return false;
}

void trajOptimization(Eigen::MatrixXd path, std::vector<Eigen::Vector3d> grid_path,vec_Vec3f grid_pathf ) {
  cout <<"***start trajOptimization***"<< endl; // for test
  // if( !has_odom ) return;
  MatrixXd vel = MatrixXd::Zero(2, 3);
  MatrixXd acc = MatrixXd::Zero(2, 3);

  vel.row(0) = start_vel;

  /**
   *
   * STEP 3.1:  finish the timeAllocation() using resonable allocation
   *
   * **/
  cout << "[Debug] Computing time allocation" << endl; //for test
  _polyTime = timeAllocation(path); // in TimeAllocation: VectorXd time(Path.rows() - 1);
  cout << "[Debug] Time allocation completed with " << _polyTime.size() << " segments" << endl; // for test

  /**
   *
   * STEP 3.2:  generate a minimum-jerk piecewise monomial polynomial-based
   * trajectory
   *
   * **/
  //get corridor 
  std::vector<Eigen::Matrix<double, 6, -1>> corridors;
  EllipsoidDecomp3D decomp_util;
  observations = getSubPointClouds(global_cloud, grid_pathf); //vec_Vec3f observations; //selfadd:vector<Vector3d>
  ROS_INFO_STREAM("observations size: " << observations.size());
  visualizeSubpointcloud(observations);
  decomp_util.set_obs(observations);
  decomp_util.set_local_bbox(Vec3f(0.8, 1.5, 1));
  decomp_util.dilate(grid_pathf);
  corridors = polyhTypeConverter(decomp_util.get_polyhedrons());
  /* clean buffer */
  ROS_INFO_STREAM("corridor size: " << corridors.size());
  visualizeCorridors(corridors);
  visualizeCorridors2(corridors);

  try {
    _polyCoeff = _trajGene->CorridorMinsnapSolve(_dev_order, path, _Vel, _Acc, _polyTime,corridors);
  } catch (const std::exception& e) {
    cout << "[Error] Exception in PolyQPGeneration: " << e.what() << endl; //for test
    return;
  }
  
  // // check if the trajectory is safe, if not, do reoptimize
  int unsafe_segment;

  // /**
  //  *
  //  * STEP 3.3:  finish the safeCheck()
  //  *
  //  * **/
  // // cout << "[Debug] doing Safety check" << endl; // for test
  unsafe_segment = _astar_path_finder->safeCheck(_polyCoeff, _polyTime); //return int
  cout << "[Debug] Safety check complete. Unsafe segment: " << unsafe_segment << endl; // for test
  MatrixXd repath = path;
  // int count = 0;
  // while (unsafe_segment != -1) {
  //   cout <<"***inside trajOptimization, start reoptimize***"<< endl; // for test
  //   /**
  //    *
  //    * STEP 3.4:  reoptimize
  //    * the method comes from: "Polynomial Trajectory
  //    * Planning for Aggressive Quadrotor Flight in Dense Indoor Environment"
  //    * part 3.5
  //    *
  //    * **/
  //   //找到第seg段segment两端点(第seg个点，和seg+1)
  //   //生成中点
  //   //加入path,在第seg后面(如果从零计数的话) //MatrixXd path(int(grid_path.size()), 3); 
  //   //_polyTime，在第seg后面(如果从零计数的话) //in TimeAllocation: VectorXd time(Path.rows() - 1);  

  //   // Get endpoints of unsafe segment
  //   Vector3d start_point = path.row(unsafe_segment);
  //   Vector3d end_point = path.row(unsafe_segment + 1);
    
  //   // Generate midpoint
  //   Vector3d mid_point = (start_point + end_point) / 2.0;
    
  //   // Insert midpoint into path
  //   MatrixXd new_path(path.rows() + 1, 3);
  //   new_path.topRows(unsafe_segment + 1) = path.topRows(unsafe_segment + 1);
  //   new_path.row(unsafe_segment + 1) = mid_point;
  //   new_path.bottomRows(path.rows() - (unsafe_segment + 1)) = 
  //       path.bottomRows(path.rows() - (unsafe_segment + 1));
  //   cout<<"old path size: "<<path.rows()<<endl; // for test
  //   path = new_path; //!!!
  //   cout<<"new path size: "<<new_path.rows()<<endl; // for test

  //   // Insert midpoint into grid_path
  //   vector<Vector3d> new_grid_path(grid_path.size()+1);
  //   int middleIndex = unsafe_segment + 1;
  //   grid_path.insert(grid_path.begin() + middleIndex, mid_point);

  //   // Update time allocation
  //   VectorXd new_time(_polyTime.size() + 1);
  //   new_time.head(unsafe_segment) = _polyTime.head(unsafe_segment);
  //   // Split the time of unsafe segment into two parts
  //   new_time(unsafe_segment) = _polyTime(unsafe_segment) / 1.5;
  //   new_time(unsafe_segment + 1) = _polyTime(unsafe_segment) / 1.5;
  //   new_time.tail(_polyTime.size() - unsafe_segment - 1) = 
  //       _polyTime.tail(_polyTime.size() - unsafe_segment - 1);
  //   cout<<"old time size: "<<_polyTime.size()<<endl; // for test
  //   _polyTime = new_time;
  //   cout<<"new time size: "<<new_time.size()<<endl; // for test
    
    
  //   // Reoptimize trajectory with new waypoints
  //   // _polyCoeff = getPolyCoeff(path, _polyTime);
  //   // get corridors again
  //   observations = getSubPointClouds(global_cloud, grid_path); //vec_Vec3f observations; 
  //   ROS_INFO_STREAM("observations size: " << observations.size());
  //   visualizeSubpointcloud(observations);
  //   decomp_util.set_obs(observations);
  //   decomp_util.set_local_bbox(Vec3f(10, 10, 10));
  //   decomp_util.dilate(grid_path);//selfadd:vector<Vector3d>
  //   corridors = polyhTypeConverter(decomp_util.get_polyhedrons());
  //   /* clean buffer */
  //   ROS_INFO_STREAM("corridor size: " << corridors.size());
  //   visualizeCorridors(corridors);
  //   visualizeCorridors2(corridors);
  //   _trajGene->CorridorMinsnapSolve(_dev_order, path, _Vel, _Acc, _polyTime,corridors);

  //   count=count+1;
  //   cout<<"count: "<<count<<endl; // for tests
    
  //   // Check safety of new trajectory
  //   unsafe_segment = _astar_path_finder->safeCheck(_polyCoeff, new_time);

  // }
  // std::cout<<"***inside trajOptimization, after reoptimize***"<<"count="<< count << std::endl; // for test
  // visulize path and trajectory
  try{
    visPath(repath);
  } catch (const std::exception& e) {
    cout<<"[Debug] visPath failed"<< endl;
  }
  try{
  visTrajectory(_polyCoeff, _polyTime);
  } catch (const std::exception& e) {
    cout<<"[Debug] visTrajectory failed"<< endl;
  }
}

void trajPublish(MatrixXd polyCoeff, VectorXd time) {
  if (polyCoeff.size() == 0 || time.size() == 0) {
    ROS_WARN("[trajectory_generator_waypoint] empty trajectory, nothing to "
             "publish.");
    return;
  }
  cout<<"[Debug] inside trajPublish , just start"<< endl;

  unsigned int poly_number;

  static int count = 1; // The first trajectory_id must be greater than 0. zxzxzxzx

  quadrotor_msgs::PolynomialTrajectory traj_msg;

  traj_msg.header.seq = count;
  traj_msg.header.stamp = ros::Time::now();
  traj_msg.header.frame_id = std::string("world"); //??? // modified
  traj_msg.trajectory_id = count;
  traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;

  traj_msg.num_order = 2 * _dev_order - 1; // the order of polynomial
  traj_msg.num_segment = time.size();

  Vector3d initialVel, finalVel;
  initialVel = _trajGene->getVelPoly(_polyCoeff, 0, 0);
  finalVel = _trajGene->getVelPoly(_polyCoeff, traj_msg.num_segment - 1,
                                   1);//！modified,origin:,_polyTime(traj_msg.num_segment - 1))
  traj_msg.start_yaw = atan2(initialVel(1), initialVel(0));
  traj_msg.final_yaw = atan2(finalVel(1), finalVel(0));
  poly_number = traj_msg.num_order + 1;

  // if (traj_msg.num_segment > polyCoeff.rows()) {
  //   // cerr << "num_segment is larger than matrix rows" << endl; // for test
  //   // Handle error
  // }

  // if (poly_number * 3 > polyCoeff.cols()) {
  //   // cerr << "poly_number requires more columns than available" << endl; // for test
  //   // Handle error
  // }

  for (unsigned int i = 0; i < traj_msg.num_segment; i++) {
    if (i >= polyCoeff.rows()) {
      cerr << "Error: i=" << i << " exceeds matrix rows=" << polyCoeff.rows() << endl;
      break;
    }
    for (unsigned int j = 0; j < poly_number; j++) {
        if (j*3+2 >= polyCoeff.cols()) {
          cerr << "Error: column index " << j*3+2 << " exceeds matrix cols=" << polyCoeff.cols() << endl;
          break;
        }
        
      // cout<<"[Debug] inside trajPublish , inside for loop, start calculate coeff_x/y/z"<< endl;
      // traj_msg.coef_x.push_back(polyCoeff(i, j) * pow(time(i), j));
      // traj_msg.coef_y.push_back(polyCoeff(i, poly_number + j) *
      //                           pow(time(i), j));
      // traj_msg.coef_z.push_back(polyCoeff(i, 2 * poly_number + j) *
      //                           pow(time(i), j));
      traj_msg.coef_x.push_back(polyCoeff(i, j));
      traj_msg.coef_y.push_back(polyCoeff(i, poly_number + j));
      traj_msg.coef_z.push_back(polyCoeff(i, 2 * poly_number + j));
      // cout<<"[Debug] inside trajPublish for loop, assigning traj_msg.coef_x/y/z complited, i= "<<i<<"j= "<<j<< endl;
    }
    traj_msg.time.push_back(time(i));
    traj_msg.order.push_back(traj_msg.num_order);
  }
  // cout<<"[Debug] inside trajPublish , after for loop"<< endl;
  traj_msg.mag_coeff = 1;

  count++;
  ROS_WARN("[traj..gen...node] traj_msg publish");
  _traj_pub.publish(traj_msg);
}

VectorXd timeAllocation(MatrixXd Path) {
  if(Path.rows()<1){
    VectorXd time;
    time<<0,0,0;
    std::cout<<"[Debug]no path for allocation!"<<std::endl;
    return time;
  }
  VectorXd time(Path.rows() - 1);
  double t = _Vel/_Acc;
  double dist_thred = _Acc*t*t;
  double time_segment;

  for(int i = 1;i<Path.rows();i++){
    double dist = (Path.row(i)-Path.row(i-1)).norm();
    if(dist > dist_thred){
      time_segment = (dist - dist_thred) / _Vel + 2*t;
    }else{
      time_segment =2*std::sqrt(dist/_Acc);
    }
    time[i-1] = time_segment;
    // 添加数值范围检查
    if (std::abs(time[i-1]) < 1e-6) {
        ROS_WARN("Time segment %d too small: %f", i, time[i-1]);
        time[i-1] = 1e-6;
    }

    // 输出调试信息
    ROS_INFO("Segment %d time: %f ", 
            i, time[i-1]);
  }
  return time;
}

void visualizeCorridors2(const std::vector<Eigen::Matrix<double, 6, -1>> hPolys) {
  decomp_ros_msgs::PolyhedronArray poly_msg;
  for (int i = 0; i < hPolys.size(); i++) {
    Eigen::MatrixXd hpoly = hPolys[i];
    decomp_ros_msgs::Polyhedron msg;
    for (int j = 0; j < hpoly.cols(); j++) {
      geometry_msgs::Point pt, n;
      pt.x = hpoly.col(j)(0);
      pt.y = hpoly.col(j)(1);
      pt.z = hpoly.col(j)(2);
      n.x = hpoly.col(j)(3);
      n.y = hpoly.col(j)(4);
      n.z = hpoly.col(j)(5);
      msg.points.push_back(pt);
      msg.normals.push_back(n);
    }
    poly_msg.polyhedrons.push_back(msg);
  }
  poly_msg.header.frame_id = "world";
  poly_msg.header.stamp = ros::Time::now();
  corridor_pub2.publish(poly_msg);
}
void visualizeCorridors(const std::vector<Eigen::Matrix<double, 6, -1>>& corridors) {
  // delete old markers
  visualization_msgs::MarkerArray delete_array;
  visualization_msgs::Marker delete_marker;
  delete_marker.header.frame_id = "world";
  delete_marker.header.stamp = ros::Time::now();
  delete_marker.ns = "corridor_faces";
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  delete_array.markers.push_back(delete_marker);
  corridor_pub.publish(delete_array);

  visualization_msgs::MarkerArray marker_array;
  std::vector<std::vector<float>> colors = {
    {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, 
    {1.0, 1.0, 0.0}, {1.0, 0.0, 1.0}, {0.0, 1.0, 1.0}
  };

  for (size_t i = 0; i < corridors.size(); ++i) {
    const auto& faces = corridors[i];
    
    visualization_msgs::Marker corridor_marker;
    corridor_marker.header.frame_id = "world";
    corridor_marker.header.stamp = ros::Time::now();
    corridor_marker.ns = "corridor_faces";
    corridor_marker.id = i;
    corridor_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    corridor_marker.action = visualization_msgs::Marker::ADD;
    corridor_marker.pose.orientation.w = 1.0;
    
    int color_index = i % colors.size();
    corridor_marker.color.r = colors[color_index][0];
    corridor_marker.color.g = colors[color_index][1];
    corridor_marker.color.b = colors[color_index][2];
    corridor_marker.color.a = 0.15;
    
    corridor_marker.scale.x = 1.0;
    corridor_marker.scale.y = 1.0;
    corridor_marker.scale.z = 1.0;

    // surface function
    std::vector<Eigen::Vector4d> planes;
    for (int j = 0; j < faces.cols(); ++j) {
      Eigen::Vector3d normal = faces.block<3,1>(3,j);
      Eigen::Vector3d point = faces.block<3,1>(0,j);
      point = point-0.2*normal;
      normal.normalize();
      double d = -normal.dot(point);
      planes.push_back(Eigen::Vector4d(normal.x(), normal.y(), normal.z(), d));
    }

    // Calculate intersection point
    std::vector<Eigen::Vector3d> vertices;
    for (size_t j = 0; j < planes.size(); ++j) {
      for (size_t k = j + 1; k < planes.size(); ++k) {
        for (size_t l = k + 1; l < planes.size(); ++l) {
          Eigen::Matrix3d A;
          A.row(0) = planes[j].head<3>();
          A.row(1) = planes[k].head<3>();
          A.row(2) = planes[l].head<3>();
          
          double det = A.determinant();
          if (std::abs(det) > 1e-6) {
            Eigen::Vector3d b(-planes[j](3), -planes[k](3), -planes[l](3));
            Eigen::Vector3d p = A.inverse() * b;
            
            bool is_valid = true;
            for (const auto& plane : planes) {
              if (plane.head<3>().dot(p) + plane(3) > 1e-6) {
                is_valid = false;
                break;
              }
            }
            
            if (is_valid) {
              // Check for duplication
              bool is_new = true;
              for (const auto& v : vertices) {
                if ((v - p).norm() < 1e-6) {
                  is_new = false;
                  break;
                }
              }
              if (is_new) {
                vertices.push_back(p);
              }
            }
          }
        }
      }
    }

    // Caculate center point
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto& v : vertices) {
      center += v;
    }
    center /= vertices.size();

    // organize vertex point according to  surface
    struct Face {
      std::vector<int> vertex_indices;
      Eigen::Vector3d normal;
    };
    std::vector<Face> polygon_faces;

    // For every original surface，find vertex point on it
    for (size_t j = 0; j < planes.size(); ++j) {
      Face face;
      face.normal = planes[j].head<3>();
      
      // find all vertex point on this surface
      for (size_t k = 0; k < vertices.size(); ++k) {
        if (std::abs(face.normal.dot(vertices[k]) + planes[j](3)) < 1e-6) {
          face.vertex_indices.push_back(k);
        }
      }

      // ROS_INFO("Corridor %zu has %zu vertices", i, vertices.size());
      // for (size_t j = 0; j < vertices.size(); ++j) {
      //   ROS_INFO("Vertex %zu: (%.3f, %.3f, %.3f)", 
      //            j, vertices[j].x(), vertices[j].y(), vertices[j].z());
      // }

      // If at least 3 points，thats a valid surface
      if (face.vertex_indices.size() >= 3) {
        // make sure in counterclockwise
        Eigen::Vector3d face_center = Eigen::Vector3d::Zero();
        for (int idx : face.vertex_indices) {
          face_center += vertices[idx];
        }
        face_center /= face.vertex_indices.size();
      
        // create a local coordinate system
        Eigen::Vector3d local_x = (vertices[face.vertex_indices[0]] - face_center).normalized();
        Eigen::Vector3d local_y = face.normal.cross(local_x);

        // Sort by angle
        std::sort(face.vertex_indices.begin(), face.vertex_indices.end(),
          [&](int a, int b) {
            Eigen::Vector3d va = vertices[a] - face_center;
            Eigen::Vector3d vb = vertices[b] - face_center;
            double angle_a = std::atan2(va.dot(local_y), va.dot(local_x));
            double angle_b = std::atan2(vb.dot(local_y), vb.dot(local_x));
            return angle_a < angle_b;
          });

        polygon_faces.push_back(face);
      }
      // ROS_INFO("Corridor %zu has %zu faces", i, polygon_faces.size());
      // for (size_t j = 0; j < polygon_faces.size(); ++j) {
      //   ROS_INFO("Face %zu has %zu vertices", 
      //            j, polygon_faces[j].vertex_indices.size());
      // }
    }

    // Perform triangulation on each face
    for (const auto& face : polygon_faces) {
      for (size_t j = 1; j < face.vertex_indices.size() - 1; ++j) {
        geometry_msgs::Point p1, p2, p3;
        
        // The first vertex is always the first point
        p1.x = vertices[face.vertex_indices[0]].x();
        p1.y = vertices[face.vertex_indices[0]].y();
        p1.z = vertices[face.vertex_indices[0]].z();
        
        // The other two vertices are adjacent points
        p2.x = vertices[face.vertex_indices[j]].x();
        p2.y = vertices[face.vertex_indices[j]].y();
        p2.z = vertices[face.vertex_indices[j]].z();
        
        p3.x = vertices[face.vertex_indices[j+1]].x();
        p3.y = vertices[face.vertex_indices[j+1]].y();
        p3.z = vertices[face.vertex_indices[j+1]].z();

        corridor_marker.points.push_back(p1);
        corridor_marker.points.push_back(p2);
        corridor_marker.points.push_back(p3);
      }
    }

    if (!corridor_marker.points.empty()) {
      marker_array.markers.push_back(corridor_marker);
    }
  }

  corridor_pub.publish(marker_array);
}

void visualizeSubpointcloud(vec_Vec3f& points) {
    
    visualization_msgs::MarkerArray marker_array;
    
    // careate a marker that delete all old corridor
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "world";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "traj_node/sub_points";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    // create a new marker
    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = "world";  // 坐标系
    points_marker.header.stamp = ros::Time::now();
    points_marker.ns = "traj_node/sub_points";
    points_marker.id = 1;
    points_marker.type = visualization_msgs::Marker::POINTS;
    points_marker.action = visualization_msgs::Marker::ADD;
    
    // set size
    points_marker.scale.x = 0.05;  // 可以调整这个值改变点的大小
    points_marker.scale.y = 0.05;
    
    // set color (here blue）
    points_marker.color.r = 1.0;
    points_marker.color.g = 0.0;
    points_marker.color.b = 0.0;
    points_marker.color.a = 1.0;
    
    // set marker life circle（Optional）
    points_marker.lifetime = ros::Duration();  // constantly show
    
    // add all points into marker
    for (const auto& point : points) {
        geometry_msgs::Point p;
        p.x = point(0);
        p.y = point(1);
        p.z = point(2);
        points_marker.points.push_back(p);
    }
    
    // add points_marker into marker_array
    marker_array.markers.push_back(points_marker);
        
    // publish marker
    subpointclouds_pub.publish(marker_array);
}



void visTrajectory(MatrixXd polyCoeff, VectorXd time) {
  visualization_msgs::Marker _traj_vis;

  _traj_vis.header.stamp = ros::Time::now();
  _traj_vis.header.frame_id = "world"; // modified

  _traj_vis.ns = "traj_node/trajectory";
  _traj_vis.id = 0;
  _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
  _traj_vis.action = visualization_msgs::Marker::ADD;
  _traj_vis.scale.x = _vis_traj_width;
  _traj_vis.scale.y = _vis_traj_width;
  _traj_vis.scale.z = _vis_traj_width;
  _traj_vis.pose.orientation.x = 0.0;
  _traj_vis.pose.orientation.y = 0.0;
  _traj_vis.pose.orientation.z = 0.0;
  _traj_vis.pose.orientation.w = 1.0;

  _traj_vis.color.a = 1.0;
  _traj_vis.color.r = 0.0;
  _traj_vis.color.g = 128.0;
  _traj_vis.color.b = 0.0;

  _traj_vis.points.clear();
  Vector3d pos;
  geometry_msgs::Point pt;

  for (int i = 0; i < time.size(); i++) {
    for (double t = 0.0; t < 1; t += 0.01) {//! modified, origin: for (double t = 0.0; t < time(i); t += 0.01)
      pos = _trajGene->getPosPoly(polyCoeff, i, t);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = pos(2);
      _traj_vis.points.push_back(pt);
    }
  }
  _traj_vis_pub.publish(_traj_vis);
}

void visPath(MatrixXd nodes) {
  visualization_msgs::Marker points;

  int id = 0;
  points.id = id;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.header.frame_id = "world";
  points.header.stamp = ros::Time::now();
  points.ns = "traj_node/path";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  points.scale.z = 0.2;
  points.color.a = 1.0;
  points.color.r = 0.0;
  points.color.g = 0.0;
  points.color.b = 1.0;

  geometry_msgs::Point p;
  for (int i = 0; i < int(nodes.rows()); i++) {
    p.x = nodes(i, 0);
    p.y = nodes(i, 1);
    p.z = nodes(i, 2);

    points.points.push_back(p);
  }
  _path_vis_pub.publish(points);
}

void visPath_astar(MatrixXd nodes) {
  visualization_msgs::Marker points;

  int id = 0;
  points.id = id;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.header.frame_id = "world";
  points.header.stamp = ros::Time::now();
  points.ns = "traj_node/path_atar";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  points.scale.z = 0.1;
  points.color.a = 0.7;
  points.color.r = 0.0;
  points.color.g = 0.0;
  points.color.b = 1.0;

  geometry_msgs::Point p;
  for (int i = 0; i < int(nodes.rows()); i++) {
    p.x = nodes(i, 0);
    p.y = nodes(i, 1);
    p.z = nodes(i, 2);

    points.points.push_back(p);
  }
  _astar_vis_pub.publish(points);
}

Vector3d getPos(double t_cur) {
  double time = 0;
  Vector3d pos = Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {
      time = time + 0.01;
      if (time > t_cur) {
        pos = _trajGene->getPosPoly(_polyCoeff, i, t/_polyTime(i));
        return pos;
      }
    }
  }
  return pos;
}

Vector3d getVel(double t_cur) {
  double time = 0;
  Vector3d Vel = Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {//!modified,origin: for (double t = 0.0; t < _polyTime(i); t += 0.01)
      time = time + 0.01;
      if (time > t_cur) {
        Vel = _trajGene->getVelPoly(_polyCoeff, i, t/_polyTime(i));
        return Vel;
      }
    }
  }
  return Vel;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_node");
  ros::NodeHandle nh("~");

  nh.param("planning/vel", _Vel, 1.0);
  nh.param("planning/acc", _Acc, 1.0);
  nh.param("planning/dev_order", _dev_order, 3);
  nh.param("planning/min_order", _min_order, 3);
  nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
  nh.param("map/resolution", _resolution, 0.2);
  nh.param("map/x_size", _x_size, 50.0);
  nh.param("map/y_size", _y_size, 50.0);
  nh.param("map/z_size", _z_size, 5.0);
  nh.param("path/resolution", _path_resolution, 0.05);
  nh.param("replanning/thresh_replan", replan_thresh, -1.0);
  nh.param("replanning/thresh_no_replan", no_replan_thresh, -1.0);

  _poly_num1D = 2 * _dev_order;

  _exec_timer = nh.createTimer(ros::Duration(0.01), execCallback);

  _odom_sub = nh.subscribe("odom", 10, rcvOdomCallback);
  _map_sub = nh.subscribe("local_pointcloud", 1, rcvPointCloudCallBack);
  _pts_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallBack);
  _glb_pc_sub = nh.subscribe("/random_complex/global_map", 1, rcvGlobalPointCloudCallBack); //selfadd

  _traj_pub =
      nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);
  _traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
  _path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_path", 1);
  _astar_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_astar_path", 1);
  corridor_pub = nh.advertise<visualization_msgs::MarkerArray>("vis_corridor", 1);
  corridor_pub2 = nh.advertise<decomp_ros_msgs::PolyhedronArray>("vis_corridor2", 1,true);
  subpointclouds_pub = nh.advertise<visualization_msgs::MarkerArray>("vis_subPointcloud", 60000);

  // set the obstacle map
  _map_lower << -_x_size / 2.0, -_y_size / 2.0, 0.0;
  _map_upper << +_x_size / 2.0, +_y_size / 2.0, _z_size;
  _inv_resolution = 1.0 / _resolution;
  _max_x_id = (int)(_x_size * _inv_resolution);
  _max_y_id = (int)(_y_size * _inv_resolution);
  _max_z_id = (int)(_z_size * _inv_resolution);

  _astar_path_finder = new AstarPathFinder();
  _astar_path_finder->initGridMap(_resolution, _map_lower, _map_upper,
                                  _max_x_id, _max_y_id, _max_z_id);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }

  return 0;
}