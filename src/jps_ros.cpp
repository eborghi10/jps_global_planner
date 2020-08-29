/*
 * Copyright (c) 2020, Emiliano Javier Borghi Orue.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Emiliano Javier Borghi Orue nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <jps_global_planner/jps_ros.h>
#include <jps_basis/data_utils.h>

#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <algorithm>
#include <iterator>
#include <string>
#include <vector>


// Register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(jps::JumpPointSearchROS, nav_core::BaseGlobalPlanner)

namespace jps
{
JumpPointSearchROS::JumpPointSearchROS()
    : costmap_(NULL)
    , jps_planner_()
    , dmp_planner_()
    , initialized_(false) {}

JumpPointSearchROS::JumpPointSearchROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_(NULL)
    , jps_planner_()
    , dmp_planner_()
    , initialized_(false)
{
  // Initialize the planner
  initialize(name, costmap_ros);
}

JumpPointSearchROS::JumpPointSearchROS(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
    : costmap_(NULL)
    , jps_planner_()
    , dmp_planner_()
    , initialized_(false)
{
  // Initialize the planner
  initialize(name, costmap, global_frame);
}

JumpPointSearchROS::~JumpPointSearchROS()
{
};

void JumpPointSearchROS::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
{
  if (!initialized_)
  {
    costmap_ = costmap;
    global_frame_ = global_frame;

    ros::NodeHandlePtr pnh = boost::make_shared<ros::NodeHandle>("~/" + name);
    plan_pub_ = pnh->advertise<nav_msgs::Path>("plan", 1);

    // ROS parameters
    double dmp_potential_radius_x, dmp_potential_radius_y;
    double dmp_search_radius_x, dmp_search_radius_y;
    double dmp_potential_map_range_x, dmp_potential_map_range_y;
    pnh->param<bool>("debug", debug_, false);
    pnh->param<double>("dmp/potential_radius/x", dmp_potential_radius_x, 0.0);
    pnh->param<double>("dmp/potential_radius/y", dmp_potential_radius_y, 0.0);
    pnh->param<double>("dmp/search_radius/x", dmp_search_radius_x, 0.0);
    pnh->param<double>("dmp/search_radius/y", dmp_search_radius_y, 0.0);
    pnh->param<double>("dmp/potential_map_range/x", dmp_potential_map_range_x, 0.0);
    pnh->param<double>("dmp/potential_map_range/y", dmp_potential_map_range_y, 0.0);

    jps_path_pub_ = pnh->advertise<nav_msgs::Path>("jps_path", 1);
    pose_sub_ = pnh->subscribe<geometry_msgs::PoseStamped>("goal", 1, &JumpPointSearchROS::poseCallback, this);

    map_util_ = std::make_shared<JPS::OccMapUtil>();

    ///////////////////////////////////////////////

    const Vec2f map_origin(costmap_->getOriginX(), costmap_->getOriginY());
    const Vec2i map_dimension(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    const int32_t dimension(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY());

    unsigned char* map_array = costmap_->getCharMap();
    JPS::Tmap map_data;
    map_data.resize(dimension);
    for (unsigned int i = 0; i < dimension; i++)
      map_data[i] = map_array[i] > 0 ? 1 : 0;

    const decimal_t map_resolution = costmap_->getResolution();

    map_util_->setMap(
        map_origin,     // origin position
        map_dimension,  // number of cells in each dimension
        map_data,       // map resolution
        map_resolution);

    if (debug_) map_util_->info();

    ///////////////////////////////////////////////

    // Set up JPS planner
    jps_planner_ = std::make_shared<JPSPlanner2D>(debug_);
    jps_planner_->setMapUtil(map_util_);  // Set collision checking function
    jps_planner_->updateMap();

    dmp_planner_ = std::make_shared<DMPlanner2D>(debug_);
    // Set 2D potential field radius
    dmp_planner_->setPotentialRadius(Vec2f(dmp_potential_radius_x, dmp_potential_radius_y));
    // Set the valid search region around given path
    dmp_planner_->setSearchRadius(Vec2f(dmp_search_radius_x, dmp_search_radius_y));
    dmp_planner_->setPotentialMapRange(Vec2f(dmp_potential_map_range_x, dmp_potential_map_range_y));

    make_plan_srv_ =  pnh->advertiseService("make_plan", &JumpPointSearchROS::makePlanService, this);

    initialized_ = true;
  }
  else
    ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}

void JumpPointSearchROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  costmap_ros_ = costmap_ros;
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}


void JumpPointSearchROS::clearRobotCell(const geometry_msgs::PoseStamped& global_pose,
                                        unsigned int mx, unsigned int my)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return;
  }

  // Set the associated costs in the cost map to be free
  costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool JumpPointSearchROS::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
  makePlan(req.start, req.goal, resp.plan.poses);

  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = global_frame_;

  return true;
}

bool JumpPointSearchROS::makePlan(const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }

  // clear the plan, just in case
  plan.clear();

  // until tf can handle transforming things that are way in the past...
  // we'll require the goal to be in our global frame
  if (goal.header.frame_id != global_frame_)
  {
    ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
              global_frame_.c_str(), goal.header.frame_id.c_str());
    return false;
  }

  if (start.header.frame_id != global_frame_)
  {
    ROS_ERROR("The start pose passed to this planner must be in the %s frame. It is instead in the %s frame.",
              global_frame_.c_str(), start.header.frame_id.c_str());
    return false;
  }

  double wx = start.pose.position.x;
  double wy = start.pose.position.y;

  unsigned int mx, my;
  if (!costmap_->worldToMap(wx, wy, mx, my))
  {
    ROS_WARN_STREAM("The robot's start position is off the global costmap." <<
        "Planning will always fail, are you sure the robot has been properly localized?");
    return false;
  }

  // Clear the starting cell within the costmap because we know it can't be an obstacle
  clearRobotCell(start, mx, my);

  /**
   * World: coordinates in the "map" frame
   * Map: discretized coordinates for the occupancy grid
   *
   */
  const Vec2f start_m(mx, my);
  const Vec2f start_w(wx, wy);

  ////////////////////////////////////////////

  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  wx = goal.pose.position.x;
  wy = goal.pose.position.y;

  if (!costmap_->worldToMap(wx, wy, mx, my))
  {
    mx = 0;
    my = 0;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  const Vec2f goal_m(map_goal[0], map_goal[1]);
  const Vec2f goal_w(wx, wy);

  ROS_DEBUG("[JPS] Planning path from [%f, %f] to [%f, %f]",
      start_w[0], start_w[1], goal_w[0], goal_w[1]);

  jps_planner_->updateMap();

  bool result = jps_planner_->plan(start_w, goal_w, /*eps*/1, /*use_jps*/true);

  if (!result) return false;

  // Get the planned raw path from JPS
  const vec_Vec2f path_jps = jps_planner_->getRawPath();
  nav_msgs::Path jps_path;
  jps_path.header.frame_id = global_frame_;
  jps_path.header.stamp = ros::Time::now();
  const std::size_t jps_path_size = path_jps.size();
  jps_path.poses.resize(jps_path_size);
  for (int i=0; i < jps_path_size; i++)
  {
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = path_jps[i](0);
    msg.pose.position.y = path_jps[i](1);
    msg.pose.orientation.w = 1;
    jps_path.poses[i] = msg;
  }
  jps_path_pub_.publish(jps_path);

  // Set map util for collision checking, must be called before planning
  dmp_planner_->setMap(map_util_, start_w);

  // Compute the path given the jps path
  result = dmp_planner_->computePath(start_w, goal_w, path_jps);

  if (!result) return false;

  const vec_Vec2f path_dist = dmp_planner_->getRawPath();

  // Fill plan to be returned
  std::transform(path_dist.cbegin(), path_dist.cend(),
    std::back_inserter(plan), [this](const Vec2f& g)
    {
      geometry_msgs::PoseStamped ps;
      ps.header.stamp = ros::Time::now();
      ps.header.frame_id = global_frame_;
      ps.pose.position.x = g[0];
      ps.pose.position.y = g[1];
      return ps;
    }
  );  // NOLINT(whitespace/parens)

  // Publish the plan for visualization purposes
  publishPlan(plan, 0.0, 1.0, 0.0, 0.0);

  return !plan.empty();
}

void JumpPointSearchROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path,
                                      double r, double g, double b, double a)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return;
  }

  // Create a message for the plan
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());

  if (path.empty())
  {
    // Still set a valid frame so visualization won't hit transform issues
    gui_path.header.frame_id = global_frame_;
    gui_path.header.stamp = ros::Time::now();
  }
  else
  {
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;
  }

  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for (unsigned int i=0; i < path.size(); i++)
  {
    gui_path.poses[i] = path[i];
  }

  plan_pub_.publish(gui_path);
}

void JumpPointSearchROS::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  geometry_msgs::PoseStamped global_pose;
  costmap_ros_->getRobotPose(global_pose);
  std::vector<geometry_msgs::PoseStamped> path;
  makePlan(global_pose, *goal, path);
}

};  // namespace jps
