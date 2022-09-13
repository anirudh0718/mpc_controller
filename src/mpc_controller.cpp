#include "mpc_controller/mpc_controller.h"
#include <tf2/utils.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/footprint_helper.h>
#include <pluginlib/class_list_macros.h>
#include <cmath>
#include <angles/angles.h>

#include <algorithm>
#include <casadi/casadi.hpp>

// register this planner as a BaseGlobalPlanner plugin
// (see http://www.ros.org/wiki/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin)
PLUGINLIB_EXPORT_CLASS(mpc_controller::MPCController, nav_core::BaseLocalPlanner)

namespace mpc_controller {

	using namespace casadi;
    

    MPCController::MPCController():
    tf2_listener(tf_buffer)
    {

    }
    MPCController::~MPCController()
    {

    }

    void MPCController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
        ros::NodeHandle nh;
        m_tf = tf;
		m_cost_map = costmap_ros;
		m_base_frame = costmap_ros->getBaseFrameID();
		m_global_frame = costmap_ros->getGlobalFrameID();
		m_cost_map->getRobotPose(current_pose_);
		initial_pose_ = current_pose_;
		// make sure to update the costmap we'll use for this cycle
		costmap_2d::Costmap2D* costmap = m_cost_map->getCostmap();
		planner_util_.initialize(m_tf, costmap, m_cost_map->getGlobalFrameID());

		odom_sub = nh.subscribe<nav_msgs::Odometry>("odometry/filtered_map", 1, boost::bind(&MPCController::odomCallback, this, _1));
		goal_sub = nh.subscribe<mbf_msgs::MoveBaseActionGoal>("move_base_flex/move_base/goal",1,&MPCController::GoalCallback,this);
		odom_helper_.setOdomTopic( "odometry/filtered_map");
		initialized_ = true;
    }

    bool MPCController::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
        global_plan.clear();
		global_plan = plan;
		if (! isInitialized()) {
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}
		//when we get a new plan, we also want to clear any latch we may have on goal tolerances
		latchedStopRotateController_.resetLatching();
		goal_reached_ = false;
		return planner_util_.setPlan(plan);
    }

    bool MPCController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){

    }

    bool MPCController::isGoalReached()

	{
		
		if (! isInitialized()) {
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}
		if ( ! m_cost_map->getRobotPose(current_pose_)) {
		ROS_ERROR("Could not get robot pose");
		return false;
		}

		if (goal_reached_){
			
			ROS_INFO("GOAL Reached!");
			return true;
		}
		else{
			return false;

		}
		
	}

    void MPCController::UpdateStatechangerate(const double& vel,const double& heading,const double& omega){
        double x_dot = vel*cos(heading);
        double y_dot = vel*cos(heading);
        double theta_dot =  omega;

        dot_state_.pose.position.x = x_dot;
        dot_state_.pose.position.y = y_dot;
        dot_state_.pose.position.z = theta_dot;
    }

    void MPCController::UpdateState(){
        double xt_1 = current_state_.pose.position.x + dot_state_.pose.position.x*dt;
        double yt_1 = current_state_.pose.position.y + dot_state_.pose.position.y*dt;
        double theta_1 = current_state_.pose.position.z + dot_state_.pose.position.z*dt;

        current_state_.pose.position.x = xt_1;
        current_state_.pose.position.y = yt_1;
        current_state_.pose.position.z = theta_1;

		
		

    }

}   


