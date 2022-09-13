#ifndef MPC_CONTROLLER_H_
#define MPC_CONTROLLER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <dynamic_reconfigure/server.h>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/local_planner_limits.h>
#include <std_msgs/Float64.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/MoveBaseActionGoal.h>
#include <mbf_msgs/MoveBaseGoal.h>
#include <tf2_ros/transform_listener.h>
#include <base_local_planner/Position2DInt.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <base_local_planner/simple_trajectory_generator.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <numeric>




#endif /* INCLUDE_MPC_CONTROLLER_H_ */

namespace mpc_controller{

class MPCController : public nav_core::BaseLocalPlanner{

public:
    /**
    * @brief Default constructor of the teb plugin
    */
	MPCController();
    
    

	/**
    * @brief  Destructor of the plugin
    */
	~MPCController();

	/**
    * @brief compute the velocity commands to send to the robot
    */
	bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

	/**
    * @brief Check whether goal is reached 
    */
	bool isGoalReached() override;

	/**
    * @brief Ser the plan for the custom planner to follow
	* @param plan global plan received from the global_planner
    */
	bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
	
	/**
    * @brief Initializes the custom plugin
    * @param name The name of the instance
    * @param tf Pointer to a tf buffer
    * @param costmap_ros Cost map representing occupied and free space
    */
	void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;

	/**
    * @brief boolean to check if the planner is initialized
    */
	bool isInitialized() {
        return initialized_;
      }

private:
    /**
	 * @brief Odometry callback
	 * 
	 */
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

	/**
	 * @brief Goal callback
	 * 
	 */
	void GoalCallback(const mbf_msgs::MoveBaseActionGoal::ConstPtr& msg);

	/**
	 * @brief Update State of the model
	 * 
	 */
	void UpdateState();

	void UpdateStatechangerate(const double& vel, const double& heading, const double& omega);

	

private:
    tf2_ros::Buffer* m_tf = 0;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf2_listener;
	costmap_2d::Costmap2DROS* m_cost_map = 0;
	std::vector<geometry_msgs::PoseStamped> global_plan;

	//base_local_planner containers
    base_local_planner::LocalPlannerUtil planner_util_;
    base_local_planner::OdometryHelperRos odom_helper_;
	base_local_planner::LatchedStopRotateController latchedStopRotateController_;

    //Initialise ros Subscribers and Publishers
    ros::Subscriber odom_sub;
	ros::Subscriber goal_sub;
	ros::Publisher m_local_plan_pub;
	ros::Publisher dist_pub;

	std::string m_global_frame = "map";
	std::string m_local_frame = "odom";
	std::string m_base_frame = "base_link";

    //booleans
    bool initialized_;
	bool goal_reached_ = false; 

    //
    geometry_msgs::PoseStamped current_pose_; // current position of the robot
	geometry_msgs::PoseStamped updated_stae_; // updated stae of the model
	geometry_msgs::PoseStamped current_state_; // current state of the model
	geometry_msgs::PoseStamped dot_state_;     // rate of change of model
	geometry_msgs::PoseStamped initial_pose_; // Initial position of the robot


	// Numericals

	double dt ;

};
    
}

    
    



