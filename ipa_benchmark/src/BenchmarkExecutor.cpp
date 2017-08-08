
#include <ipa_benchmark/BenchmarkExecutor.h>

#include <moveit/version.h>
#include <eigen_conversions/eigen_msg.h>

#include <boost/regex.hpp>
#include <boost/progress.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <unistd.h>

static std::string getHostname()
{
  static const int BUF_SIZE = 1024;
  char buffer[BUF_SIZE];
  int err = gethostname(buffer, sizeof(buffer));
  if (err != 0)
    return std::string();
  else
  {
    buffer[BUF_SIZE - 1] = '\0';
    return std::string(buffer);
  }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------

//initilaize planning plugin class loader
BenchmarkExecutor::BenchmarkExecutor(const std::string& robot_description_param)
{
	psm_ = NULL;


	psm_ = new planning_scene_monitor::PlanningSceneMonitor(robot_description_param);
	planning_scene_ = psm_->getPlanningScene();

	// Initialize the class loader for planner plugin
	try
	{
		planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	}
	catch(pluginlib::PluginlibException& ex)
	{
	  ROS_FATAL_STREAM("BenchmarkExecutor --> Exception while creating planning plugin loader " << ex.what());
	}
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------

//initilaize planning plugin class loader
BenchmarkExecutor::BenchmarkExecutor(const std::string& robot_description_param, ros::NodeHandle& nh)
{
	psm_ = NULL;

	pathPlan_ = nh.advertiseService("pathPlanning", &BenchmarkExecutor::pathPlanCB, this);

	psm_ = new planning_scene_monitor::PlanningSceneMonitor(robot_description_param);
	planning_scene_ = psm_->getPlanningScene();

	// Initialize the class loader for planner plugin
	try
	{
		planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	}
	catch(pluginlib::PluginlibException& ex)
	{
	  ROS_FATAL_STREAM("BenchmarkExecutor --> Exception while creating planning plugin loader " << ex.what());
	}

	ROS_WARN("BenchmarkExecutor --> init_done");
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Clear all allocated memory
BenchmarkExecutor::~BenchmarkExecutor()
{
	delete psm_;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * initialize benchmarkExecutor class by loading planner plugin, load planner plugins for running benchmark and creat unique instance of each planner plugin
 * @param plugin_classes = avaliable plugins on parameter server which is set on yaml file
**/
void BenchmarkExecutor::initializeBenchmarkExecutor(const std::vector<std::string>& plugin_classes)
{

	//nh_.advertiseService( "Add_Obstacle", &BenchmarkExecutor::pathPlanCB, this );
	planner_interfaces_.clear();

	// Load the planning plugins
	const std::vector<std::string>& classes = planner_plugin_loader_->getDeclaredClasses();

	for (std::size_t i = 0; i < plugin_classes.size(); ++i)
	{
		std::vector<std::string>::const_iterator it = std::find(classes.begin(), classes.end(), plugin_classes[i]);
		if (it == classes.end())
		{
			ROS_ERROR("initializeBenchmarkExecutor --> Failed to find plugin_class %s", plugin_classes[i].c_str());
			return;
		}
		try
		{
			planning_interface::PlannerManagerPtr p = planner_plugin_loader_->createUniqueInstance(plugin_classes[i]);

			if	(!p->initialize(planning_scene_->getRobotModel(), ros::this_node::getName()))
				ROS_FATAL_STREAM("initializeBenchmarkExecutor --> Could not initialize planner interfaces");

			planner_interfaces_[plugin_classes[i]] = p;
		}

		catch (pluginlib::PluginlibException& ex)
		{
			ROS_ERROR_STREAM("initializeBenchmarkExecutor --> Exception while loading planner '" << plugin_classes[i] << "': " << ex.what());
		}
	}

	// error check
	if (planner_interfaces_.empty())
		ROS_ERROR("initializeBenchmarkExecutor --> No planning plugins have been loaded. Nothing to do for the benchmarking service.");
	else
	{
		std::stringstream ss;
		for (std::map<std::string, planning_interface::PlannerManagerPtr>::const_iterator it = planner_interfaces_.begin(); it != planner_interfaces_.end(); ++it)
			ss << it->first << " ";
		ROS_WARN("initializeBenchmarkExecutor --> Available planner instances: %s", ss.str().c_str());
	  }
}

bool BenchmarkExecutor::pathPlanCB(ipa_benchmark::pathPlanning::Request& request, ipa_benchmark::pathPlanning::Response& response)
{

	robot_model::RobotModelConstPtr robot_model = planning_scene_->	getRobotModel();

	robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();

	const robot_state::JointModelGroup* joint_model_group =  robot_model->getJointModelGroup("arm");

	std::vector<double> start_state_joint_value, goal_state_joint_value;

	bool found_start_state = robot_state.setFromIK(joint_model_group, request.start_pose.pose, 10, 1.0);	//compute start state joint value

	robot_state::RobotState start_state(robot_state);

	if (found_start_state)
	{
		ROS_WARN("IK found");

		start_state_joint_value.resize(6, 0);
		start_state_joint_value[0] = -1.02; start_state_joint_value[1] = -0.40; start_state_joint_value[2] = -2.402;
		start_state_joint_value[3] = -3.48; start_state_joint_value[4] = -1.81; start_state_joint_value[5] = 0.00;

		//start_state.copyJointGroupPositions("arm", start_state_joint_value);
		start_state.setJointGroupPositions(joint_model_group, start_state_joint_value);

	}
	for (std::vector<double>::const_iterator it = start_state_joint_value.begin(); it!= start_state_joint_value.end(); ++it)
		ROS_WARN_STREAM(" "<<*it);

	bool found_goal_state = start_state.setFromIK(joint_model_group, request.goal_pose.pose, 10, 1.0);	//compute start state joint value

		if (found_goal_state)
		{
			ROS_WARN("goal IK found");

			moveit_msgs::MotionPlanRequest plan_req;
			moveit_msgs::MotionPlanDetailedResponse res;
			plan_req.allowed_planning_time = 10;
			plan_req.group_name = "arm";
			plan_req.num_planning_attempts = 1;


			//start_state.copyJointGroupPositions("arm", goal_state_joint_value);
			//start_state.setJointGroupPositions(joint_model_group, goal_state_joint_value);
			ROS_WARN("goal IK found");
		}



	std::cout<<std::endl;
	for (std::vector<double>::const_iterator it = goal_state_joint_value.begin(); it!= goal_state_joint_value.end(); ++it)
		ROS_WARN_STREAM(" "<<*it);

}

