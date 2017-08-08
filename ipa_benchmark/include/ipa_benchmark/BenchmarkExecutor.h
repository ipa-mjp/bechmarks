
#ifndef IPA_BENCHMARK_BENCHMARKEXECUTOR_H
#define IPA_BENCHMARK_BENCHMARKEXECUTOR_H

#include <ipa_benchmark/BenchmarkOptions.h>

#include <ipa_benchmark/pathPlanning.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <pluginlib/class_loader.h>

#include <boost/shared_ptr.hpp>
#include <map>
#include <vector>
#include <string>
#include <boost/function.hpp>
#include <memory>


class BenchmarkExecutor
{
public:

	/// Structure to hold information for a single run of a planner
	typedef std::map<std::string, std::string> PlannerRunData;
	/// Structure to hold information for a single planner's benchmark data.
	typedef std::vector<PlannerRunData> PlannerBenchmarkData;

	BenchmarkExecutor(const std::string& robot_description_param = "robot_description");
	BenchmarkExecutor(const std::string& robot_description_param, ros::NodeHandle& nh);

	virtual ~BenchmarkExecutor();

	void initializeBenchmarkExecutor(const std::vector<std::string>& plugin_classes);


protected:

	ros::ServiceServer pathPlan_;

	bool pathPlanCB(ipa_benchmark::pathPlanning::Request& request, ipa_benchmark::pathPlanning::Response& response);

	planning_scene::PlanningScenePtr planning_scene_;
	planning_scene_monitor::PlanningSceneMonitor* psm_;

	BenchmarkOptions options_;

	boost::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader_;
	std::map<std::string, planning_interface::PlannerManagerPtr> planner_interfaces_;

};


#endif
