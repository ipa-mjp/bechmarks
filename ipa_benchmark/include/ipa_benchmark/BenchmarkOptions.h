
#ifndef IPA_BENCHMARK_BENCHMARKOPTIONS_H
#define IPA_BENCHMARK_BENCHMARKOPTIONS_H

#include <string>
#include <map>
#include <vector>
#include <ros/ros.h>
#include <moveit_msgs/WorkspaceParameters.h>
#include <moveit_msgs/DisplayTrajectory.h>

class BenchmarkOptions
{
public:

	BenchmarkOptions();
	BenchmarkOptions(const std::string& ros_namespace);
	BenchmarkOptions(ros::NodeHandle& nh);

	virtual ~BenchmarkOptions();

	void setNamespace(const std::string& ros_namespace);

	int getNumRuns() const;
	double getTimeout() const;
	const std::string& getBenchmarkName() const;
	const std::string& getGroupName() const;
	const std::string& getOutputDirectory() const;
	const std::map<std::string, std::vector<std::string>>& getPlannerConfigurations() const;
	void getPlannerPluginList(std::vector<std::string>& plugin_list) const;

protected:
	void readBenchmarkOptions(const std::string& ros_namespace);
	void readBenchmarkOptions(ros::NodeHandle& nh);
	void readBenchmarkParameters(ros::NodeHandle& nh);
	void readPlannerConfigs(ros::NodeHandle& nh);

	  /// benchmark parameters
	  int runs_;
	  double timeout_;
	  std::string benchmark_name_;
	  std::string group_name_;
	  std::string output_directory_;


	  /// planner configurations
	  std::map<std::string, std::vector<std::string>> planners_;

};

#endif
