
#include <ros/ros.h>
#include <string>

#include <ipa_benchmark/BenchmarkOptions.h>
#include <ipa_benchmark/BenchmarkExecutor.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "benchmarks_node");
	//ros::AsyncSpinner spinner(1);
	//spinner.start();

	ros::NodeHandle nh(ros::this_node::getName());
	BenchmarkOptions opts(nh);

	BenchmarkExecutor server("robot_description", nh);
	std::vector<std::string> plugins;
	opts.getPlannerPluginList(plugins);
	server.initializeBenchmarkExecutor(plugins);

	ros::spin();

}
