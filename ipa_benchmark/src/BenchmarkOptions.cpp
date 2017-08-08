
#include "../include/ipa_benchmark/BenchmarkOptions.h"


BenchmarkOptions::BenchmarkOptions()
{
}

BenchmarkOptions::BenchmarkOptions(const std::string& ros_namespace)
{
  readBenchmarkOptions(ros_namespace);
}

BenchmarkOptions::BenchmarkOptions(ros::NodeHandle& nh)
{
  readBenchmarkOptions(nh);
}

BenchmarkOptions::~BenchmarkOptions()
{
}

void BenchmarkOptions::setNamespace(const std::string& ros_namespace)
{
  readBenchmarkOptions(ros_namespace);
}

void BenchmarkOptions::readBenchmarkOptions(const std::string& ros_namespace)
{
  ros::NodeHandle nh(ros_namespace);

  XmlRpc::XmlRpcValue benchmark_config;
  if (nh.getParam("benchmark_config", benchmark_config))
  {
    readBenchmarkParameters(nh);
    readPlannerConfigs(nh);
  }
  else
  {
    ROS_WARN("No benchmark_config found on param server");
  }
}

void BenchmarkOptions::readBenchmarkOptions(ros::NodeHandle& nh)
{

  XmlRpc::XmlRpcValue benchmark_config;
  if (nh.getParam("benchmark_config", benchmark_config))
  {
    readBenchmarkParameters(nh);
    readPlannerConfigs(nh);
  }
  else
  {
    ROS_WARN("No benchmark_config found on param server");
  }
}

int BenchmarkOptions::getNumRuns() const
{
  return runs_;
}

double BenchmarkOptions::getTimeout() const
{
  return timeout_;
}

const std::string& BenchmarkOptions::getBenchmarkName() const
{
  return benchmark_name_;
}

const std::string& BenchmarkOptions::getGroupName() const
{
  return group_name_;
}

const std::string& BenchmarkOptions::getOutputDirectory() const
{
  return output_directory_;
}


const std::map<std::string, std::vector<std::string>>& BenchmarkOptions::getPlannerConfigurations() const
{
  return planners_;
}

void BenchmarkOptions::getPlannerPluginList(std::vector<std::string>& plugin_list) const
{
  plugin_list.clear();
  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners_.begin(); it != planners_.end();
       ++it)
    plugin_list.push_back(it->first);
}


void BenchmarkOptions::readBenchmarkParameters(ros::NodeHandle& nh)
{
  nh.param(std::string("benchmark_config/parameters/name"), benchmark_name_, std::string(""));
  nh.param(std::string("benchmark_config/parameters/runs"), runs_, 10);
  nh.param(std::string("benchmark_config/parameters/timeout"), timeout_, 10.0);
  nh.param(std::string("benchmark_config/parameters/output_directory"), output_directory_, std::string(""));

  if (!nh.getParam(std::string("benchmark_config/parameters/group"), group_name_))
    ROS_WARN("Benchmark group NOT specified");

  ROS_INFO("Benchmark name: '%s'", benchmark_name_.c_str());
  ROS_INFO("Benchmark #runs: %d", runs_);
  ROS_INFO("Benchmark timeout: %f secs", timeout_);
  ROS_INFO("Benchmark group: %s", group_name_.c_str());
  ROS_INFO("Benchmark output directory: %s", output_directory_.c_str());
}

void BenchmarkOptions::readPlannerConfigs(ros::NodeHandle& nh)
{
  planners_.clear();

  XmlRpc::XmlRpcValue planner_configs;
  if (nh.getParam("benchmark_config/planners", planner_configs))
  {
    if (planner_configs.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("readPlannerConfigs --> Expected a list of planner configurations to benchmark");
      return;
    }

    for (std::size_t i = 0; i < planner_configs.size(); ++i)
    {
      if (planner_configs[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_WARN("readPlannerConfigs --> Improper YAML type for planner configurations");
        continue;
      }
      if (!planner_configs[i].hasMember("plugin") || !planner_configs[i].hasMember("planners"))
      {
        ROS_WARN("readPlannerConfigs --> Malformed YAML for planner configurations");
        continue;
      }

      if (planner_configs[i]["planners"].getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_WARN("readPlannerConfigs --> Expected a list of planners to benchmark");
        continue;
      }

      std::string plugin = planner_configs[i]["plugin"];
      ROS_INFO("readPlannerConfigs --> Reading in planner names for plugin '%s'", plugin.c_str());

      std::vector<std::string> planners;
      for (std::size_t j = 0; j < planner_configs[i]["planners"].size(); ++j)
        planners.push_back(planner_configs[i]["planners"][j]);

      for (std::size_t j = 0; j < planners.size(); ++j)
        ROS_INFO("  [%lu]: %s", j, planners[j].c_str());

      planners_[plugin] = planners;
    }
  }
}
