// Offline use of the installed controller's interpolation library. No ROS node,
// controller instance, hardware, network or trajectory publisher is created.
#include <joint_trajectory_controller/trajectory.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <iomanip>
#include <iostream>

trajectory_msgs::msg::JointTrajectoryPoint point(const YAML::Node& node) {
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions=node["q_rad"].as<std::vector<double>>();
  p.velocities=node["velocity_rad_s"].as<std::vector<double>>();
  p.accelerations=node["acceleration_rad_s2"].as<std::vector<double>>();
  if(p.positions.size()!=6 || p.velocities.size()!=6 || p.accelerations.size()!=6)
    throw std::runtime_error("Exactly six complete position/velocity/acceleration fields required");
  for(auto values:{p.positions,p.velocities,p.accelerations})
    for(double value:values)if(!std::isfinite(value))throw std::runtime_error("Nonfinite trajectory");
  return p;
}

int main(int argc,char**argv) {
  try {
    if(argc!=2)throw std::runtime_error("Usage: sample_timed_trajectory draft.json");
    const auto data=YAML::LoadFile(argv[1]);
    const auto points=data["points"];
    if(points.size()<2)throw std::runtime_error("At least two points required");
    const double period=data["sample_period_s"].as<double>();
    if(!std::isfinite(period) || period<=0 || period>.01)throw std::runtime_error("Invalid sample period");
    joint_trajectory_controller::Trajectory interpolator;
    std::cout << "time_s";
    for(const auto* prefix:{"q","v","a"})for(size_t i=0;i<6;++i)std::cout << ',' << prefix << i;
    std::cout << '\n' << std::setprecision(17);
    for(size_t index=1;index<points.size();++index) {
      const double a=points[index-1]["time_s"].as<double>(),b=points[index]["time_s"].as<double>();
      if(!std::isfinite(a) || !std::isfinite(b) || a<0 || b<=a)throw std::runtime_error("Invalid time ordering");
      const auto left=point(points[index-1]),right=point(points[index]);
      const auto count=static_cast<size_t>(std::ceil((b-a)/period));
      const rclcpp::Time ta(static_cast<int64_t>(std::llround(a*1e9)),RCL_ROS_TIME);
      const rclcpp::Time tb(static_cast<int64_t>(std::llround(b*1e9)),RCL_ROS_TIME);
      for(size_t i=(index==1?0:1);i<=count;++i) {
        const double t=a+(b-a)*static_cast<double>(i)/count;
        const rclcpp::Time ts(static_cast<int64_t>(std::llround(t*1e9)),RCL_ROS_TIME);
        trajectory_msgs::msg::JointTrajectoryPoint out;
        interpolator.interpolate_between_points(ta,left,tb,right,ts,out);
        if(out.positions.size()!=6 || out.velocities.size()!=6 || out.accelerations.size()!=6)
          throw std::runtime_error("Incomplete interpolation output");
        std::cout << ts.seconds();
        for(const auto& values:{out.positions,out.velocities,out.accelerations})
          for(double value:values)std::cout << ',' << value;
        std::cout << '\n';
      }
    }
    return 0;
  }catch(const std::exception& error){std::cerr << error.what() << '\n';return 1;}
}
