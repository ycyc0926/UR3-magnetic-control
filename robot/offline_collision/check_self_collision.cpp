// In-memory diagnostic only. No ROS node, executor, publisher, hardware driver,
// sockets, motion controller, or execution API is created by this program.
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cmath>

std::string read(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot read " + path);
  std::ostringstream data; data << input.rdbuf(); return data.str();
}

moveit_msgs::msg::AttachedCollisionObject box(const YAML::Node& geometry, bool test=false) {
  moveit_msgs::msg::AttachedCollisionObject object;
  object.link_name = "tool0";
  object.touch_links = {"tool0", "flange", "wrist_3_link"};
  object.object.header.frame_id = "tool0";
  object.object.id = test ? "diagnostic_positive_control" : "provisional_whole_tool_envelope";
  object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive shape; shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  geometry_msgs::msg::Pose pose; pose.orientation.w=1.;
  auto low=geometry["min_xyz_m"].as<std::vector<double>>();
  auto high=geometry["max_xyz_m"].as<std::vector<double>>();
  if(low.size()!=3 || high.size()!=3) throw std::runtime_error("Invalid tool box");
  std::vector<double> center(3);
  for(size_t i=0;i<3;++i) {
    if(!std::isfinite(low[i]) || !std::isfinite(high[i]) || high[i]<=low[i])
      throw std::runtime_error("Invalid tool bounds");
    shape.dimensions.push_back(test ? 10. : high[i]-low[i]);
    center[i]=test ? 0. : (low[i]+high[i])/2.;
  }
  pose.position.x=center[0];pose.position.y=center[1];pose.position.z=center[2];
  object.object.primitives.push_back(shape);object.object.primitive_poses.push_back(pose);
  return object;
}

std::string pairs(const collision_detection::CollisionResult& result) {
  std::ostringstream out;
  for(const auto& entry:result.contacts) {
    if(out.tellp()>0)out << ';';
    out << entry.first.first << '|' << entry.first.second;
  }
  return out.str();
}

int main(int argc, char** argv) {
  try {
    if(argc!=4)throw std::runtime_error("Usage: check_self_collision URDF SRDF diagnostic.json");
    auto urdf=urdf::parseURDF(read(argv[1]));
    auto srdf=std::make_shared<srdf::Model>();
    if(!urdf || !srdf->initString(*urdf,read(argv[2])))throw std::runtime_error("Invalid robot XML");
    auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    if(model->getVariableCount()!=6 || model->getLinkModelsWithCollisionGeometry().size()<7)
      throw std::runtime_error("Incomplete UR3 model");
    for(const auto* link:model->getLinkModelsWithCollisionGeometry())
      for(const auto& shape:link->getShapes())
        if(!shape)throw std::runtime_error("Missing collision shape");
    planning_scene::PlanningScene bare(model), tool(model);
    bare.allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
    tool.allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
    auto data=YAML::LoadFile(argv[3]);
    auto names=data["joint_names"].as<std::vector<std::string>>();
    if(names.size()!=6 || data["samples"].size()==0)throw std::runtime_error("Missing states");
    const auto& variables=model->getVariableNames();
    for(const auto& name:names)
      if(std::find(variables.begin(),variables.end(),name)==variables.end())throw std::runtime_error("Unknown joint");
    auto geometry=data["tool_geometry"]["provisional_whole_tool_envelope"];
    collision_detection::CollisionRequest request;
    request.contacts=true;request.max_contacts=100;request.max_contacts_per_pair=1;
    // Positive control: an intentionally enormous attached box must collide
    // with non-touch links. It is removed before any requested sample is tested.
    if(!tool.processAttachedCollisionObjectMsg(box(geometry,true)))throw std::runtime_error("Test attachment failed");
    collision_detection::CollisionResult positive;
    tool.checkSelfCollision(request,positive);
    if(!positive.collision)throw std::runtime_error("FCL positive control failed");
    moveit_msgs::msg::AttachedCollisionObject remove;
    remove.link_name="tool0";remove.object.id="diagnostic_positive_control";
    remove.object.operation=moveit_msgs::msg::CollisionObject::REMOVE;
    if(!tool.processAttachedCollisionObjectMsg(remove))throw std::runtime_error("Test removal failed");
    if(!tool.processAttachedCollisionObjectMsg(box(geometry)))throw std::runtime_error("Tool attachment failed");
    std::cerr << "FCL positive control passed; standard SRDF exclusions retained; only flange/tool0/wrist_3 are tool touch links.\n";
    std::cout << "index,bare_collision,tool_collision,joint_bounds_ok,bare_pairs,tool_pairs,fx,fy,fz,qx,qy,qz,qw\n";
    std::cout << std::setprecision(17);
    size_t index=0;
    for(const auto& sample:data["samples"]) {
      auto q=sample["q_rad"].as<std::vector<double>>();
      if(q.size()!=names.size())throw std::runtime_error("Bad joint count");
      for(auto value:q)if(!std::isfinite(value))throw std::runtime_error("Nonfinite joint");
      auto& bs=bare.getCurrentStateNonConst();auto& ts=tool.getCurrentStateNonConst();
      bs.setVariablePositions(names,q);ts.setVariablePositions(names,q);bs.update();ts.update();
      collision_detection::CollisionResult br,tr;
      bare.checkSelfCollision(request,br);tool.checkSelfCollision(request,tr);
      auto flange=bs.getGlobalLinkTransform("base").inverse()*bs.getGlobalLinkTransform("tool0");
      Eigen::Quaterniond quat(flange.rotation());
      std::cout << index++ << ',' << br.collision << ',' << tr.collision << ',' << bs.satisfiesBounds()
                << ',' << pairs(br) << ',' << pairs(tr) << ',' << flange.translation().x()
                << ',' << flange.translation().y() << ',' << flange.translation().z()
                << ',' << quat.x() << ',' << quat.y() << ',' << quat.z() << ',' << quat.w() << '\n';
    }
    return 0;
  } catch(const std::exception& error) {std::cerr << error.what() << '\n';return 1;}
}
