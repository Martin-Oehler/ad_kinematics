#include <ad_kinematics/tree.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

namespace ad_kinematics {

Tree::Tree(const urdf::ModelInterfaceSharedPtr& urdf)
{
  base_link_name_ = urdf->getRoot()->name;
  buildTree(urdf->getRoot(), nullptr);
}

unsigned int Tree::getNumActuatedJoints()
{
  return actuated_joint_names.size();
}
std::string Tree::getBaseLinkName() const
{
  return base_link_name_;
}
std::vector<std::string> Tree::getJointNames() const
{
  return joint_names_;
}
std::vector<std::string> Tree::getActuatedJointNames() const
{
  return actuated_joint_names;
}
std::vector<int> Tree::getJointQIndices(const std::vector<std::string>& joint_names)
{
  std::vector<int> indices;
  for (const std::string& joint_name: joint_names) {
    bool found = false;
    for (int j = 0; j < actuated_joint_names.size(); ++j) {
      if (actuated_joint_names[j] == joint_name) {
        indices.push_back(j);
        found = true;
        break;
      }
    }
    if (!found) {
      ROS_ERROR_STREAM("Could not find joint with name '" << joint_name << "'.");
      indices.push_back(-1);
    }
  }
  return indices;
}
void Tree::buildTree(const urdf::LinkConstSharedPtr& link_ptr, const std::shared_ptr<Link>& parent)
{
  for (const auto& child_link_ptr: link_ptr->child_links) {
    auto tree_link = addToTree(child_link_ptr, parent);
    buildTree(child_link_ptr, tree_link);
  }

}
std::shared_ptr<Link> Tree::addToTree(const urdf::LinkConstSharedPtr& urdf_link, const std::shared_ptr<Link>& parent)
{
  // q_index of joint matches current number of actuated joints
  // q_index parameter is only used, if joint is actually actuated
  std::shared_ptr<Joint> joint = urdf_loader::toJoint(urdf_link->parent_joint, getNumActuatedJoints());
  joint_names_.push_back(joint->getName());
  if (joint->isActuated()) {
    actuated_joint_names.push_back(joint->getName());
  }
  //  ROS_INFO_STREAM("Origin: " << joint->getOrigin() << ", Axis: " << joint->getAxis() << ", Pose(0): " << joint->pose(0.0).toString());
  auto link = std::make_shared<Link>(urdf_link->name, urdf_loader::toTransform(urdf_link->parent_joint->parent_to_joint_origin_transform), joint, parent);
  ROS_INFO_STREAM("Adding link " << urdf_link->name);
  links_.emplace(link->getName(), link);

  return link;
}
std::shared_ptr<Link> Tree::getLink(const std::string& link_name) const
{
  auto it = links_.find(link_name);
  if (it == links_.end()) {
    ROS_ERROR_STREAM("[Tree::computeTransform] Unknown link '" << link_name << "'.");
    return {};
  }
  return it->second;
}

}
