#include <ad_kinematics/tree.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

namespace ad_kinematics {

Tree::Tree(const urdf::ModelInterfaceSharedPtr& urdf)
: disable_transform_cache_(false)
{
  base_link_name_ = urdf->getRoot()->name;
  buildTree(urdf->getRoot(), nullptr);
  updateMimicJoints(urdf);
}

unsigned int Tree::getNumActiveJoints()
{
  return active_joint_names_.size();
}
std::string Tree::getBaseLinkName() const
{
  return base_link_name_;
}
std::vector<std::string> Tree::getJointNames() const
{
  return joint_names_;
}
std::vector<std::string> Tree::getActiveJointNames() const
{
  return active_joint_names_;
}
std::vector<int> Tree::getJointQIndices(const std::vector<std::string>& joint_names)
{
  std::vector<int> indices;
  for (const std::string& joint_name: joint_names) {
    bool found = false;
    for (int j = 0; j < active_joint_names_.size(); ++j) {
      if (active_joint_names_[j] == joint_name) {
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
  // q_index of joint matches current number of active joints
  // q_index parameter is only used, if joint is actually actuated
  std::shared_ptr<Joint> joint = urdf_loader::toJoint(urdf_link->parent_joint, getNumActiveJoints());
  joints_.emplace(joint->getName(), joint);
  joint_names_.push_back(joint->getName());
  if (joint->isActuated()) {
    if (joint->isActive()) {
      active_joint_names_.push_back(joint->getName());
    } else {
      mimic_joint_names_.push_back(joint->getName());
    }
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
void Tree::updateMimicJoints(const urdf::ModelInterfaceSharedPtr& urdf)
{
  for (auto& mimic_name: mimic_joint_names_) {
    auto joint = getJoint(mimic_name);
    if (!joint) continue;
    if (!joint->isActuated() || joint->isActive()) continue;
    auto joint_mimic = getJoint(joint->getMimic()->mimic_joint_name);
    if (!joint_mimic) continue;

    joint->setQIndex(joint_mimic->getQIndex());
    ROS_INFO_STREAM("Setting joint " << joint->getName() << " as mimic of " << joint->getMimic()->mimic_joint_name << " (offset: " << joint->getMimic()->offset << ", multiplier: " << joint->getMimic()->multiplier << ", q_index: " << joint->getQIndex() << ")");
  }
}
std::shared_ptr<Joint> Tree::getJoint(const std::string& joint_name) const
{
  auto it = joints_.find(joint_name);
  if (it == joints_.end()) {
    ROS_ERROR_STREAM("[Tree::computeTransform] Unknown joint '" << joint_name << "'.");
    return {};
  }
  return it->second;
}
void Tree::disableTransformCache() {
  disable_transform_cache_ = true;
}

}
