#include <ad_kinematics/chain.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

namespace ad_kinematics {

Chain::Chain(const urdf::ModelInterfaceSharedPtr& urdf, const moveit::core::JointModelGroup *joint_group)
{
  init(urdf, joint_group);
}

Chain::Chain(std::string group_name)
{
  robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr robot_model_ptr = robot_model_loader.getModel();
  robot_model::JointModelGroup* joint_model_group = robot_model_ptr->getJointModelGroup(group_name);
  init(robot_model_ptr->getURDF(), joint_model_group);
}

unsigned int Chain::getNumActuatedJoints() {
  return num_actuated_joints_;
}

std::vector<int> Chain::getJointQIndices(const std::vector<std::string> &joint_names)
{
  std::vector<int> indices;
  for (const std::string& joint_name: joint_names) {
    bool found = false;
    for (const Link& link: chain_) {
      if (link.getParentJoint()->getName() == joint_name) {
        indices.push_back(link.getParentJoint()->getQIndex());
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

void Chain::init(const urdf::ModelInterfaceSharedPtr &urdf, const moveit::core::JointModelGroup *joint_group)
{
  num_actuated_joints_ = 0;
  buildChain(urdf->getRoot(), joint_group);
}

void Chain::buildChain(const boost::shared_ptr<const urdf::Link>& root, const robot_model::JointModelGroup* joint_group) {
  ROS_INFO_STREAM("Parsing URDF");
  base_link_name_ = root->name;
  tip_link_name_ = base_link_name_;
  std::vector<const robot_model::LinkModel*> link_models = joint_group->getLinkModels();

  boost::shared_ptr<const urdf::Link> current_link = root;
  for (std::vector<const robot_model::LinkModel*>::iterator it = link_models.begin(); it != link_models.end(); ++it) {
    std::string link_name = (*it)->getName();

    bool found = false;
    ROS_INFO_STREAM(current_link->name << ":");
    // TODO replace with 'std::find'
    for (std::vector<boost::shared_ptr<urdf::Link>>::const_iterator it_childs = current_link->child_links.begin();
         it_childs != current_link->child_links.end() && !found;
         ++it_childs) {

      ROS_INFO_STREAM(" --> " << (*it_childs)->name);

      if ((*it_childs)->name == link_name) {
        addToChain(*it_childs);
        current_link = *it_childs;
        found = true;
      }
    }

    if (!found) {
      ROS_WARN_STREAM("URDF Loader could not find link '" << link_name << "'.");
    }

  }
  ROS_INFO_STREAM("URDF parsing finished.");
}

bool Chain::addToChain(const boost::shared_ptr<const urdf::Link>& urdf_link) {
  // q_index of joint matches current number of actuated joints
  // q_index parameter is only used, if joint is actually actuated
  boost::shared_ptr<Joint> joint = urdf_loader::toJoint(urdf_link->parent_joint, getNumActuatedJoints());
  if (joint->isActuated()) {
    num_actuated_joints_++;
  }
//  ROS_INFO_STREAM("Origin: " << joint->getOrigin() << ", Axis: " << joint->getAxis() << ", Pose(0): " << joint->pose(0.0).toString());
  Link link(urdf_link->name, urdf_loader::toTransform(urdf_link->parent_joint->parent_to_joint_origin_transform), joint);
  ROS_INFO_STREAM("Adding link " << urdf_link->name);
  chain_.push_back(link);
  tip_link_name_ = link.getName();
  return true;
}

std::string Chain::getTipLinkName() const
{
  return tip_link_name_;
}

std::vector<std::string> Chain::getJointNames() const
{
  std::vector<std::string> joint_names;
  for (const Link& link: chain_) {
    joint_names.push_back(link.getParentJoint()->getName());
  }
  return joint_names;
}

std::vector<std::string> Chain::getActuatedJointNames() const
{
  std::vector<std::string> joint_names;
  for (const Link& link: chain_) {
    if (link.getParentJoint()->isActuated()) {
      joint_names.push_back(link.getParentJoint()->getName());
    }
  }
  return joint_names;
}

std::string Chain::getBaseLinkName() const
{
  return base_link_name_;
}

std::vector<Link> Chain::getChain() const
{
  return chain_;
}

}
