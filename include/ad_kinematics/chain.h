#ifndef AD_KINEMATICS__CHAIN_H
#define AD_KINEMATICS__CHAIN_H

#include <ad_kinematics/urdf_loader.h>

namespace ad_kinematics {
class Chain {
public:
  Chain(const urdf::ModelInterfaceSharedPtr &urdf, const robot_model::JointModelGroup* joint_group);
  Chain(std::string group_name);

  unsigned int getNumActuatedJoints();

  template<typename T>
  Transform<T> computeTipTransform(const std::vector<T>& joint_angles) {
    return computeTransform<T>(getTipLinkName(), joint_angles);
  }

  template<typename T>
  Transform<T> computeTransform(const std::string& link_name, const std::vector<T>& joint_angles) {
    if (joint_angles.size() != getNumActuatedJoints()) {
      ROS_ERROR("[Chain::computeTransform] joint_angles size (%lu) doesn't match number of actuacted joints (%u).", joint_angles.size(), getNumActuatedJoints());
      return Transform<T>();
    }

    Transform<T> current_pose; // identity
    if (link_name == getBaseLinkName()) {
      return current_pose;
    }

    bool link_found = false;
    for (unsigned int i = 0; i < chain_.size(); i++) {
      current_pose = current_pose * chain_[i].pose<T>(joint_angles);

      if (chain_[i].getName() == link_name) {
        link_found = true;
        break;
      }
    }

    if (!link_found) {
      ROS_ERROR_STREAM("[Chain::computeTransform] Could not find link '" << link_name << "' in chain.");
      return Transform<T>();
    }

    return current_pose;
  }

  std::vector<int> getJointQIndices(const std::vector<std::string> &joint_names);
  std::vector<Link> getChain() const;
  std::string getBaseLinkName() const;
  std::string getTipLinkName() const;
  std::vector<std::string> getJointNames() const;
  std::vector<std::string> getActuatedJointNames() const;

private:
  void init(const urdf::ModelInterfaceSharedPtr &urdf, const robot_model::JointModelGroup* joint_group);
  void buildChain(const boost::shared_ptr<const urdf::Link>& root, const robot_model::JointModelGroup* joint_group);
  bool addToChain(const boost::shared_ptr<const urdf::Link>& urdf_link);
  std::vector<Link> chain_;
  unsigned int num_actuated_joints_;
  std::string base_link_name_;
  std::string tip_link_name_;
};
}

#endif
