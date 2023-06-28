#ifndef AD_KINEMATICS_TREE_H
#define AD_KINEMATICS_TREE_H

#include <ad_kinematics/urdf_loader.h>
#include <unordered_map>

namespace ad_kinematics {
class Tree {
public:
  explicit Tree(const urdf::ModelInterfaceSharedPtr &urdf);
  unsigned int getNumActuatedJoints();

  /**
   * Computes transform from base link to specified link
   * @tparam T
   * @param link_name
   * @param joint_angles
   * @return
   */
  template<typename T>
  Transform<T> computeTransform(const std::string& link_name, const std::vector<T>& joint_angles) {
    if (joint_angles.size() != getNumActuatedJoints()) {
      ROS_ERROR("[Tree::computeTransform] joint_angles size (%lu) doesn't match number of actuated joints (%u).", joint_angles.size(), getNumActuatedJoints());
      return Transform<T>();
    }

    // Handle special case: base link
    if (link_name == getBaseLinkName()) {
      return Transform<T>(); // Identity
    }

    // Find end link
    auto it = links_.find(link_name);
    if (it == links_.end()) {
      ROS_ERROR_STREAM("[Tree::computeTransform] Unknown link '" << link_name << "'.");
      return Transform<T>();
    }

    std::shared_ptr<Link> current_link = it->second;
    Transform<T> current_pose = current_link->pose<T>(joint_angles);
    current_link = current_link->getParentLink();

    while (current_link) {
      current_pose = current_link->pose<T>(joint_angles) * current_pose;
      current_link = current_link->getParentLink();
    }
    return current_pose;
  }
  std::vector<int> getJointQIndices(const std::vector<std::string> &joint_names);
  std::string getBaseLinkName() const;
  std::vector<std::string> getJointNames() const;
  std::vector<std::string> getActuatedJointNames() const;
private:
  void buildTree(const urdf::LinkConstSharedPtr& link_ptr, const std::shared_ptr<Link>& parent);
  std::shared_ptr<Link> addToTree(const urdf::LinkConstSharedPtr& urdf_link, const std::shared_ptr<Link>& parent);

  std::unordered_map<std::string, std::shared_ptr<Link>> links_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> actuated_joint_names;
  std::string base_link_name_;
};
}



#endif  // AD_KINEMATICS_TREE_H
