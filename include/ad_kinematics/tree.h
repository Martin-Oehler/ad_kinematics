#ifndef AD_KINEMATICS_TREE_H
#define AD_KINEMATICS_TREE_H

#include <ad_kinematics/urdf_loader.h>
#include <unordered_map>

namespace ad_kinematics {
class Tree {
public:
  explicit Tree(const urdf::ModelInterfaceSharedPtr &urdf);
  unsigned int getNumActiveJoints();

  /**
   * Computes transform from base link to specified link
   * @tparam T
   * @param link_name
   * @param joint_angles
   * @return
   */
  template<typename T>
  Transform<T> computeTransform(const std::string& link_name, const std::vector<T>& joint_angles) {
    if (joint_angles.size() != getNumActiveJoints()) {
      ROS_ERROR("[Tree::computeTransform] joint_angles size (%lu) doesn't match number of actuated joints (%u).", joint_angles.size(), getNumActiveJoints());
      return Transform<T>();
    }

    // Handle special case: base link
    // The base link does not exist explicitly in the tree, therefore, we have to catch it separately
    if (link_name == getBaseLinkName()) {
      return Transform<T>(); // Identity
    }

    // Find end link
    auto it = links_.find(link_name);
    if (it == links_.end()) {
      ROS_ERROR_STREAM("[Tree::computeTransform] Unknown link '" << link_name << "'.");
      return Transform<T>();
    }

    return computeTransform(it->second, joint_angles);
  }

  template<typename T>
  Transform<T> computeTransform(const std::shared_ptr<Link>& link, const std::vector<T>& joint_angles) {
    Transform<T> transform = link->pose<T>(joint_angles);
    std::shared_ptr<Link> parent = link->getParentLink();
    if (parent) {
      return computeTransform<T>(parent, joint_angles) * transform;
    } else {
      return transform;
    }
  }

  std::vector<int> getJointQIndices(const std::vector<std::string> &joint_names);
  std::string getBaseLinkName() const;
  std::shared_ptr<Link> getLink(const std::string& link_name) const;
  std::shared_ptr<Joint> getJoint(const std::string& joint_name) const;
  std::vector<std::string> getJointNames() const;
  std::vector<std::string> getActiveJointNames() const;
private:
  void buildTree(const urdf::LinkConstSharedPtr& link_ptr, const std::shared_ptr<Link>& parent);
  std::shared_ptr<Link> addToTree(const urdf::LinkConstSharedPtr& urdf_link, const std::shared_ptr<Link>& parent);
  void updateMimicJoints(const urdf::ModelInterfaceSharedPtr& urdf);

  std::unordered_map<std::string, std::shared_ptr<Link>> links_;
  std::unordered_map<std::string, std::shared_ptr<Joint>> joints_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> active_joint_names_;
  std::vector<std::string> mimic_joint_names_;
  std::string base_link_name_;
};
}



#endif  // AD_KINEMATICS_TREE_H
