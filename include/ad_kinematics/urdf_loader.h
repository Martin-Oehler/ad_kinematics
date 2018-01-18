#ifndef AD_KINEMATICS__URDF_LOADER_H
#define AD_KINEMATICS__URDF_LOADER_H

#include <ros/ros.h>

#include <ad_kinematics/transforms.h>

#include <moveit/robot_model/robot_model.h>

namespace ad_kinematics {
namespace urdf_loader {
  Transformd toTransform(const urdf::Pose& p);
  Eigen::Quaterniond toRotation(const urdf::Rotation& r);
  Eigen::Vector3d toTranslation(const urdf::Vector3& v);

  std::shared_ptr<Joint> toJoint(const boost::shared_ptr<const urdf::Joint>& urdf_joint, int q_index=-1);
}
}

#endif
