#include <ad_kinematics/transforms.h>
#include <eigen_conversions/eigen_msg.h>

namespace ad_kinematics {

// Transform
geometry_msgs::Pose transformToMsg(const Transform<double>& transform) {
  geometry_msgs::Pose msg;
//  msg.position.x = transform.translation(0);
//  msg.position.y = transform.translation(1);
//  msg.position.z = transform.translation(2);

//  msg.orientation.x = transform.rotation.x();
//  msg.orientation.y = transform.rotation.y();
//  msg.orientation.z = transform.rotation.z();
//  msg.orientation.w = transform.rotation.w();

  tf::pointEigenToMsg(transform.translation, msg.position);
  tf::quaternionEigenToMsg(transform.rotation, msg.orientation);

  return msg;
}

Transformd msgToTransform(const geometry_msgs::Pose& msg) {
  Transformd t;
  tf::pointMsgToEigen(msg.position, t.translation);
  tf::quaternionMsgToEigen(msg.orientation, t.rotation);
  return t;
//  return Transformd(Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z),
//                   Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z));
}

// FixedJoint
FixedJoint::FixedJoint(std::string name, const Eigen::Vector3d& origin)
  : Joint(name, origin) {}

bool FixedJoint::isActuated() const {
  return false;
}

// RevoluteJoint
RevoluteJoint::RevoluteJoint(std::string name, const Eigen::Vector3d& origin, const Eigen::Vector3d& axis, int q_index, double upper_limit, double lower_limit)
  : Joint(name, origin, axis, q_index, upper_limit, lower_limit) {}

bool RevoluteJoint::isActuated() const {
  return true;
}

// Continuous Joint
ContinuousJoint::ContinuousJoint(std::string name, const Eigen::Vector3d &origin, const Eigen::Vector3d &axis, int q_index)
  : RevoluteJoint(name, origin, axis, q_index, M_PI, -M_PI) {}

// Link
Link::Link(std::string name, const Transformd& tip_transform, const std::shared_ptr<Joint>& joint)
  : name_(name), tip_transform_(joint->pose<double>(0).inverse() * tip_transform), parent_joint_(joint) {}

std::shared_ptr<Joint> Link::getParentJoint() const {
  return parent_joint_;
}

std::string Link::getName() const {
  return name_;
}

Transformd Link::getTipTransform() const {
  return tip_transform_;
}

}
