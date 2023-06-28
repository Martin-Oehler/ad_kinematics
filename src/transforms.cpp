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
Link::Link(std::string name, const Transformd& tip_transform, const std::shared_ptr<Joint>& joint, const std::shared_ptr<Link>& parent_link)
  : name_(name), tip_transform_(joint->pose<double>(0).inverse() * tip_transform), parent_joint_(joint), parent_link_(parent_link) {}

std::shared_ptr<Joint> Link::getParentJoint() const {
  return parent_joint_;
}

std::shared_ptr<Link> Link::getParentLink() const {
  return parent_link_;
}

std::string Link::getName() const {
  return name_;
}

Transformd Link::getTipTransform() const {
  return tip_transform_;
}

Joint::Joint(std::string name, const Eigen::Vector3d& origin, const Eigen::Vector3d& axis, int q_index, double upper_limit, double lower_limit)
  : name_(name), origin_(origin), axis_(axis), q_index_(q_index), mimic_(false), multiplier_(1.0), offset_(0.0), upper_limit_(upper_limit), lower_limit_(lower_limit) {}

Joint::~Joint() {}

double Joint::getUpperLimit() const {
  return upper_limit_;
}

double Joint::getLowerLimit() const {
  return lower_limit_;
}

std::string Joint::getName() const {
  return name_;
}

Eigen::Vector3d Joint::getOrigin() const {
  return origin_;
}

Eigen::Vector3d Joint::getAxis() const {
  return axis_;
}

int Joint::getQIndex() const {
  return q_index_;
}

void Joint::setMimic(int q_index, double multiplier, double offset)
{
  mimic_ = true;
  q_index_ = q_index;
  multiplier_ = multiplier;
  offset_ = offset;
}

bool Joint::isActive() const
{
  return !isActuated() && mimic_;
}

}
