#ifndef AD_KINEMATICS__TRANSFORMS_H
#define AD_KINEMATICS__TRANSFORMS_H

#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>

#include <iostream>

#include <geometry_msgs/Pose.h>

namespace ad_kinematics {

template<typename T> using Vector3 = Eigen::Matrix<T, 3, 1>;
template<typename T> using Affine3 = Eigen::Transform<T,3,Eigen::Affine>;

template<typename T> struct Transform {
  Eigen::Quaternion<T> rotation;
  Vector3<T> translation;

  Transform()
    : rotation(Eigen::Quaternion<T>(T(1.0), T(0.0), T(0.0), T(0.0))),
      translation(Vector3<T>(T(0.0), T(0.0), T(0.0))) {}

  Transform(const Eigen::Quaternion<T>& _rotation, const Vector3<T>& _translation) {
    rotation = _rotation;
    translation = _translation;
  }

  Transform inverse() {
    return Transform(rotation.inverse(), rotation.inverse() * translation * -1);
  }

  std::string toString() {
    std::stringstream ss;
    ss << "[" << translation(0) << ", " << translation(1) << ", " << translation(2) << "; "
       << rotation.w() << ", " << rotation.x() << ", " << rotation.y() << ", " << rotation.z() << "]";
    return ss.str();
  }

  template<typename T2>
  Transform<T2> cast() const {
    return Transform<T2>(rotation.template cast<T2>(), translation.template cast<T2>());
  }

  Affine3<T> toAffine() const {
   Affine3<T> affine(rotation);
   affine.translation() = translation;
   return affine;
 }
};

template <typename T> Transform<T> operator *(const Transform<T>& lhs, const Transform<T>& rhs) {
  return Transform<T>(lhs.rotation * rhs.rotation, lhs.translation + (lhs.rotation * rhs.translation));
}

template <typename T> Vector3<T> operator *(const Transform<T>& lhs, const Vector3<T>& rhs) {
  return lhs.translation + lhs.rotation * rhs;
}

typedef Transform<double> Transformd;

class Joint {
public:
  Joint(std::string name, const Eigen::Vector3d& origin, const Eigen::Vector3d& axis=Eigen::Vector3d::Zero(), int q_index=-1, double upper_limit=0.0, double lower_limit=0.0);
  virtual ~Joint();

  template<typename T> Transform<T> pose(const std::vector<T>& q) const {
    if (q_index_ != -1) {
      return pose<T>(q[q_index_]);
    } else {
      return pose<T>(T(0.0));
    }
  }

  template<typename T> Transform<T> pose(const T& q) const;

  void setMimic(int q_index, double multiplier, double offset);

  virtual bool isActuated() const = 0;
  virtual bool isActive() const;

  double getUpperLimit() const;

  double getLowerLimit() const;

  std::string getName() const;

  Eigen::Vector3d getOrigin() const;

  Eigen::Vector3d getAxis() const;

  int getQIndex() const;

protected:
  std::string name_;
  Eigen::Vector3d origin_;
  Eigen::Vector3d axis_;

  int q_index_;
  bool mimic_;
  double multiplier_;
  double offset_;

  double upper_limit_;
  double lower_limit_;
};

class FixedJoint : public Joint {
public:
  FixedJoint(std::string name, const Eigen::Vector3d& origin);

  template<typename T> Transform<T> pose(const T& /*q*/) const {
    return Transform<T>();
  }

  bool isActuated() const;
};

class RevoluteJoint : public Joint {
public:
  RevoluteJoint(std::string name, const Eigen::Vector3d& origin, const Eigen::Vector3d& axis, int q_index, double upper_limit, double lower_limit);

  template<typename T> Transform<T> pose(const T& q) const {
    return Transform<T>(Eigen::Quaternion<T>(Eigen::AngleAxis<T>(q, axis_.cast<T>())), origin_.cast<T>());
  }

  bool isActuated() const;
};

class ContinuousJoint : public RevoluteJoint {
public:
  ContinuousJoint(std::string name, const Eigen::Vector3d& origin, const Eigen::Vector3d& axis, int q_index);
};

// Workaround, because c++ doesn't allow virtual template functions
template <typename T> Transform<T> Joint::pose(const T &q) const {
  T q_scaled = T(multiplier_) * q + T(offset_);
  if (const FixedJoint* joint = dynamic_cast<const FixedJoint*>(this)) {
    return joint->pose(q_scaled);
  }
  else if (const RevoluteJoint* joint = dynamic_cast<const RevoluteJoint*>(this)) {
    return joint->pose(q_scaled);
  }
  else if (const ContinuousJoint* joint = dynamic_cast<const ContinuousJoint*>(this)) {
    return joint->pose(q_scaled);
  }
  else {
    std::cerr << "Dynamic cast failed!" << std::endl;
    return Transform<T>();
  }
}

class Link {
public:
  Link(std::string name, const Transformd& tip_transform, const std::shared_ptr<Joint>& joint, const std::shared_ptr<Link>& parent_link=nullptr);

  template<typename T> Transform<T> pose(const T& q) const {
    return parent_joint_->pose(q) * tip_transform_.cast<T>();
  }

  template<typename T> Transform<T> pose(const std::vector<T>& q) const {
    return parent_joint_->pose(q) * tip_transform_.cast<T>();
  }

  //Transform<double> pose(double q) const;
  std::shared_ptr<Joint> getParentJoint() const;
  std::shared_ptr<Link> getParentLink() const;
  std::string getName() const;
  Transformd getTipTransform() const;

private:
  std::string name_;
  Transformd tip_transform_;
  std::shared_ptr<Joint> parent_joint_;
  std::shared_ptr<Link> parent_link_;
};

geometry_msgs::Pose transformToMsg(const Transformd& transform);
Transformd msgToTransform(const geometry_msgs::Pose& msg);
}
#endif
