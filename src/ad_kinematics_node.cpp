#include <ros/ros.h>

#include <ad_kinematics/chain.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ad_kinematics_node");
  ros::NodeHandle pnh("~");

  std::string group_name = pnh.param<std::string>("group_name", "arm_group");
  ROS_INFO_STREAM("Loading group '" << group_name << "'.");

  ad_kinematics::Chain chain(group_name);

  std::vector<double> joint_states = {0.0, 1.55, 2.94, 1.61, 0.0, 0.0};
// TF ECHO:
//  - Translation: [0.094, 0.067, 1.460]
//  - Rotation: in Quaternion [-0.706, -0.024, -0.080, 0.704]
//  std::vector<double> joint_states = {-2.77, 2.59, 4.77, 1.72, -0.83};
//  TF ECHO:
//    - Translation: [-0.536, -0.219, 0.027]
//    - Rotation: in Quaternion [0.603, 0.494, 0.289, 0.556]
//  std::vector<double> joint_states = {0, 3, 4.71612, 0.75, 0};



  ad_kinematics::Transformd transform = chain.computeTipTransform<double>(joint_states);
//  ad_kinematics::Transformd transform = chain.computeTransform<double>("gripper_finger_link_0", joint_states);
  // TODO add asserts
  ROS_INFO_STREAM("Transform from "<< chain.getBaseLinkName() << " to " << chain.getTipLinkName() << ":");
  ROS_INFO_STREAM(transform.toString());

  std::vector<std::string> joint_names = {"arm_joint_1", "sensor_head_yaw_joint"};
  std::vector<int> indices = chain.getJointQIndices(joint_names);
  ROS_INFO_STREAM("Joint indices:");
  for (unsigned int i = 0; i < joint_names.size(); i++) {
    ROS_INFO_STREAM(joint_names[i] << " --> " << indices[i]);
  }

//  ros::spin();

  return 0;
}
