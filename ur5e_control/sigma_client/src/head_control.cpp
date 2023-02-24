#include "sigma_client/quaternions.h"
#include "sword/Stick.h"
using namespace sword;

#define PUBLISH

quaTransform::quaTransform()
{

#ifdef PUBLISH
  // topic to cartesian controllers
  target_pub_right = nh.advertise<geometry_msgs::PoseStamped>("/right/cartesian_motion_controller/target_frame", 1);
  sub_current_pose_right = nh.subscribe("/right/cartesian_motion_controller/current_pose", 1, &quaTransform::current_pose_callback_right, this);
#else
  first_flag_right = 2;
#endif

  //  path_pub_left = nh.advertise<nav_msgs::Path>("/left/path", 1);
  //  path_pub_right = nh.advertise<nav_msgs::Path>("/right/path", 1);

  // subscribe from sigma7 devices
  // pose
  sub_sigma_right = nh.subscribe("/sigma7/sigma0/pose", 1, &quaTransform::callback_right, this);
  // buttons
  sub_sigma_button_right = nh.subscribe("/sigma7/sigma0/buttons", 1, &quaTransform::callback_button_right, this);

  head_client = nh.serviceClient<sword::Stick>("backend_infer");

  // path config
  f = boost::bind(&quaTransform::configCallback, this, _1, _2);
  server.setCallback(f);
}

void quaTransform::current_pose_callback_right(const geometry_msgs::PoseStampedConstPtr &msgs)
{
  current_pose_right.pose = msgs->pose;
  if (first_flag_right == 0)
    first_flag_right = 2;
}

void quaTransform::configCallback(sigma_client::PathGenerationConfig &config, uint32_t level)
{
  path_config = config.Path_command;
  // ROS_INFO("here...");
}

void quaTransform::callback_right(const geometry_msgs::PoseStampedConstPtr &last_msgs_right)
{
  // ROS_INFO("callback right");
  q_transform_right.x() = 0.0;
  q_transform_right.y() = 0.0;
  q_transform_right.z() = 0.7071068;
  q_transform_right.w() = 0.7071068;

  // current sigma orientation
  q_sigma_right.x() = last_msgs_right->pose.orientation.x;
  q_sigma_right.y() = last_msgs_right->pose.orientation.y;
  q_sigma_right.z() = last_msgs_right->pose.orientation.z;
  q_sigma_right.w() = last_msgs_right->pose.orientation.w;

  q_target_right = q_transform_right * q_sigma_right;

  geometry_msgs::PoseStamped target_pose_right;

  target_pose_right.header.stamp = ros::Time::now();
  target_pose_right.header.frame_id = "base_link";

  delta_position[3] = last_msgs_right->pose.position.x - last_sigma_right.pose.position.x;
  delta_position[4] = last_msgs_right->pose.position.y - last_sigma_right.pose.position.y;
  delta_position[5] = last_msgs_right->pose.position.z - last_sigma_right.pose.position.z;

  q_cur_right.x() = q_target_right.x();
  q_cur_right.y() = q_target_right.y();
  q_cur_right.z() = q_target_right.z();
  q_cur_right.w() = q_target_right.w();
  q_last_right.x() = last_sigma_right.pose.orientation.x;
  q_last_right.y() = last_sigma_right.pose.orientation.y;
  q_last_right.z() = last_sigma_right.pose.orientation.z;
  q_last_right.w() = last_sigma_right.pose.orientation.w;

  delta_q_right = q_cur_right * q_last_right.conjugate();

  if (first_flag_right == 2 && button_right == 0)
  {
    // initialize the pose
    target_pose_right.pose.position.x = 0.0;
    target_pose_right.pose.position.y = 0.35;
    target_pose_right.pose.position.z = 0.35;
    target_pose_right.pose.orientation.x = 0.0;
    target_pose_right.pose.orientation.y = 1.0;
    target_pose_right.pose.orientation.z = 0.0;
    target_pose_right.pose.orientation.w = 0.0;

    // auto srv = Stick();
    // srv.request.cur_p[0] = target_pose_right.pose.position.x;
    // srv.request.cur_p[1] = target_pose_right.pose.position.y;
    // srv.request.cur_p[2] = target_pose_right.pose.position.z;
    // srv.request.cur_r[0] = target_pose_right.pose.orientation.x;
    // srv.request.cur_r[1] = target_pose_right.pose.orientation.y;
    // srv.request.cur_r[2] = target_pose_right.pose.orientation.z;
    // srv.request.cur_r[3] = target_pose_right.pose.orientation.w;
    // srv.request.tar_p[0] = target_pose_right.pose.position.x;
    // srv.request.tar_p[1] = target_pose_right.pose.position.y;
    // srv.request.tar_p[2] = target_pose_right.pose.position.z - 0.01;
    // srv.request.tar_r[0] = target_pose_right.pose.orientation.x;
    // srv.request.tar_r[1] = target_pose_right.pose.orientation.y;
    // srv.request.tar_r[2] = target_pose_right.pose.orientation.z;
    // srv.request.tar_r[3] = target_pose_right.pose.orientation.w;
    // head_client.call(srv);

    ROS_INFO("%f, %f, %f", target_pose_right.pose.position.x, target_pose_right.pose.position.y, target_pose_right.pose.position.z);
#ifdef PUBLISH
    target_pub_right.publish(target_pose_right);
#endif
    last_pose_right.pose = target_pose_right.pose;
    first_flag_right = 1;
  }
  else if (first_flag_right == 1 && button_right == 1)
  {
    // main logic
    ur_q_cur_right.x() = last_pose_right.pose.orientation.x;
    ur_q_cur_right.y() = last_pose_right.pose.orientation.y;
    ur_q_cur_right.z() = last_pose_right.pose.orientation.z;
    ur_q_cur_right.w() = last_pose_right.pose.orientation.w;

    ur_q_target_right = delta_q_right * ur_q_cur_right;

    // build the srv
    auto srv = Stick();

    // assign the data
    srv.request.cur_p[0] = last_pose_right.pose.position.x;
    srv.request.cur_p[1] = last_pose_right.pose.position.y;
    srv.request.cur_p[2] = last_pose_right.pose.position.z;
    srv.request.cur_r[0] = last_pose_right.pose.orientation.x;
    srv.request.cur_r[1] = last_pose_right.pose.orientation.y;
    srv.request.cur_r[2] = last_pose_right.pose.orientation.z;
    srv.request.cur_r[3] = last_pose_right.pose.orientation.w;
    srv.request.tar_p[0] = last_pose_right.pose.position.x - delta_position[4];
    srv.request.tar_p[1] = last_pose_right.pose.position.y + delta_position[3];
    srv.request.tar_p[2] = last_pose_right.pose.position.z + delta_position[5];
    srv.request.tar_r[0] = ur_q_target_right.x();
    srv.request.tar_r[1] = ur_q_target_right.y();
    srv.request.tar_r[2] = ur_q_target_right.z();
    srv.request.tar_r[3] = ur_q_target_right.w();
    // invoke the method
    head_client.call(srv);

    // compose the actual pose
    // auto actual_p = srv.response.v;
    // auto actual_r = srv.response.w;
    target_pose_right.pose.position.x = srv.response.v[0];
    target_pose_right.pose.position.y = srv.response.v[1];
    target_pose_right.pose.position.z = srv.response.v[2];
    target_pose_right.pose.orientation.x = srv.response.w[0];
    target_pose_right.pose.orientation.y = srv.response.w[1];
    target_pose_right.pose.orientation.z = srv.response.w[2];
    target_pose_right.pose.orientation.w = srv.response.w[3];

#ifdef PUBLISH
    target_pub_right.publish(target_pose_right);
#endif
    ROS_INFO("%f, %f, %f", target_pose_right.pose.position.x, target_pose_right.pose.position.y, target_pose_right.pose.position.z);
    last_pose_right.pose = target_pose_right.pose;
  }
  else
  {
    // keep the last pose
    last_pose_right.header.stamp = ros::Time::now();
    last_pose_right.header.frame_id = "base_link";
#ifdef PUBLISH
    target_pub_right.publish(last_pose_right);
#endif
    // ROS_INFO("keep the pose %d, %d", first_flag_right, button_right);
  }

  last_sigma_right.pose.position = last_msgs_right->pose.position;
  last_sigma_right.pose.orientation.x = q_target_right.x();
  last_sigma_right.pose.orientation.y = q_target_right.y();
  last_sigma_right.pose.orientation.z = q_target_right.z();
  last_sigma_right.pose.orientation.w = q_target_right.w();
}

void quaTransform::callback_button_right(const sensor_msgs::JoyConstPtr &last_button_right)
{
  button_right = last_button_right->buttons[0];
}

Eigen::Quaterniond quaTransform::scaleRotation(Eigen::Quaterniond &q_current, double scale)
{
  Eigen::Quaterniond scale_q;
  std::vector<double> eulerAngles = toEulerAngle(q_current);
  for (int i = 0; i < 3; i++)
  {
    eulerAngles[0] *= scale;
    eulerAngles[1] *= scale;
    eulerAngles[2] *= scale;
  }
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngles[0], ::Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngles[1], ::Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngles[2], ::Eigen::Vector3d::UnitZ()));
  scale_q = rollAngle * pitchAngle * yawAngle;

  return scale_q;
}

std::vector<double> quaTransform::toEulerAngle(Eigen::Quaterniond &q)
{
  std::vector<double> eulerAngles(3);
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  eulerAngles[0] = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    eulerAngles[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    eulerAngles[1] = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  eulerAngles[2] = atan2(siny_cosp, cosy_cosp);

  return eulerAngles;
}

/*

lsusb
sudo chmod 777 /dev/bus/usb/001/007
roslaunch sigma7 sigma.launch
roslaunch ur_robot_driver right_motion.launch
*/
int main(int argc, char **argv)
{
  // Initiate ROS
  ros::init(argc, argv, "head");
  ROS_INFO("head: starting ...");
  // Create an object of class SubscribeAndPublish that will take care of everything
  quaTransform transform;

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  // ros::spin();

  return 0;
}