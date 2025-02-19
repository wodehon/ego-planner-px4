#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "ego_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"

ros::Publisher pos_cmd_pub;
ros::Publisher pos_cmd_arc_pub;

quadrotor_msgs::PositionCommand cmd;
std_msgs::Float32MultiArray arc_cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

// Eigen::Vector3d came_to_world(const Eigen::Vector3d& p_c){
//   tf::TransformListener tf_listener;

//   return 
// }

void bsplineCallback(ego_planner::BsplineConstPtr msg) 
{
  ROS_WARN("bsplineCallback begin");
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  ROS_WARN("pos_pts init finish");

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }
  // ROS_WARN("knots init finish");

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }
  // ROS_WARN("pos_pts set finish");

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);
  // ROS_WARN("pos_traj setKnot finish");

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  // ROS_WARN("pos_traj[0] set finish");
  traj_.push_back(traj_[0].getDerivative());
  // ROS_WARN("pos_traj[1] set finish");
  traj_.push_back(traj_[1].getDerivative());
  // ROS_WARN("pos_traj[2] set finish");
  traj_.push_back(traj_[2].getDerivative());
  // ROS_WARN("pos_traj[3] set finish");
  // traj_.push_back(traj_[3].getDerivative());
  // ROS_WARN("pos_traj[4] set finish");


  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
  // ROS_WARN("bsplineCallback end, receive_traj_");
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

// // 从yolo计算
// std::pair<double, double> calculate_yaw(Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
// {
//   constexpr double PI = 3.1415926;
//   constexpr double YAW_DOT_MAX_PER_SEC = PI;
//   // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
//   std::pair<double, double> yaw_yawdot(0, 0);
//   double yaw = 0;
//   double yawdot = 0;

//   Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
//   double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
//   double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
//   if (yaw_temp - last_yaw_ > PI)
//   {
//     if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
//     {
//       yaw = last_yaw_ - max_yaw_change;
//       if (yaw < -PI)
//         yaw += 2 * PI;

//       yawdot = -YAW_DOT_MAX_PER_SEC;
//     }
//     else
//     {
//       yaw = yaw_temp;
//       if (yaw - last_yaw_ > PI)
//         yawdot = -YAW_DOT_MAX_PER_SEC;
//       else
//         yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
//     }
//   }
//   else if (yaw_temp - last_yaw_ < -PI)
//   {
//     if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
//     {
//       yaw = last_yaw_ + max_yaw_change;
//       if (yaw > PI)
//         yaw -= 2 * PI;

//       yawdot = YAW_DOT_MAX_PER_SEC;
//     }
//     else
//     {
//       yaw = yaw_temp;
//       if (yaw - last_yaw_ < -PI)
//         yawdot = YAW_DOT_MAX_PER_SEC;
//       else
//         yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
//     }
//   }
//   else
//   {
//     if (yaw_temp - last_yaw_ < -max_yaw_change)
//     {
//       yaw = last_yaw_ - max_yaw_change;
//       if (yaw < -PI)
//         yaw += 2 * PI;

//       yawdot = -YAW_DOT_MAX_PER_SEC;
//     }
//     else if (yaw_temp - last_yaw_ > max_yaw_change)
//     {
//       yaw = last_yaw_ + max_yaw_change;
//       if (yaw > PI)
//         yaw -= 2 * PI;

//       yawdot = YAW_DOT_MAX_PER_SEC;
//     }
//     else
//     {
//       yaw = yaw_temp;
//       if (yaw - last_yaw_ > PI)
//         yawdot = -YAW_DOT_MAX_PER_SEC;
//       else if (yaw - last_yaw_ < -PI)
//         yawdot = YAW_DOT_MAX_PER_SEC;
//       else
//         yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
//     }
//   }

//   if (fabs(yaw - last_yaw_) <= max_yaw_change)
//     yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
//   yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
//   last_yaw_ = yaw;
//   last_yaw_dot_ = yawdot;

//   yaw_yawdot.first = yaw;
//   yaw_yawdot.second = yawdot;

//   return yaw_yawdot;
// }

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jerk(Eigen::Vector3d::Zero()), snap(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    jerk = traj_[3].evaluateDeBoorT(t_cur);
    // snap = traj_[4].evaluateDeBoorT(t_cur);
    snap.setZero();
    // ROS_WARN("cal snap finish");


    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    jerk.setZero();
    snap.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd);

  // time,xyz,v_xyz,a_xyz,yaw,yaw_dot,jerk_xyz,snap_xyz
  arc_cmd.data.clear();

  // arc_cmd.data.push_back(time_now.toSec());
  arc_cmd.data.push_back(time_now.toNSec()/1000);

  arc_cmd.data.push_back(pos(0));
  arc_cmd.data.push_back(pos(1));
  arc_cmd.data.push_back(pos(2));

  arc_cmd.data.push_back(vel(0));
  arc_cmd.data.push_back(vel(1));
  arc_cmd.data.push_back(vel(2));

  arc_cmd.data.push_back(acc(0));
  arc_cmd.data.push_back(acc(1));
  arc_cmd.data.push_back(acc(2));

  arc_cmd.data.push_back(yaw_yawdot.first);
  arc_cmd.data.push_back(yaw_yawdot.second);

  arc_cmd.data.push_back(jerk(0));
  arc_cmd.data.push_back(jerk(1));
  arc_cmd.data.push_back(jerk(2));

  arc_cmd.data.push_back(snap(0));
  arc_cmd.data.push_back(snap(1));
  arc_cmd.data.push_back(snap(2));

  pos_cmd_arc_pub.publish(arc_cmd);
  ROS_INFO("cmd pub finish!");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);

  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  pos_cmd_arc_pub = node.advertise<std_msgs::Float32MultiArray>("/reference_trajectory", 50);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}