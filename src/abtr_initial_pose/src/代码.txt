#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include <cartographer_ros_msgs/FinishTrajectory.h>
#include <cartographer_ros_msgs/GetTrajectoryStates.h>
#include <cartographer_ros_msgs/StartTrajectory.h>
#include <cartographer_ros_msgs/TrajectoryStates.h>

class InitialPoseBridge {
public:
  InitialPoseBridge() : nh_(), pnh_("~") {
    // 从参数服务器读取配置
    pnh_.param<std::string>("configuration_directory", configuration_directory_,
                            std::string(""));
    pnh_.param<std::string>("configuration_basename", configuration_basename_,
                            std::string(""));
    pnh_.param<int>("relative_to_trajectory_id", relative_to_trajectory_id_, 0);
    pnh_.param<std::string>("expected_frame_id", expected_frame_id_,
                            std::string("map"));
    pnh_.param<bool>("auto_finish_active_trajectory",
                     auto_finish_active_trajectory_, true);

    if (configuration_directory_.empty() || configuration_basename_.empty()) {
      ROS_WARN_STREAM("initialpose_bridge: configuration_directory or "
                      "configuration_basename is empty. "
                      "You MUST set them via parameters, e.g.: "
                      "~configuration_directory: /xxx/configuration_files, "
                      "~configuration_basename: backpack_2d.lua");
    }

    // 创建 service client
    start_traj_client_ =
        nh_.serviceClient<cartographer_ros_msgs::StartTrajectory>(
            "start_trajectory");
    finish_traj_client_ =
        nh_.serviceClient<cartographer_ros_msgs::FinishTrajectory>(
            "finish_trajectory");
    get_traj_states_client_ =
        nh_.serviceClient<cartographer_ros_msgs::GetTrajectoryStates>(
            "get_trajectory_states");

    ROS_INFO("initialpose_bridge: waiting for Cartographer services...");
    start_traj_client_.waitForExistence();
    finish_traj_client_.waitForExistence();
    get_traj_states_client_.waitForExistence();
    ROS_INFO("initialpose_bridge: Cartographer services are available.");

    // 订阅 RViz 的 2D Pose Estimate
    initialpose_sub_ = nh_.subscribe(
        "initialpose", 1, &InitialPoseBridge::initialPoseCallback, this);

    ROS_INFO_STREAM("initialpose_bridge: ready. "
                    << "configuration_directory = " << configuration_directory_
                    << ", configuration_basename = " << configuration_basename_
                    << ", relative_to_trajectory_id = "
                    << relative_to_trajectory_id_
                    << ", expected_frame_id = " << expected_frame_id_);
  }

private:
  // 从 Cartographer 查询当前 ACTIVE 的 trajectory_id（排除 map 轨迹）
  bool getActiveTrajectoryId(int &active_id) {
    cartographer_ros_msgs::GetTrajectoryStates srv;
    if (!get_traj_states_client_.call(srv)) {
      ROS_ERROR(
          "initialpose_bridge: failed to call get_trajectory_states service.");
      return false;
    }

    const auto &ids = srv.response.trajectory_states.trajectory_id;
    const auto &states = srv.response.trajectory_states.trajectory_state;

    if (ids.size() != states.size()) {
      ROS_ERROR(
          "initialpose_bridge: trajectory_states id/state size mismatch.");
      return false;
    }

    // 查找一个 ACTIVE 的轨迹，且不等于 relative_to_trajectory_id_
    for (size_t i = 0; i < ids.size(); ++i) {
      if (states[i] == cartographer_ros_msgs::TrajectoryStates::ACTIVE &&
          ids[i] != relative_to_trajectory_id_) {
        active_id = ids[i];
        return true;
      }
    }
    return false;
  }

  void initialPoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    ROS_INFO("initialpose_bridge: received /initialpose.");

    if (!expected_frame_id_.empty() &&
        msg->header.frame_id != expected_frame_id_) {
      ROS_WARN_STREAM(
          "initialpose_bridge: /initialpose frame_id = "
          << msg->header.frame_id
          << ", but expected_frame_id = " << expected_frame_id_
          << ". Make sure RViz 2D Pose Estimate is in the map frame.");
    }

    // 1. 如果已有 ACTIVE 的定位轨迹，则先 finish 掉
    int active_id = -1;
    if (auto_finish_active_trajectory_ && getActiveTrajectoryId(active_id)) {
      ROS_INFO_STREAM(
          "initialpose_bridge: finishing ACTIVE trajectory id = " << active_id);

      cartographer_ros_msgs::FinishTrajectory finish_srv;
      finish_srv.request.trajectory_id = active_id;

      if (!finish_traj_client_.call(finish_srv)) {
        ROS_ERROR("initialpose_bridge: failed to call finish_trajectory.");
      } else {
        ROS_INFO_STREAM("initialpose_bridge: finish_trajectory status = "
                        << finish_srv.response.status.message);
      }
    } else {
      ROS_INFO("initialpose_bridge: no ACTIVE trajectory to finish (or "
               "auto_finish disabled).");
    }

    // 2. 调 start_trajectory，用 RViz 的位姿作为 initial_pose
    if (configuration_directory_.empty() || configuration_basename_.empty()) {
      ROS_ERROR("initialpose_bridge: configuration_directory or "
                "configuration_basename is empty, "
                "cannot start trajectory.");
      return;
    }

    cartographer_ros_msgs::StartTrajectory start_srv;
    start_srv.request.configuration_directory = configuration_directory_;
    start_srv.request.configuration_basename = configuration_basename_;
    start_srv.request.use_initial_pose = true;
    start_srv.request.initial_pose = msg->pose.pose;
    start_srv.request.relative_to_trajectory_id = relative_to_trajectory_id_;

    ROS_INFO_STREAM(
        "initialpose_bridge: calling start_trajectory with initial pose "
        << "relative_to_trajectory_id = " << relative_to_trajectory_id_);

    if (!start_traj_client_.call(start_srv)) {
      ROS_ERROR("initialpose_bridge: failed to call start_trajectory.");
      return;
    }

    if (start_srv.response.status.code != 0) // 0 一般是 OK
    {
      ROS_ERROR_STREAM("initialpose_bridge: start_trajectory failed: "
                       << start_srv.response.status.message);
    } else {
      ROS_INFO_STREAM("initialpose_bridge: started new trajectory, id = "
                      << start_srv.response.trajectory_id);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber initialpose_sub_;
  ros::ServiceClient start_traj_client_;
  ros::ServiceClient finish_traj_client_;
  ros::ServiceClient get_traj_states_client_;

  std::string configuration_directory_;
  std::string configuration_basename_;
  int relative_to_trajectory_id_;
  std::string expected_frame_id_;
  bool auto_finish_active_trajectory_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "cartographer_initialpose_bridge");
  InitialPoseBridge bridge;
  ros::spin();
  return 0;
}

