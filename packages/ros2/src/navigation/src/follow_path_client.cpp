#include <chrono>
#include <memory>
#include <string>

#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class FollowPathClient : public rclcpp::Node {
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  FollowPathClient() : rclcpp::Node("follow_path_client") {
    global_path_topic_ = this->declare_parameter<std::string>("global_path_topic", "/global_path");
    action_name_ = this->declare_parameter<std::string>("action_name", "/follow_path");
    controller_id_ = this->declare_parameter<std::string>("controller_id", "FollowPath");
    goal_checker_id_ = this->declare_parameter<std::string>("goal_checker_id", "general_goal_checker");

    client_ = rclcpp_action::create_client<FollowPath>(this, action_name_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(global_path_topic_, qos, std::bind(&FollowPathClient::onPath, this, std::placeholders::_1));
  }

private:
  void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!msg) return;
    if (msg->poses.size() < 2) return;
    if (sent_) return;

    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "FollowPath action server not available on %s", action_name_.c_str());
      return;
    }

    FollowPath::Goal goal;
    goal.path = *msg;
    goal.controller_id = controller_id_;
    goal.goal_checker_id = goal_checker_id_;

    rclcpp_action::Client<FollowPath>::SendGoalOptions opts;

    opts.goal_response_callback = [this](const GoalHandle::SharedPtr &handle) {
      if (!handle) {
        RCLCPP_ERROR(this->get_logger(), "FollowPath goal rejected");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "FollowPath goal accepted");
    };

    opts.result_callback = [this](const GoalHandle::WrappedResult &res) {
      RCLCPP_INFO(this->get_logger(), "FollowPath result code %d", static_cast<int>(res.code));
    };

    client_->async_send_goal(goal, opts);
    sent_ = true;
  }

  std::string global_path_topic_;
  std::string action_name_;
  std::string controller_id_;
  std::string goal_checker_id_;

  bool sent_{false};

  rclcpp_action::Client<FollowPath>::SharedPtr client_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowPathClient>());
  rclcpp::shutdown();
  return 0;
}
