#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AlignTagPrecisionAction : public plansys2::ActionExecutorClient {
public:
  AlignTagPrecisionAction()
  : plansys2::ActionExecutorClient("align_tag_precision", 500ms) {
    target_pub_ = this->create_publisher<std_msgs::msg::String>("/vision_target", 10);
    status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/vision_status", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) vision_finished_ = true;
      });
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override {
    command_sent_ = false;
    vision_finished_ = false;
    target_pub_->on_activate();
    return ActionExecutorClient::on_activate(state);
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override {
    target_pub_->on_deactivate();
    return ActionExecutorClient::on_deactivate(state);
  }

protected:
  void do_work() override {
    if (!command_sent_) {
      std_msgs::msg::String cmd_msg;
      cmd_msg.data = "precisao:start";
      target_pub_->publish(cmd_msg);
      command_sent_ = true;
      vision_finished_ = false;
      send_feedback(0.0, "Visão reorganizando...");
    }
    if (vision_finished_) {
      command_sent_ = false;
      finish(true, 1.0, "Reorganização concluída");
    }
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr target_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
  bool command_sent_ = false;
  bool vision_finished_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AlignTagPrecisionAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "align_tag_precision"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}