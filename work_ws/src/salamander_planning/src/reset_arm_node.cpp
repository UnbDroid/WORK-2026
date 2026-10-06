#include <memory>
#include <string>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;

class ResetArmAction : public plansys2::ActionExecutorClient {
public:
  ResetArmAction() : plansys2::ActionExecutorClient("reset-arm", 500ms) {
    cmd_pub_ = this->create_publisher("/arm_command", 10);
    status_sub_ = this->create_subscription(
      "/arm_status", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) arm_finished_ = true;
      });
  }
protected:
  void do_work() override {
    if (!command_sent_) {
      std_msgs::msg::String cmd_msg;
      cmd_msg.data = "reset_arm";
      cmd_pub_->publish(cmd_msg);
      command_sent_ = true;
      arm_finished_ = false;
      send_feedback(0.0, "Acionando braço para resetar...");
    }
    if (arm_finished_) {
      command_sent_ = false;
      finish(true, 1.0, "Cubo recolhido da mesa.");
    }
  }
private:
  rclcpp::Publisher::SharedPtr cmd_pub_;
  rclcpp::Subscription::SharedPtr status_sub_;
  bool command_sent_ = false;
  bool arm_finished_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared();
  node->set_parameter(rclcpp::Parameter("action_name", "reset-arm"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}