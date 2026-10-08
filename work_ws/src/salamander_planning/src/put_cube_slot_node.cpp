#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;

class PutCubeSlotAction : public plansys2::ActionExecutorClient {
public:
  PutCubeSlotAction() : plansys2::ActionExecutorClient("put_cube_slot", 500ms) {
    cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/arm_command", 10);
    status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/arm_status", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) arm_finished_ = true;
      });
  }
protected:
  void do_work() override {
    if (!command_sent_) {
      // No PDDL: (put_cube_slot ?c ?s ?pos-s)
      std::string target_slot = get_arguments()[1]; 
      
      std_msgs::msg::String cmd_msg;
      cmd_msg.data = "put_" + target_slot; 
      cmd_pub_->publish(cmd_msg);
      
      command_sent_ = true;
      arm_finished_ = false;
      send_feedback(0.0, "Guardando cubo no " + target_slot + "...");
    }
    if (arm_finished_) {
      command_sent_ = false;
      finish(true, 1.0, "Cubo guardado com sucesso no slot.");
    }
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
  bool command_sent_ = false;
  bool arm_finished_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PutCubeSlotAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "put_cube_slot"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}