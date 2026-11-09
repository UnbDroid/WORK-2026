#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;

class StackCubeTopAction : public plansys2::ActionExecutorClient {
public:
  StackCubeTopAction() : plansys2::ActionExecutorClient("stack_cube_top", 500ms) {
    arm_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/arm_command", 10);
    arm_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/arm_status", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) arm_finished_ = true;
      });
  }
protected:
  void do_work() override {
    // PDDL: stack_cube_top ?w - workspace ?c - cube ?t_desk - tag
    std::string cube = get_arguments()[1];

    if (!command_sent_) {
      std_msgs::msg::String cmd_msg;
      cmd_msg.data = "PLACE_TOP"; // Comando específico para o braço soltar no topo
      arm_cmd_pub_->publish(cmd_msg);
      command_sent_ = true;
      arm_finished_ = false;
      send_feedback(0.0, "Manipulador descendo para soltar o cubo (" + cube + ")...");
    }
    
    if (arm_finished_) {
      command_sent_ = false;
      finish(true, 1.0, "Cubo " + cube + " posicionado no topo com sucesso.");
    }
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_status_sub_;
  bool command_sent_ = false;
  bool arm_finished_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StackCubeTopAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "stack_cube_top"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}