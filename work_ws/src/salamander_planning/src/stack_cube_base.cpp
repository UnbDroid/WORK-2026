#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;

class StackCubeBaseAction : public plansys2::ActionExecutorClient {
public:
  StackCubeBaseAction() : plansys2::ActionExecutorClient("stack_cube_base", 500ms) {
    arm_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/arm_command", 10);
    arm_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/arm_status", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) arm_finished_ = true;
      });
  }
protected:
  void do_work() override {
    // PDDL: stack_cube_base ?w - workspace ?c_base - cube ?s1 - slot ?t_desk - tag ?c_top - cube
    std::string workspace = get_arguments()[0];
    std::string cube = get_arguments()[1];
    std::string slot = get_arguments()[2];

    if (!command_sent_) {
      std_msgs::msg::String cmd_msg;
      cmd_msg.data = "PLACE_BASE_" + workspace + "_" + slot; 
      arm_cmd_pub_->publish(cmd_msg);
      command_sent_ = true;
      arm_finished_ = false;
      send_feedback(0.0, "Manipulador buscando a base " + cube + " no " + slot + " para soltar em " + workspace + "...");
    }
    
    if (arm_finished_) {
      command_sent_ = false;
      finish(true, 1.0, "Cubo " + cube + " posicionado na base com sucesso.");
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
  auto node = std::make_shared<StackCubeBaseAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "stack_cube_base"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}