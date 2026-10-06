#include <memory>
#include <string>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;

class PutCubeSlotAction : public plansys2::ActionExecutorClient {
public:
  PutCubeSlotAction() : plansys2::ActionExecutorClient("put-cube-slot", 500ms) {
    cmd_pub_ = this->create_publisher("/arm_command", 10);
    status_sub_ = this->create_subscription(
      "/arm_status", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) arm_finished_ = true;
      });
  }
protected:
  void do_work() override {
    if (!command_sent_) {
      // No PDDL: (put-cube-slot ?c ?s ?pos-s)
      // O argumento [0] é o cubo (ex: cubo1)
      // O argumento [1] é o slot (ex: slot1)
      std::string target_slot = get_arguments()[1]; 
      
      std_msgs::msg::String cmd_msg;
      cmd_msg.data = "put_" + target_slot; // Gera "put_slot1", "put_slot2", etc.
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
  rclcpp::Publisher::SharedPtr cmd_pub_;
  rclcpp::Subscription::SharedPtr status_sub_;
  bool command_sent_ = false;
  bool arm_finished_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared();
  node->set_parameter(rclcpp::Parameter("action_name", "put-cube-slot"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}