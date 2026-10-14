#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;

class AlignAction : public plansys2::ActionExecutorClient {
public:
  AlignAction() : plansys2::ActionExecutorClient("align_to_cube", 500ms) {
    target_pub_ = this->create_publisher<std_msgs::msg::String>("/vision_target", 10);
    status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/vision_status", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) vision_finished_ = true;
      });
  }
protected:
  void do_work() override {
    // PDDL: align_to_cube ?w - workspace ?c - cube ?t - tag
    std::string target_tag = get_arguments()[2]; 

    if (!command_sent_) {
      std_msgs::msg::String cmd_msg;
      cmd_msg.data = target_tag; // Envia "tag1", "tag2", etc.
      target_pub_->publish(cmd_msg);
      command_sent_ = true;
      vision_finished_ = false;
      send_feedback(0.0, "Acordando visão para procurar o cubo da " + target_tag + "...");
    }
    
    if (vision_finished_) {
      command_sent_ = false;
      finish(true, 1.0, "Robô centralizado com a " + target_tag);
    }
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
  bool command_sent_ = false;
  bool vision_finished_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AlignAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "align_to_cube"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}