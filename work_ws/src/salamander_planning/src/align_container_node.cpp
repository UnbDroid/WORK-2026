#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;

class AlignContainerAction : public plansys2::ActionExecutorClient {
public:
  AlignContainerAction()
  : plansys2::ActionExecutorClient("align_container_node", 500ms)
  {
    target_pub_ = this->create_publisher<std_msgs::msg::String>("/vision_target", 10);

    status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/vision_status", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && command_sent_) {   // só vale se já enviamos o comando
          vision_finished_ = true;
        }
      });
  }

protected:
  void do_work() override {
    // PDDL: align-container ?w - workspace ?c - cube ?k - container
    const std::string container = get_arguments()[2];   // "container-blue"

    if (!command_sent_) {
      std_msgs::msg::String msg;
      msg.data = "container:" + container;                // a visão remove o prefixo "container-"
      target_pub_->publish(msg);

      command_sent_ = true;
      vision_finished_ = false;
      start_time_ = this->now();
      send_feedback(0.0, "Procurando " + container + "...");
      return;
    }

    if (vision_finished_) {
      reset_state();
      finish(true, 1.0, "Robô alinhado com " + container);
      return;
    }

    if ((this->now() - start_time_).seconds() > timeout_s_) {
      reset_state();
      finish(false, 0.0, "Timeout alinhando com " + container);
      return;
    }

    send_feedback(0.5, "Alinhando com " + container + "...");
  }

private:
  void reset_state() {
    command_sent_ = false;
    vision_finished_ = false;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
  rclcpp::Time start_time_;
  bool command_sent_ = false;
  bool vision_finished_ = false;
  double timeout_s_ = 20.0;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AlignContainerAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "align-container"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}