#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;

class PutCubeContainerAction : public plansys2::ActionExecutorClient {
public:
  PutCubeContainerAction()
  : plansys2::ActionExecutorClient("put_cube_container_node", 500ms)
  {
    cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/arm_command", 10);
    status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/arm_status", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && command_sent_) arm_finished_ = true;
      });
  }

protected:
  void do_work() override {
    // PDDL: put-cube-container ?w ?c ?k  ->  [0]=w, [1]=c, [2]=k
    const std::string container = get_arguments()[2];

    if (!command_sent_) {
      std_msgs::msg::String cmd_msg;
      cmd_msg.data = "put_cube_container_" + container;
      cmd_pub_->publish(cmd_msg);
      command_sent_ = true;
      arm_finished_ = false;
      start_time_ = this->now();
      send_feedback(0.0, "Soltando cubo em " + container + "...");
      return;
    }

    if (arm_finished_) {
      command_sent_ = false;
      arm_finished_ = false;
      finish(true, 1.0, "Cubo guardado em " + container);
      return;
    }

    if ((this->now() - start_time_).seconds() > timeout_s_) {
      command_sent_ = false;
      finish(false, 0.0, "Timeout soltando cubo em " + container);
      return;
    }

    send_feedback(0.5, "Braço em movimento...");
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
  rclcpp::Time start_time_;
  bool command_sent_ = false;
  bool arm_finished_ = false;
  double timeout_s_ = 30.0;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PutCubeContainerAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "put-cube-container"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}