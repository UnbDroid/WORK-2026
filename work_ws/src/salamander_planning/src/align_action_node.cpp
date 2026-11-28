#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"   // NOVO

using namespace std::chrono_literals;

class AlignAction : public plansys2::ActionExecutorClient {
public:
  AlignAction() : plansys2::ActionExecutorClient("align_tag", 500ms) {
    target_pub_ = this->create_publisher<std_msgs::msg::String>("/vision_target", 10);
    status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/vision_status", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && command_sent_) vision_finished_ = true;
      });
  }

protected:
  void do_work() override {
    // PDDL: align_tag ?w - workspace ?t - tag
    std::string workspace = get_arguments()[0];
    std::string target_tag = get_arguments()[1];

    if (!command_sent_) {
      // NOVO: alinhamento numa zona de empilhamento (stacking-area w) -> modo "stack" da visão.
      // Vale para as duas ações: tag da mesa (antes do stack_cube_base) e
      // tag do cubo da base (antes do stack_cube_top). Os dois usam o z do perfil "stack".
      // Fora da zona de empilhamento -> modo pick, como antes.
      std::string mode;
      if (exists("(stacking-area " + workspace + ")")) {
        mode = "stack:";
      }

      std_msgs::msg::String cmd_msg;
      cmd_msg.data = mode + target_tag;
      target_pub_->publish(cmd_msg);
      RCLCPP_INFO(this->get_logger(), "Enviado à visão: %s", cmd_msg.data.c_str());

      command_sent_ = true;
      vision_finished_ = false;
      send_feedback(0.0, "Acordando visão para procurar a referência " + target_tag + "...");
      return;
    }

    if (vision_finished_) {
      command_sent_ = false;
      vision_finished_ = false;
      finish(true, 1.0, "Robô centralizado com a " + target_tag);
    }
  }

private:
  // Consulta o PlanSys2: o predicado existe no estado atual?
  bool exists(const std::string & predicate) {
    try {
      if (!problem_expert_) {
        problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
      }
      return problem_expert_->existPredicate(plansys2::Predicate(predicate));
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Não consegui consultar %s (%s); usando modo pick.",
                  predicate.c_str(), e.what());
      return false;
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  bool command_sent_ = false;
  bool vision_finished_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AlignAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "align_tag"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
