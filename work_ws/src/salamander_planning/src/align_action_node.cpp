#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;

class AlignAction : public plansys2::ActionExecutorClient
{
public:
  AlignAction()
  : plansys2::ActionExecutorClient("align_to_cube", 500ms)
  {
    //Cria o Publisher (quem acorda o launch do alinhamento)
    trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vision_trigger", 10);
    
    //Cria o Subscriber (Quem espera a confirmação do Python)
    status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/vision_status", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data == true) {
          vision_finished_ = true;
        }
      });
  }

protected:
  void do_work() override
  {
    // Acordar o Python (Roda só na primeira vez)
    if (!command_sent_) {
      std_msgs::msg::Bool trigger_msg;
      trigger_msg.data = true;
      trigger_pub_->publish(trigger_msg);
      
      command_sent_ = true;
      vision_finished_ = false;
      
      send_feedback(0.0, "Acordando a visão computacional...");
    }

    //Monitorar o trabalho
    if (vision_finished_) {
      // o launch terminou
      command_sent_ = false;
      vision_finished_ = false;
      
      finish(true, 1.0, "Alinhamento concluído com sucesso!");
    } else {
      // O launch ainda está trabalhando
      send_feedback(0.5, "Aguardando robô centralizar no cubo...");
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
  bool command_sent_ = false;
  bool vision_finished_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AlignAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "align_to_cube"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}