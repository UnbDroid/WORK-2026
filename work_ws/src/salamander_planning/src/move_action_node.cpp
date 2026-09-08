#include <memory>
#include <string>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move", 500ms)
  {
    // Cria o cliente de ação para navegação
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    
    // 1. Declara e lê a lista principal com os nomes dos pontos
    this->declare_parameter<std::vector<std::string>>("waypoints", std::vector<std::string>{});
    std::vector<std::string> wp_names;
    this->get_parameter("waypoints", wp_names);

    // 2. Loop automático: cria e lê as coordenadas de cada ponto da lista
    for (const auto & wp : wp_names) {
      this->declare_parameter<std::vector<double>>(wp + "_coords", std::vector<double>{0.0, 0.0, 0.0});
      std::vector<double> coords;
      this->get_parameter(wp + "_coords", coords);
      
      if (coords.size() == 3) {
        waypoints_[wp] = create_pose(coords[0], coords[1], coords[2]);
        RCLCPP_INFO(this->get_logger(), "Carregado: %s [X: %.2f, Y: %.2f, Yaw: %.2f]", wp.c_str(), coords[0], coords[1], coords[2]);
      }
    }
  }

protected:
  void do_work() override
  {
    if (!goal_sent_) {
      goal_sent_ = true;
      std::string dest = get_arguments()[2];
      
      // Trava de segurança: se o PDDL mandar para um ponto que não está no YAML
      if (waypoints_.find(dest) == waypoints_.end()) {
         send_feedback(0.0, "Erro: Ponto " + dest + " não existe no arquivo YAML");
         goal_sent_ = false;
         finish(false, 0.0, "Destino inválido");
         return;
      }

      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
      goal_msg.pose = waypoints_[dest];
      
      send_feedback(0.0, "Iniciando navegação para " + dest);
      
      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      
      send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
        goal_sent_ = false;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          finish(true, 1.0, "Destino alcançado");
        } else {  
          finish(false, 1.0, "Falha na navegação");
        }
      };
      
      nav_client_->async_send_goal(goal_msg, send_goal_options);
    }
  }

private:
  bool goal_sent_ = false;
  geometry_msgs::msg::PoseStamped create_pose(double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    
    return pose;
  }

  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}