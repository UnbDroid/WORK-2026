#include <memory>
#include <string>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"   // NOVO: limite de velocidade perto do waypoint
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

    // NOVO: desaceleração perto do waypoint (mesas).
    // Quando faltar menos que 'slow_down_distance' metros de caminho, o robô passa a andar a
    // 'slow_down_percentage' % da velocidade máxima (vale para frente, lado e giro).
    speed_limit_pub_ = this->create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit", 10);
    this->declare_parameter<double>("slow_down_distance", 0.6);
    this->declare_parameter<double>("slow_down_percentage", 35.0);
    this->get_parameter("slow_down_distance", slow_down_distance_);
    this->get_parameter("slow_down_percentage", slow_down_percentage_);
    
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
      std::string origin = get_arguments()[0];
      std::string dest = get_arguments()[1];
      
      // LOG: O robô começou a se mover
      RCLCPP_INFO(this->get_logger(), ">> MOVIMENTAÇÃO: PALIO navegando de %s para %s...", origin.c_str(), dest.c_str());

      // Trava de segurança: se o PDDL mandar para um ponto que não está no YAML
      if (waypoints_.find(dest) == waypoints_.end()) {
         RCLCPP_ERROR(this->get_logger(), "!! ERRO: Ponto %s não existe no arquivo YAML de configuração!", dest.c_str());
         send_feedback(0.0, "Erro: Ponto " + dest + " não existe no arquivo YAML");
         goal_sent_ = false;
         finish(false, 0.0, "Destino inválido");
         return;
      }

      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
      goal_msg.pose = waypoints_[dest];
      
      send_feedback(0.0, "Iniciando navegação para " + dest);
      
      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

      // NOVO: começa cada navegação SEM limite de velocidade
      publish_speed_limit(0.0);
      slowed_ = false;
      armed_ = false;

      // NOVO: acompanha a distância que falta e desacelera perto do destino
      send_goal_options.feedback_callback = [this](
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
          const double restante = feedback->distance_remaining;
          // No começo o Nav2 ainda não calculou o caminho e manda 0: só "arma" depois de
          // ver uma distância maior que a zona de desaceleração.
          if (!armed_) {
            if (restante > slow_down_distance_) armed_ = true;
            return;
          }
          if (!slowed_ && restante < slow_down_distance_) {
            publish_speed_limit(slow_down_percentage_);
            slowed_ = true;
            RCLCPP_INFO(this->get_logger(), "   Perto do destino (%.2f m): velocidade limitada a %.0f%%",
                        restante, slow_down_percentage_);
          }
        };
      
      // Capturamos [this, dest] para a função lambda lembrar qual era o destino
      send_goal_options.result_callback = [this, dest](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
        goal_sent_ = false;
        publish_speed_limit(0.0);   // NOVO: tira o limite ao terminar (sucesso ou falha)
        slowed_ = false;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          // LOG: O robô chegou no alvo
          RCLCPP_INFO(this->get_logger(), "<< CHEGOU: PALIO alcançou o ponto %s com sucesso!", dest.c_str());
          finish(true, 1.0, "Destino alcançado");
        } else {  
          // LOG: Algo deu errado na navegação do Nav2
          RCLCPP_ERROR(this->get_logger(), "!! FALHA: PALIO não conseguiu chegar ao ponto %s. Nav2 abortou.", dest.c_str());
          finish(false, 1.0, "Falha na navegação");
        }
      };
      
      nav_client_->async_send_goal(goal_msg, send_goal_options);
    }
  }

private:
  // NOVO: publica o limite de velocidade. 0 = sem limite.
  void publish_speed_limit(double percentage)
  {
    nav2_msgs::msg::SpeedLimit msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.percentage = true;
    msg.speed_limit = percentage;
    speed_limit_pub_->publish(msg);
  }

  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  double slow_down_distance_ = 0.6;
  double slow_down_percentage_ = 40.0;
  bool slowed_ = false;
  bool armed_ = false;

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