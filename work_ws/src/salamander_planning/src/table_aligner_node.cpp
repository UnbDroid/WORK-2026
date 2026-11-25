#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class TableAligner : public rclcpp::Node {
public:
  TableAligner() : Node("table_aligner")
  {
    target_dist_ = declare_parameter("target_dist", 0.25);
    edge_y_target_ = declare_parameter("edge_y_target", 0.10);
    tol_ = declare_parameter("tol", 0.01);
    timeout_ = declare_parameter("timeout", 10.0);
    kp_ = declare_parameter("kp", 1.0);
    vmax_ = declare_parameter("vmax", 0.08);
    max_ex_ = declare_parameter("max_ex", 0.4);
    max_ey_ = declare_parameter("max_ey", 0.25);

    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions so;
    so.callback_group = cb_group_;

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr m) { scan_ = m; }, so);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    srv_ = create_service<std_srvs::srv::Trigger>(
        "align_to_table",
        std::bind(&TableAligner::handle, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), cb_group_);
  }

private:
  void handle(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    auto t0 = now();
    rclcpp::Rate rate(20.0);

    while (rclcpp::ok() && (now() - t0).seconds() < timeout_) {
      auto scan = scan_;
      if (!scan) { rate.sleep(); continue; }

      std::vector<double> xs, ys;
      for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double r = scan->ranges[i];
        if (!std::isfinite(r) || r <= scan->range_min || r >= scan->range_max) continue;
        double a = scan->angle_min + i * scan->angle_increment;
        double x = r * std::cos(a), y = r * std::sin(a);
        if (x > 0.05 && x < 0.6 && std::fabs(y) < 0.4) { xs.push_back(x); ys.push_back(y); }
      }

      if (xs.size() < 5) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Mesa nao detectada");
        rate.sleep(); continue;
      }

      // distância: mediana dos 10 pontos mais próximos
      std::vector<double> sx = xs;
      std::sort(sx.begin(), sx.end());
      size_t n = std::min<size_t>(10, sx.size());
      double dist = sx[n / 2];
      double edge_y = *std::max_element(ys.begin(), ys.end());

      double ex = dist - target_dist_;
      double ey = edge_y - edge_y_target_;

      if (std::fabs(ex) > max_ex_ || std::fabs(ey) > max_ey_) {
        stop();
        res->success = false;
        res->message = "Erro grande demais, deteccao suspeita";
        return;
      }

      if (std::fabs(ex) < tol_ && std::fabs(ey) < tol_) {
        stop();
        res->success = true;
        res->message = "Alinhado";
        return;
      }

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = std::clamp(kp_ * ex, -vmax_, vmax_);
      // primeiro corrige a distância, depois o lado
      cmd.linear.y = (std::fabs(ex) < 2 * tol_) ? std::clamp(kp_ * ey, -vmax_, vmax_) : 0.0;
      cmd_pub_->publish(cmd);
      rate.sleep();
    }

    stop();
    res->success = false;
    res->message = "Timeout";
  }

  void stop() { cmd_pub_->publish(geometry_msgs::msg::Twist()); }

  double target_dist_, edge_y_target_, tol_, timeout_, kp_, vmax_, max_ex_, max_ey_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TableAligner>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}