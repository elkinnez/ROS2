#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "taller_tortuga_cpp/srv/cambiar_modo.hpp"
#include <cmath>
#include <vector>
#include <memory>

using namespace std::chrono_literals;

class ControlTortuga : public rclcpp::Node
{
public:
    ControlTortuga() : Node("control_tortuga"), modo_actual_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            [this](turtlesim::msg::Pose::SharedPtr msg) { pose_ = *msg; });
        
        service_ = this->create_service<taller_tortuga_cpp::srv::CambiarModo>(
            "cambiar_modo",
            [this](const std::shared_ptr<taller_tortuga_cpp::srv::CambiarModo::Request> req,
                   std::shared_ptr<taller_tortuga_cpp::srv::CambiarModo::Response> res)
            {
                modo_actual_ = req->modo;
                sentido_ = req->sentido;
                punto_ = 0;
                
                if (modo_actual_ == 2) {
                    puntos_ = {{2.0, 8.0}, {8.0, 8.0}, {5.0, 5.0}};
                }
                
                RCLCPP_INFO(this->get_logger(), "Modo cambiado a: %d", modo_actual_);
                res->resultado = "OK";
                res->modo_actual = modo_actual_;
                res->exito = true;
            });
        
        timer_ = this->create_wall_timer(100ms, [this]() {
            auto msg = geometry_msgs::msg::Twist();
            
            if (modo_actual_ == 1) {
                msg.linear.x = 1.0;
                msg.angular.z = (sentido_ == "horario") ? -1.5 : 1.5;
            }
            else if (modo_actual_ == 2) {
                if (punto_ < puntos_.size()) {
                    double dx = puntos_[punto_].first - pose_.x;
                    double dy = puntos_[punto_].second - pose_.y;
                    double dist = std::sqrt(dx*dx + dy*dy);
                    
                    if (dist < 0.3) {
                        punto_++;
                        if (punto_ >= puntos_.size()) {
                            modo_actual_ = 0;
                            RCLCPP_INFO(this->get_logger(), "Trayectoria terminada - Modo Manual");
                        }
                    } else {
                        double ang = std::atan2(dy, dx) - pose_.theta;
                        if (std::abs(ang) > 0.1) {
                            msg.angular.z = (ang > 0) ? 1.0 : -1.0;
                        } else {
                            msg.linear.x = 1.0;
                        }
                    }
                }
            }
            
            publisher_->publish(msg);
        });
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::Service<taller_tortuga_cpp::srv::CambiarModo>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    turtlesim::msg::Pose pose_;
    int modo_actual_;
    std::string sentido_;
    std::vector<std::pair<double, double>> puntos_;
    size_t punto_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlTortuga>());
    rclcpp::shutdown();
    return 0;
}