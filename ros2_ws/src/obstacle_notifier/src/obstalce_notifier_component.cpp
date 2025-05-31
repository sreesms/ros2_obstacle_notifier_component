#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace obstacle_notifier
{

    class ObstacleNotifierComponent : public rclcpp::Node
    {
        public:
            ObstacleNotifierComponent(const rclcpp::NodeOptions & options): 
            Node("obstacle_notifier", options)

            {
                
                declare_parameter("threshold_distance",0.5);
                get_parameter("threshold_distance",threshold_distance_);

                scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,
                    std::bind(&ObstacleNotifierComponent::scan_callback,this,std::placeholders::_1));
                
                RCLCPP_INFO(this->get_logger(),"ObstacleNotifier Component started");
            }

        private:
            void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
            {
                for(const auto & range :msg->ranges){
                    if(range > msg->range_min && range < threshold_distance_){
                        RCLCPP_WARN(this-> get_logger(),"obstacle detected within %.2f meters",range);
                        return;
                    
                    }
                }
            }
        double threshold_distance_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_notifier::ObstacleNotifierComponent)
