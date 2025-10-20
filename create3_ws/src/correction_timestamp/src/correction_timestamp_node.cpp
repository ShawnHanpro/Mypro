#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

using std::placeholders::_1;

class RangeTimestampCorrector : public rclcpp::Node
{
public:
    RangeTimestampCorrector()
    : Node("range_timestamp_corrector")
    {
        // 声明并获取话题名称参数
        this->declare_parameter("input_topic", "point_laser");
        this->declare_parameter("output_topic", "point_laser_corrected");
        
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

        subscriber_ = this->create_subscription<sensor_msgs::msg::Range>(
            input_topic, 10, std::bind(&RangeTimestampCorrector::range_callback, this, _1));

        publisher_ = this->create_publisher<sensor_msgs::msg::Range>(output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Timestamp Corrector Initialized.");
        RCLCPP_INFO(this->get_logger(), "Listening on: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
    }

private:
    void range_callback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
        auto corrected_msg = std::make_unique<sensor_msgs::msg::Range>(*msg);

        corrected_msg->header.stamp = this->now();

        publisher_->publish(std::move(corrected_msg));

        static bool first_run = true;
        if (first_run) {
            RCLCPP_INFO(this->get_logger(), "First correction applied. All subsequent stamps will use rclcpp::Node::now().");
            first_run = false;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RangeTimestampCorrector>());
    rclcpp::shutdown();
    return 0;
}