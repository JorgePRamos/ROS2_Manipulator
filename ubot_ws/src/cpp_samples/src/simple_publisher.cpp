#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

// Use 1s as frequency for wall_timer
using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node{
// Public class atributes
public:
    // Constructor for the SimplePublisher calling inherited Node("NodeName"), init cnt_
    SimplePublisher() : Node("simple_pub"), cnt_(0){
        // store output in var pub_
        // create_publisher<std_msgs::msg::String>(topic_name,queue_size)
        pub_ =  create_publisher<std_msgs::msg::String>("chatter",10);
        
        // Timer which takes a frequency and a function name which to be executed at set frequency
        // (frequency, function_name)
        timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));

        // Log info (loggers_code, msg)
        RCLCPP_INFO(get_logger(),"Pub at 1 Hz");
    };
    
    void timerCallback(){
        // Create msg::str var for msg (Type of variable is auto deducted)
        auto msg = std_msgs::msg::String();
        // Set data and concatenated cnt to string
        msg.data = "Hello - cnt: " + std::to_string(cnt_);
        // Use publisher object to publish
        pub_->publish(msg);


    };


// Private class atributes
private:
    unsigned int cnt_;
    // Create Publisher class new obj <type of msg>.Shared pointer to obj
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    // Create TimerBase class new obj.Shared pointer to obj
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char const *argv[])
{
    // Init ros2
    rclcpp::init(argc, argv);
    // Create ros2 node
    auto node = std::make_shared<SimplePublisher>();
    // Spin to keep active
    rclcpp::spin(node);
    // Garbage when ctr+c
    rclcpp::shutdown();
    return 0;
}
