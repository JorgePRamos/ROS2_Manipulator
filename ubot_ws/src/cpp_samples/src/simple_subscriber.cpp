#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node{
// Public class atributes
public:
    // Constructor for the SimpleSubscriber calling inherited Node("NodeName"), init cnt_
    SimpleSubscriber() : Node("simple_sub"){
        // store output in var sub_
        // create_subscription<std_msgs::msg::String>(topic_name,queue_size,call_back_func, pointer_to_current_version, call_back_takes_1_input)

        sub_ = create_subscription<std_msgs::msg::String>("chatter",10, std::bind(&SimpleSubscriber::msgCallback, this, _1));
    };
    
    void msgCallback(const std_msgs::msg::String &msg) const{
        // Log info (loggers_code, msg)
        RCLCPP_INFO(get_logger(),"I listen: %s", msg.data.c_str());
    }


// Private class atributes
private:
    // Create Publisher class new obj <type of msg>.Shared pointer to obj
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char const *argv[])
{
    // Init ros2
    rclcpp::init(argc, argv);
    // Create ros2 node
    auto node = std::make_shared<SimpleSubscriber>();
    // Spin to keep active
    rclcpp::spin(node);
    // Garbage when ctr+c
    rclcpp::shutdown();
    return 0;
}
