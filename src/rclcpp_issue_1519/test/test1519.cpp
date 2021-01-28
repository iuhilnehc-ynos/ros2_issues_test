#include <gtest/gtest.h>

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono;

class TestBug1519: public ::testing::Test{
protected:
  static void SetUpTestCase(){
    rclcpp::init(0, NULL);
  }

  static void TearDownTestCase(){
    rclcpp::shutdown();
  }

  void SetUp(){
  }

  void TearDown(){
  }
};

class Module
{
public:
  Module(rclcpp::Node::SharedPtr node):node_(node){};

  ~Module() = default;

  void stringCallback(std_msgs::msg::String::SharedPtr msg){
    static_cast<void>(msg);
  }
private:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TestBug1519, test_resource_deadlock) {
  auto test_function = [] () {
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<Module> module;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stringSub;
    rclcpp::Node::SharedPtr pubNode;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stringPub;

    auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
    node = std::make_shared<rclcpp::Node>("test_bug", options);
    module = std::make_shared<Module>(node);

    // a node to create subscription
    {
      auto fcn = std::bind(&Module::stringCallback, module, std::placeholders::_1);
      stringSub = node->create_subscription<std_msgs::msg::String>(
        "/string", 1, fcn);
    }

    // another node to create publisher
    pubNode = std::make_shared<rclcpp::Node>("pub_node", options);
    stringPub = pubNode->create_publisher<std_msgs::msg::String>("/string", 1);

    // if stringSub not reset, aborted happened at line 71
    // only while using `use_intra_process_comms(true)` to create node,
    // which means it's OK if using use_intra_process_comms(false).
    // stringSub.reset();

    // continue to use exist variables for new instances
    node = std::make_shared<rclcpp::Node>("test_bug_new_node", options);
    module = std::make_shared<Module>(node);
    auto fcn = std::bind(&Module::stringCallback, module, std::placeholders::_1);
    stringSub = node->create_subscription<std_msgs::msg::String>(
      "/string", 1, fcn);
    // ...
    std::cout << "end" << std::endl;
  };

  test_function();

  // exception can't be caught because abort internal
  // EXPECT_THROW(test_function(), std::system_error);
  // So It seems we can't write a test case by using EXPECT_NO_THROW()
}
