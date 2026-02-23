#include <gtest/gtest.h>
#include <memory>
#include <list>
#include <vector>
#include "rr_motor_controller/motor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"

// Simple mock implementation of RRGPIOInterface (without gmock)
class MockRRGPIOInterface : public rrobots::interfaces::RRGPIOInterface {
 public:
  // Lifecycle callbacks
  CallbackReturn configure(const rclcpp_lifecycle::State& state,
                           rclcpp_lifecycle::LifecycleNode::SharedPtr node) override
  {
    (void)state;
    (void)node;
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override { (void)state; return CallbackReturn::SUCCESS; }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override { (void)state; return CallbackReturn::SUCCESS; }
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override { (void)state; return CallbackReturn::SUCCESS; }

  std::map<std::string, ValueType> hardware_report() const override { return {}; }
  int initialise() override { return 0; }
  int terminate() override { return 0; }

  // GPIO operations (signatures must match interface)
  int set_pin_mode(unsigned pin, int mode) override {
    last_pin_mode_pin = static_cast<int>(pin);
    last_pin_mode = mode;
    if (first_pin_mode_pin == -1) {
      first_pin_mode_pin = static_cast<int>(pin);
    }
    return set_pin_mode_result;
  }

  int set_pull_up_down(unsigned pin, unsigned pud) override { (void)pin; (void)pud; return 0; }

  uint32_t tick(void) override { return 0; }

  int set_isr_func_ex(unsigned gpio, unsigned edge, int timeout, gpio_isr_func_ex_t func, void* userdata) override {
    (void)gpio; (void)edge; (void)timeout; (void)func; (void)userdata; return 0; }

  int clear_isr_func(unsigned gpio) override { (void)gpio; return 0; }

  int digital_write(unsigned gpio, unsigned level) override {
    last_digital_write_pin = static_cast<int>(gpio);
    last_digital_write_value = static_cast<int>(level);
    return digital_write_result;
  }

  int digital_read(unsigned gpio) override {
    last_digital_read_pin = static_cast<int>(gpio);
    return digital_read_result;
  }

  int gpio_hardware_pwm(unsigned pin, unsigned pwm_freq, unsigned pwm_duty_cycle) override {
    last_pwm_pin = static_cast<int>(pin);
    last_pwm_freq = static_cast<int>(pwm_freq);
    last_pwm_duty = static_cast<int>(pwm_duty_cycle);
    return gpio_hardware_pwm_result;
  }

  int gpio_hardware_get_pwm(unsigned pin) override {
    last_get_pwm_pin = static_cast<int>(pin);
    return gpio_hardware_get_pwm_result;
  }

  std::list<unsigned> get_pwm_pins() const override {
    return pwm_pins_list;
  }

  // Mock state for testing
  int set_pin_mode_result = 0;
  int digital_write_result = 0;
  int gpio_hardware_pwm_result = 0;
  int gpio_hardware_get_pwm_result = 0;
  int digital_read_result = 0;

  int last_pin_mode_pin = -1;
  int last_pin_mode = -1;
  int first_pin_mode_pin = -1;
  int last_digital_write_pin = -1;
  int last_digital_write_value = -1;
  int last_digital_read_pin = -1;
  int last_pwm_pin = -1;
  int last_pwm_freq = -1;
  int last_pwm_duty = -1;
  int last_get_pwm_pin = -1;

  std::list<unsigned> pwm_pins_list{12, 13, 18, 19};
};

class MotorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    mock_gpio_ = std::make_shared<MockRRGPIOInterface>();
    motor_ = std::make_unique<rr_motor_controller::Motor>();
  }

  std::shared_ptr<MockRRGPIOInterface> mock_gpio_;
  std::unique_ptr<rr_motor_controller::Motor> motor_;
};

// Test configure with valid inputs
TEST_F(MotorTest, ConfigureSuccess) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_motor_node");
  node->declare_parameter("pwm_pins", std::vector<int64_t>{12});
  node->declare_parameter("dir_pins", std::vector<int64_t>{17});
  node->declare_parameter("pwm_freq", 700);

  rclcpp_lifecycle::State previous_state;
  auto result = motor_->configure(previous_state, node, mock_gpio_, 0);
  EXPECT_EQ(result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_F(MotorTest, ConfigureFailsWithNullNode) {
  rclcpp_lifecycle::State previous_state;
  auto result = motor_->configure(previous_state, nullptr, mock_gpio_, 0);
  EXPECT_EQ(result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE);
}

TEST_F(MotorTest, ConfigureFailsWithNullGPIOPlugin) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_motor_node");
  rclcpp_lifecycle::State previous_state;
  auto result = motor_->configure(previous_state, node, nullptr, 0);
  EXPECT_EQ(result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE);
}

TEST_F(MotorTest, ConfigureFailsWithInvalidPWMPin) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_motor_node");
  node->declare_parameter("pwm_pins", std::vector<int64_t>{99});
  node->declare_parameter("dir_pins", std::vector<int64_t>{17});
  node->declare_parameter("pwm_freq", 700);

  // ensure mock pwm list does not contain 99
  mock_gpio_->pwm_pins_list = {12, 13, 18, 19};

  rclcpp_lifecycle::State previous_state;
  auto result = motor_->configure(previous_state, node, mock_gpio_, 0);
  EXPECT_EQ(result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE);
}

TEST_F(MotorTest, ConfigureFailsWithDuplicatePins) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_motor_node");
  node->declare_parameter("pwm_pins", std::vector<int64_t>{12});
  node->declare_parameter("dir_pins", std::vector<int64_t>{12});
  node->declare_parameter("pwm_freq", 700);

  mock_gpio_->pwm_pins_list = {12, 13, 18, 19};

  rclcpp_lifecycle::State previous_state;
  auto result = motor_->configure(previous_state, node, mock_gpio_, 0);
  EXPECT_EQ(result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE);
}

TEST_F(MotorTest, OnActivateSuccess) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_motor_node");
  node->declare_parameter("pwm_pins", std::vector<int64_t>{12});
  node->declare_parameter("dir_pins", std::vector<int64_t>{17});
  node->declare_parameter("pwm_freq", 700);

  mock_gpio_->pwm_pins_list = {12, 13, 18, 19};
  mock_gpio_->set_pin_mode_result = 0;
  mock_gpio_->digital_write_result = 0;
  mock_gpio_->gpio_hardware_pwm_result = 0;

  rclcpp_lifecycle::State prev_state;
  auto config_result = motor_->configure(prev_state, node, mock_gpio_, 0);
  EXPECT_EQ(config_result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto activate_result = motor_->on_activate(prev_state);
  EXPECT_EQ(activate_result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  EXPECT_EQ(mock_gpio_->first_pin_mode_pin, 17);
  EXPECT_EQ(mock_gpio_->last_digital_write_pin, 17);
  EXPECT_EQ(mock_gpio_->last_pwm_pin, 12);
}

TEST_F(MotorTest, OnActivateFailsOnSetDirPinMode) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_motor_node");
  node->declare_parameter("pwm_pins", std::vector<int64_t>{12});
  node->declare_parameter("dir_pins", std::vector<int64_t>{17});
  node->declare_parameter("pwm_freq", 700);

  mock_gpio_->pwm_pins_list = {12, 13, 18, 19};
  mock_gpio_->set_pin_mode_result = -1; // fail

  rclcpp_lifecycle::State prev_state;
  motor_->configure(prev_state, node, mock_gpio_, 0);

  auto activate_result = motor_->on_activate(prev_state);
  EXPECT_EQ(activate_result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE);
}

TEST_F(MotorTest, OnDeactivateSuccess) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_motor_node");
  node->declare_parameter("pwm_pins", std::vector<int64_t>{12});
  node->declare_parameter("dir_pins", std::vector<int64_t>{17});
  node->declare_parameter("pwm_freq", 700);

  mock_gpio_->pwm_pins_list = {12, 13, 18, 19};
  mock_gpio_->gpio_hardware_pwm_result = 0;
  mock_gpio_->digital_write_result = 0;

  rclcpp_lifecycle::State prev_state;
  motor_->configure(prev_state, node, mock_gpio_, 0);

  auto deactivate_result = motor_->on_deactivate(prev_state);
  EXPECT_EQ(deactivate_result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_F(MotorTest, SetDirection) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_motor_node");
  node->declare_parameter("pwm_pins", std::vector<int64_t>{12});
  node->declare_parameter("dir_pins", std::vector<int64_t>{17});
  node->declare_parameter("pwm_freq", 700);

  mock_gpio_->pwm_pins_list = {12, 13, 18, 19};
  mock_gpio_->digital_write_result = 0;

  rclcpp_lifecycle::State prev_state;
  motor_->configure(prev_state, node, mock_gpio_, 0);

  auto result = motor_->set_direction(rr_motor_controller::Motor::FORWARD);
  EXPECT_EQ(result, 0);
  EXPECT_EQ(mock_gpio_->last_digital_write_pin, 17);
  EXPECT_EQ(mock_gpio_->last_digital_write_value, rr_motor_controller::Motor::FORWARD);
}

TEST_F(MotorTest, SetPWM) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_motor_node");
  node->declare_parameter("pwm_pins", std::vector<int64_t>{12});
  node->declare_parameter("dir_pins", std::vector<int64_t>{17});
  node->declare_parameter("pwm_freq", 700);

  mock_gpio_->pwm_pins_list = {12, 13, 18, 19};
  mock_gpio_->gpio_hardware_pwm_result = 0;

  rclcpp_lifecycle::State prev_state;
  motor_->configure(prev_state, node, mock_gpio_, 0);

  auto result = motor_->set_pwm(50);
  EXPECT_EQ(result, 0);
  EXPECT_EQ(mock_gpio_->last_pwm_pin, 12);
  EXPECT_EQ(mock_gpio_->last_pwm_freq, 700);
  EXPECT_EQ(mock_gpio_->last_pwm_duty, 50 * rr_motor_controller::Motor::DUTY_OFFSET);
}

// Getting failures for this test on occassion.
TEST_F(MotorTest, GetPWM) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_motor_node");
  node->declare_parameter("pwm_pins", std::vector<int64_t>{12});
  node->declare_parameter("dir_pins", std::vector<int64_t>{17});
  node->declare_parameter("pwm_freq", 700);

  mock_gpio_->pwm_pins_list = {12, 13, 18, 19};
  mock_gpio_->gpio_hardware_get_pwm_result = 50 * rr_motor_controller::Motor::DUTY_OFFSET;

  rclcpp_lifecycle::State prev_state;
  motor_->configure(prev_state, node, mock_gpio_, 0);

  auto result = motor_->get_pwm();
  EXPECT_EQ(result, 50);
  EXPECT_EQ(mock_gpio_->last_get_pwm_pin, 12);
}

TEST_F(MotorTest, GetDirection) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_motor_node");
  node->declare_parameter("pwm_pins", std::vector<int64_t>{12});
  node->declare_parameter("dir_pins", std::vector<int64_t>{17});
  node->declare_parameter("pwm_freq", 700);

  mock_gpio_->pwm_pins_list = {12, 13, 18, 19};
  mock_gpio_->digital_read_result = rr_motor_controller::Motor::FORWARD;

  rclcpp_lifecycle::State prev_state;
  motor_->configure(prev_state, node, mock_gpio_, 0);

  auto result = motor_->get_direction();
  EXPECT_EQ(result, rr_motor_controller::Motor::FORWARD);
  EXPECT_EQ(mock_gpio_->last_digital_read_pin, 17);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
