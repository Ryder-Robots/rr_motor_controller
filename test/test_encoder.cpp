#include <gtest/gtest.h>
#include <memory>
#include <list>
#include <vector>
#include "rr_motor_controller/encoder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Mock GPIO that captures ISR function pointer for simulating interrupts
class MockEncoderGPIO : public rrobots::interfaces::RRGPIOInterface {
 public:
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

  int set_pin_mode(unsigned pin, int mode) override {
    last_set_pin_mode_pin = static_cast<int>(pin);
    last_set_pin_mode_mode = mode;
    set_pin_mode_call_count++;
    return set_pin_mode_result;
  }

  int set_pull_up_down(unsigned pin, unsigned pud) override {
    last_set_pud_pin = static_cast<int>(pin);
    last_set_pud_value = static_cast<int>(pud);
    set_pud_call_count++;
    return set_pull_up_down_result;
  }

  uint32_t tick(void) override { return tick_value; }

  int set_isr_func_ex(unsigned gpio, unsigned edge, int timeout, gpio_isr_func_ex_t func, void* userdata) override {
    last_isr_gpio = static_cast<int>(gpio);
    last_isr_edge = static_cast<int>(edge);
    last_isr_timeout = timeout;
    captured_isr_func = func;
    captured_isr_userdata = userdata;
    return set_isr_func_ex_result;
  }

  int clear_isr_func(unsigned gpio) override {
    last_clear_isr_pin = static_cast<int>(gpio);
    clear_isr_call_count++;
    return clear_isr_func_result;
  }

  int digital_write(unsigned gpio, unsigned level) override { (void)gpio; (void)level; return 0; }
  int digital_read(unsigned gpio) override { (void)gpio; return 0; }
  int gpio_hardware_pwm(unsigned pin, unsigned pwm_freq, unsigned pwm_duty_cycle) override {
    (void)pin; (void)pwm_freq; (void)pwm_duty_cycle; return 0;
  }
  int gpio_hardware_get_pwm(unsigned pin) override { (void)pin; return 0; }
  std::list<unsigned> get_pwm_pins() const override { return {}; }

  // Simulate a hardware interrupt by calling the captured ISR
  void fire_interrupt(int gpio, int level, uint32_t tick_val) {
    if (captured_isr_func && captured_isr_userdata) {
      captured_isr_func(gpio, level, tick_val, captured_isr_userdata);
    }
  }

  // Configurable return values
  int set_pin_mode_result = 0;
  int set_pull_up_down_result = 0;
  int set_isr_func_ex_result = 0;
  int clear_isr_func_result = 0;
  uint32_t tick_value = 1000;

  // Captured state
  int last_set_pin_mode_pin = -1;
  int last_set_pin_mode_mode = -1;
  int set_pin_mode_call_count = 0;

  int last_set_pud_pin = -1;
  int last_set_pud_value = -1;
  int set_pud_call_count = 0;

  int last_isr_gpio = -1;
  int last_isr_edge = -1;
  int last_isr_timeout = -1;

  int last_clear_isr_pin = -1;
  int clear_isr_call_count = 0;

  gpio_isr_func_ex_t captured_isr_func = nullptr;
  void* captured_isr_userdata = nullptr;
};

// Holds data from a single tick callback invocation
struct TickEvent {
  int gpio_pin;
  uint32_t delta_us;
  uint32_t tick;
  rr_motor_controller::TickStatus status;
};

class EncoderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    mock_gpio_ = std::make_shared<MockEncoderGPIO>();
    encoder_ = std::make_unique<rr_motor_controller::MotorEncoder>();
    tick_events_.clear();
  }

  // Helper: create a lifecycle node with encoder parameters declared
  rclcpp_lifecycle::LifecycleNode::SharedPtr make_node(const std::string& name,
                                                        int pin = 4,
                                                        int timeout = 500,
                                                        int min_interval = 100)
  {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(name);
    node->declare_parameter("encoder_pin", pin);
    node->declare_parameter("encoder_timeout", timeout);
    node->declare_parameter("encoder_min_interval_us", min_interval);
    return node;
  }

  // Tick callback that records events for assertion
  rr_motor_controller::EncoderTickCallback make_tick_cb() {
    return [this](int gpio_pin, uint32_t delta_us, uint32_t tick,
                  rr_motor_controller::TickStatus status) {
      tick_events_.push_back({gpio_pin, delta_us, tick, status});
    };
  }

  // Helper: configure + activate with defaults
  void configure_and_activate(const std::string& node_name = "test_encoder_node") {
    auto node = make_node(node_name);
    rclcpp_lifecycle::State prev_state;
    auto cfg = encoder_->configure(prev_state, node, mock_gpio_, make_tick_cb());
    ASSERT_EQ(cfg, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    auto act = encoder_->on_activate(prev_state);
    ASSERT_EQ(act, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  }

  std::shared_ptr<MockEncoderGPIO> mock_gpio_;
  std::unique_ptr<rr_motor_controller::MotorEncoder> encoder_;
  std::vector<TickEvent> tick_events_;
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ---------------------------------------------------------------------------
// configure() tests
// ---------------------------------------------------------------------------

TEST_F(EncoderTest, ConfigureSuccess) {
  auto node = make_node("test_cfg_ok");
  rclcpp_lifecycle::State prev_state;
  auto result = encoder_->configure(prev_state, node, mock_gpio_, make_tick_cb());
  EXPECT_EQ(result, CallbackReturn::SUCCESS);
}

TEST_F(EncoderTest, ConfigureFailsMissingParameters) {
  // Node without any encoder parameters declared
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_cfg_missing");
  rclcpp_lifecycle::State prev_state;
  auto result = encoder_->configure(prev_state, node, mock_gpio_, make_tick_cb());
  EXPECT_EQ(result, CallbackReturn::FAILURE);
}

TEST_F(EncoderTest, ConfigureFailsNullCallback) {
  auto node = make_node("test_cfg_null_cb");
  rclcpp_lifecycle::State prev_state;
  auto result = encoder_->configure(prev_state, node, mock_gpio_, nullptr);
  EXPECT_EQ(result, CallbackReturn::FAILURE);
}

// ---------------------------------------------------------------------------
// on_activate() tests
// ---------------------------------------------------------------------------

TEST_F(EncoderTest, OnActivateSuccess) {
  auto node = make_node("test_act_ok", 4, 500, 100);
  mock_gpio_->tick_value = 5000;

  rclcpp_lifecycle::State prev_state;
  encoder_->configure(prev_state, node, mock_gpio_, make_tick_cb());

  auto result = encoder_->on_activate(prev_state);
  EXPECT_EQ(result, CallbackReturn::SUCCESS);

  // Verify GPIO was configured correctly
  EXPECT_EQ(mock_gpio_->last_set_pin_mode_pin, 4);
  EXPECT_EQ(mock_gpio_->last_set_pin_mode_mode, rrobots::interfaces::RRGPIOInterface::PI_INPUT);
  EXPECT_EQ(mock_gpio_->last_set_pud_pin, 4);
  EXPECT_EQ(mock_gpio_->last_set_pud_value, rrobots::interfaces::RRGPIOInterface::PI_PUD_DOWN);
  EXPECT_EQ(mock_gpio_->last_isr_gpio, 4);
  EXPECT_EQ(mock_gpio_->last_isr_edge, static_cast<int>(rrobots::interfaces::RRGPIOInterface::RISING_EDGE));
  EXPECT_EQ(mock_gpio_->last_isr_timeout, 500);
  EXPECT_NE(mock_gpio_->captured_isr_func, nullptr);
}

TEST_F(EncoderTest, OnActivateFailsWithoutConfigure) {
  // tick_cb_ is null since configure was never called
  rclcpp_lifecycle::State prev_state;
  auto result = encoder_->on_activate(prev_state);
  EXPECT_EQ(result, CallbackReturn::FAILURE);
}

TEST_F(EncoderTest, OnActivateFailsSetPinMode) {
  auto node = make_node("test_act_pin_fail");
  rclcpp_lifecycle::State prev_state;
  encoder_->configure(prev_state, node, mock_gpio_, make_tick_cb());

  mock_gpio_->set_pin_mode_result = -1;
  auto result = encoder_->on_activate(prev_state);
  EXPECT_EQ(result, CallbackReturn::FAILURE);
}

TEST_F(EncoderTest, OnActivateFailsSetPullUpDown) {
  auto node = make_node("test_act_pud_fail");
  rclcpp_lifecycle::State prev_state;
  encoder_->configure(prev_state, node, mock_gpio_, make_tick_cb());

  mock_gpio_->set_pull_up_down_result = -1;
  auto result = encoder_->on_activate(prev_state);
  EXPECT_EQ(result, CallbackReturn::FAILURE);
}

TEST_F(EncoderTest, OnActivateFailsSetIsrFuncEx) {
  auto node = make_node("test_act_isr_fail");
  rclcpp_lifecycle::State prev_state;
  encoder_->configure(prev_state, node, mock_gpio_, make_tick_cb());

  mock_gpio_->set_isr_func_ex_result = -1;
  auto result = encoder_->on_activate(prev_state);
  EXPECT_EQ(result, CallbackReturn::FAILURE);
}

// ---------------------------------------------------------------------------
// on_deactivate() tests
// ---------------------------------------------------------------------------

TEST_F(EncoderTest, OnDeactivateSuccess) {
  configure_and_activate("test_deact_ok");
  rclcpp_lifecycle::State prev_state;

  auto result = encoder_->on_deactivate(prev_state);
  EXPECT_EQ(result, CallbackReturn::SUCCESS);
  EXPECT_EQ(mock_gpio_->clear_isr_call_count, 1);
  EXPECT_EQ(mock_gpio_->last_clear_isr_pin, 4);
}

TEST_F(EncoderTest, OnDeactivateFailsClearIsr) {
  configure_and_activate("test_deact_isr_fail");
  mock_gpio_->clear_isr_func_result = -1;

  rclcpp_lifecycle::State prev_state;
  auto result = encoder_->on_deactivate(prev_state);
  EXPECT_EQ(result, CallbackReturn::FAILURE);
}

TEST_F(EncoderTest, OnDeactivateFailsSetPudOff) {
  configure_and_activate("test_deact_pud_fail");
  mock_gpio_->set_pull_up_down_result = -1;

  rclcpp_lifecycle::State prev_state;
  auto result = encoder_->on_deactivate(prev_state);
  EXPECT_EQ(result, CallbackReturn::FAILURE);
}

TEST_F(EncoderTest, OnDeactivateFailsSetPinMode) {
  configure_and_activate("test_deact_pin_fail");
  mock_gpio_->set_pin_mode_result = -1;

  rclcpp_lifecycle::State prev_state;
  auto result = encoder_->on_deactivate(prev_state);
  EXPECT_EQ(result, CallbackReturn::FAILURE);
}

TEST_F(EncoderTest, OnDeactivateContinuesAfterPartialFailure) {
  configure_and_activate("test_deact_partial");
  // First call (clear_isr) fails, but set_pull_up_down and set_pin_mode should still be called
  mock_gpio_->clear_isr_func_result = -1;

  // Reset counters after activate
  mock_gpio_->set_pud_call_count = 0;
  mock_gpio_->set_pin_mode_call_count = 0;

  rclcpp_lifecycle::State prev_state;
  auto result = encoder_->on_deactivate(prev_state);
  EXPECT_EQ(result, CallbackReturn::FAILURE);
  // Verify remaining cleanup still ran
  EXPECT_EQ(mock_gpio_->set_pud_call_count, 1);
  EXPECT_EQ(mock_gpio_->set_pin_mode_call_count, 1);
}

// ---------------------------------------------------------------------------
// ISR / handle_interrupt() tests (via captured function pointer)
// ---------------------------------------------------------------------------

TEST_F(EncoderTest, InterruptHealthyRisingEdge) {
  mock_gpio_->tick_value = 1000;
  configure_and_activate("test_isr_healthy");

  // expected_level_ is RISING_EDGE (0), fire with level=0 → HEALTHY
  mock_gpio_->fire_interrupt(4, 0, 2000);

  ASSERT_EQ(tick_events_.size(), 1u);
  EXPECT_EQ(tick_events_[0].gpio_pin, 4);
  EXPECT_EQ(tick_events_[0].delta_us, 1000u);  // 2000 - 1000
  EXPECT_EQ(tick_events_[0].tick, 2000u);
  EXPECT_EQ(tick_events_[0].status, rr_motor_controller::TickStatus::HEALTHY);
}

TEST_F(EncoderTest, InterruptTimeout) {
  mock_gpio_->tick_value = 1000;
  configure_and_activate("test_isr_timeout");

  // level=2 → TIMEOUT
  mock_gpio_->fire_interrupt(4, 2, 5000);

  ASSERT_EQ(tick_events_.size(), 1u);
  EXPECT_EQ(tick_events_[0].status, rr_motor_controller::TickStatus::TIMEOUT);
  EXPECT_EQ(tick_events_[0].delta_us, 4000u);  // 5000 - 1000
}

TEST_F(EncoderTest, InterruptNoiseRejected) {
  mock_gpio_->tick_value = 1000;
  configure_and_activate("test_isr_noise");

  // level=1 (rising edge level, but expected_level_ is 0) → NOISE_REJECTED
  mock_gpio_->fire_interrupt(4, 1, 1050);

  ASSERT_EQ(tick_events_.size(), 1u);
  EXPECT_EQ(tick_events_[0].status, rr_motor_controller::TickStatus::NOISE_REJECTED);
  EXPECT_EQ(tick_events_[0].delta_us, 50u);
}

TEST_F(EncoderTest, InterruptDeltaUsAccumulatesAcrossTicks) {
  mock_gpio_->tick_value = 1000;
  configure_and_activate("test_isr_accum");

  mock_gpio_->fire_interrupt(4, 0, 2000);
  mock_gpio_->fire_interrupt(4, 0, 3500);
  mock_gpio_->fire_interrupt(4, 0, 4000);

  ASSERT_EQ(tick_events_.size(), 3u);
  EXPECT_EQ(tick_events_[0].delta_us, 1000u);  // 2000 - 1000
  EXPECT_EQ(tick_events_[1].delta_us, 1500u);  // 3500 - 2000
  EXPECT_EQ(tick_events_[2].delta_us, 500u);   // 4000 - 3500
}

TEST_F(EncoderTest, InterruptTickWraparound) {
  // Set initial tick near UINT32_MAX
  mock_gpio_->tick_value = UINT32_MAX - 500;
  configure_and_activate("test_isr_wrap");

  // Fire with a tick that has wrapped around past 0
  uint32_t wrapped_tick = 499;  // (UINT32_MAX - 500) + 1000 wraps to 499
  mock_gpio_->fire_interrupt(4, 0, wrapped_tick);

  ASSERT_EQ(tick_events_.size(), 1u);
  // Unsigned subtraction handles wraparound correctly: 499 - (UINT32_MAX - 500) = 1000
  EXPECT_EQ(tick_events_[0].delta_us, 1000u);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
