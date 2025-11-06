#pragma once
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

namespace esphome {
namespace powermust {

enum ENUMPollingCommand {
  POLLING_Q1 = 0,
  POLLING_F = 1,
  POLLING_I = 2,  // ← Comando I: UPS Information
};

struct PollingCommand {
  uint8_t *command;
  uint8_t length = 0;
  uint8_t errors;
  ENUMPollingCommand identifier;
};

#define POWERMUST_VALUED_ENTITY_(type, name, polling_command, value_type) \
 protected: \
  value_type value_##name##_; \
  POWERMUST_ENTITY_(type, name, polling_command)

#define POWERMUST_ENTITY_(type, name, polling_command) \
 protected: \
  type *name##_{}; /* NOLINT */ \
\
 public: \
  void set_##name(type *name) { /* NOLINT */ \
    this->name##_ = name; \
    this->add_polling_command_(#polling_command, POLLING_##polling_command); \
  }

#define POWERMUST_SENSOR(name, polling_command, value_type) \
  POWERMUST_VALUED_ENTITY_(sensor::Sensor, name, polling_command, value_type)

#define POWERMUST_SWITCH(name, polling_command) POWERMUST_ENTITY_(switch_::Switch, name, polling_command)

#define POWERMUST_BINARY_SENSOR(name, polling_command, value_type) \
  POWERMUST_VALUED_ENTITY_(binary_sensor::BinarySensor, name, polling_command, value_type)

#define POWERMUST_VALUED_TEXT_SENSOR(name, polling_command, value_type) \
  POWERMUST_VALUED_ENTITY_(text_sensor::TextSensor, name, polling_command, value_type)

#define POWERMUST_TEXT_SENSOR(name, polling_command) POWERMUST_ENTITY_(text_sensor::TextSensor, name, polling_command)

class Powermust : public uart::UARTDevice, public PollingComponent {
  // ------------------- Q1 -------------------
  POWERMUST_SENSOR(grid_voltage, Q1, float)
  POWERMUST_SENSOR(grid_fault_voltage, Q1, float)
  POWERMUST_SENSOR(ac_output_voltage, Q1, float)
  POWERMUST_SENSOR(ac_output_load_percent, Q1, int)
  POWERMUST_SENSOR(grid_frequency, Q1, float)
  POWERMUST_SENSOR(battery_voltage, Q1, float)
  POWERMUST_SENSOR(temperature, Q1, float)

  POWERMUST_BINARY_SENSOR(utility_fail, Q1, int)
  POWERMUST_BINARY_SENSOR(battery_low, Q1, int)
  POWERMUST_BINARY_SENSOR(bypass_active, Q1, int)
  POWERMUST_BINARY_SENSOR(ups_failed, Q1, int)
  POWERMUST_BINARY_SENSOR(ups_type_standby, Q1, int)
  POWERMUST_BINARY_SENSOR(test_in_progress, Q1, int)
  POWERMUST_BINARY_SENSOR(shutdown_active, Q1, int)
  POWERMUST_BINARY_SENSOR(beeper_on, Q1, int)

  POWERMUST_SWITCH(beeper_switch, Q1)
  POWERMUST_SWITCH(quick_test_switch, Q1)
  POWERMUST_SWITCH(deep_test_switch, Q1)
  POWERMUST_SWITCH(ten_minutes_test_switch, Q1)

  // ------------------- F -------------------
  POWERMUST_SENSOR(ac_output_rating_voltage, F, float)
  POWERMUST_SENSOR(ac_output_rating_current, F, int)
  POWERMUST_SENSOR(battery_rating_voltage, F, float)
  POWERMUST_SENSOR(ac_output_rating_frequency, F, float)

  POWERMUST_TEXT_SENSOR(last_q1, Q1)
  POWERMUST_TEXT_SENSOR(last_f, F)

  // ------------------- I: UPS Information -------------------
  POWERMUST_TEXT_SENSOR(ups_info, I)  // ← Comando I: #MUST 800VA 12V 50Hz 1.0
  void set_ups_info(text_sensor::TextSensor *s) { ups_info_ = s; }

  // ------------------- SHUTDOWN SWITCHES -------------------
  void set_shutdown_switch(switch_::Switch *s) { shutdown_switch_ = s; }
  void set_shutdown_restore_switch(switch_::Switch *s) { shutdown_restore_switch_ = s; }
  void set_cancel_shutdown_switch(switch_::Switch *s) { cancel_shutdown_switch_ = s; }

  // -----------------------------------------------------------------
  void switch_command(const std::string &command);
  void setup() override;
  void loop() override;
  void dump_config() override;
  void update() override;

 protected:
  // -----------------------------------------------------------------
  static const size_t POWERMUST_READ_BUFFER_LENGTH = 110;
  static const size_t COMMAND_QUEUE_LENGTH = 10;
  static const size_t COMMAND_TIMEOUT = 1000;

  uint32_t last_poll_ = 0;

  void add_polling_command_(const char *command, ENUMPollingCommand polling_command);
  void empty_uart_buffer_();
  uint8_t check_incoming_crc_();
  uint8_t check_incoming_length_(uint8_t length);
  uint8_t send_next_command_();
  void send_next_poll_();
  void queue_command_(const char *command, uint8_t length);

  std::string command_queue_[COMMAND_QUEUE_LENGTH];
  uint8_t command_queue_position_ = 0;

  uint8_t read_buffer_[POWERMUST_READ_BUFFER_LENGTH];
  size_t read_pos_{0};
  uint32_t command_start_millis_ = 0;

  uint8_t state_;
  enum State {
    STATE_IDLE = 0,
    STATE_POLL = 1,
    STATE_COMMAND = 2,
    STATE_POLL_COMPLETE = 3,
    STATE_COMMAND_COMPLETE = 4,
    STATE_POLL_CHECKED = 5,
    STATE_POLL_DECODED = 6,
  };

  uint8_t last_polling_command_ = 0;
  PollingCommand used_polling_commands_[15];

  // ------------------- VARIABLES DE LOS SWITCHES -------------------
  switch_::Switch *beeper_switch_{nullptr};
  switch_::Switch *quick_test_switch_{nullptr};
  switch_::Switch *deep_test_switch_{nullptr};
  switch_::Switch *ten_minutes_test_switch_{nullptr};

  switch_::Switch *shutdown_switch_{nullptr};
  switch_::Switch *shutdown_restore_switch_{nullptr};
  switch_::Switch *cancel_shutdown_switch_{nullptr};
  text_sensor::TextSensor *ups_info_{nullptr};
};

}  // namespace powermust
}  // namespace esphome
