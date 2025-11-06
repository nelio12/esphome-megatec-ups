#include "powermust.h"
#include "esphome/core/log.h"

namespace esphome {
namespace powermust {

static const char *const TAG = "powermust";

void Powermust::setup() {
  this->state_ = STATE_IDLE;
  this->command_start_millis_ = 0;

  // Añadimos polling automático para Q1, F e I
  // (Q1 y F ya están por los sensores, pero I lo añadimos aquí)
  this->add_polling_command_("I", POLLING_I);
}

void Powermust::empty_uart_buffer_() {
  uint8_t byte;
  while (this->available()) {
    this->read_byte(&byte);
  }
}

void Powermust::loop() {
  // === Lectura de mensajes ===
  if (this->state_ == STATE_IDLE) {
    this->empty_uart_buffer_();
    switch (this->send_next_command_()) {
      case 0:
        // Cola vacía → polling
        if (millis() - this->last_poll_ > this->update_interval_) {
          this->send_next_poll_();
          this->last_poll_ = millis();
        }
        return;
      case 1:
        // Comando enviado
        return;
    }
  }

  // === Finalización de comando (ACK/NAK) ===
  if (this->state_ == STATE_COMMAND_COMPLETE) {
    std::string current_cmd = this->command_queue_[this->command_queue_position_];
    bool success = false;

    // Comandos que esperan ACK/NAK
    bool is_ack_command = (
        current_cmd == "T" ||
        current_cmd == "TL" ||
        current_cmd == "T10" ||
        current_cmd == "CT" ||
        current_cmd == "Q" ||
        current_cmd == "C" ||
        current_cmd == "CL" ||
        (!current_cmd.empty() && current_cmd[0] == 'S')  // S03, S02R0060, etc.
    );

    if (is_ack_command) {
      if (this->read_pos_ > 0) {
        if (this->read_buffer_[this->read_pos_ - 1] == '\r') {
          this->read_buffer_[--this->read_pos_] = '\0';
        }
        if (this->read_pos_ >= 3 && memcmp(this->read_buffer_, "NAK", 3) == 0) {
          ESP_LOGE(TAG, "Command failed: NAK for '%s'", current_cmd.c_str());
        } else {
          ESP_LOGI(TAG, "Command successful: ACK for '%s'", current_cmd.c_str());
          success = true;
        }
      } else {
        ESP_LOGE(TAG, "Command failed: no response for '%s'", current_cmd.c_str());
      }
    } else {
      if (this->read_pos_ == 0) {
        ESP_LOGI(TAG, "Command successful: no response expected");
        success = true;
      } else {
        ESP_LOGE(TAG, "Command failed: unexpected response");
      }
    }

    // === Apagar switches momentáneos ===
    if (success) {
      if (current_cmd == "T" && this->quick_test_switch_) {
        this->quick_test_switch_->publish_state(false);
      } else if (current_cmd == "TL" && this->deep_test_switch_) {
        this->deep_test_switch_->publish_state(false);
      } else if (current_cmd == "T10" && this->ten_minutes_test_switch_) {
        this->ten_minutes_test_switch_->publish_state(false);
      } else if (current_cmd == "CT") {
        if (this->quick_test_switch_) this->quick_test_switch_->publish_state(false);
        if (this->deep_test_switch_) this->deep_test_switch_->publish_state(false);
        if (this->ten_minutes_test_switch_) this->ten_minutes_test_switch_->publish_state(false);
      } else if (current_cmd[0] == 'S') {
        if (current_cmd.find("R") != std::string::npos && this->shutdown_restore_switch_) {
          this->shutdown_restore_switch_->publish_state(false);
        } else if (this->shutdown_switch_) {
          this->shutdown_switch_->publish_state(false);
        }
      } else if (current_cmd == "C" && this->cancel_shutdown_switch_) {
        this->cancel_shutdown_switch_->publish_state(false);
      }
    }

    // Limpiar cola
    this->command_queue_[this->command_queue_position_].clear();
    this->command_queue_position_ = (this->command_queue_position_ + 1) % COMMAND_QUEUE_LENGTH;
    this->state_ = STATE_IDLE;
  }

  // === Publicar valores decodificados ===
  if (this->state_ == STATE_POLL_DECODED) {
    switch (this->used_polling_commands_[this->last_polling_command_].identifier) {
      case POLLING_Q1:
        if (this->grid_voltage_) this->grid_voltage_->publish_state(value_grid_voltage_);
        if (this->grid_fault_voltage_) this->grid_fault_voltage_->publish_state(value_grid_fault_voltage_);
        if (this->ac_output_voltage_) this->ac_output_voltage_->publish_state(value_ac_output_voltage_);
        if (this->ac_output_load_percent_) this->ac_output_load_percent_->publish_state(value_ac_output_load_percent_);
        if (this->grid_frequency_) this->grid_frequency_->publish_state(value_grid_frequency_);
        if (this->battery_voltage_) this->battery_voltage_->publish_state(value_battery_voltage_);
        if (this->temperature_) this->temperature_->publish_state(value_temperature_);

        if (this->utility_fail_) this->utility_fail_->publish_state(value_utility_fail_ == 1);
        if (this->battery_low_) this->battery_low_->publish_state(value_battery_low_ == 1);
        if (this->bypass_active_) this->bypass_active_->publish_state(value_bypass_active_ == 1);
        if (this->ups_failed_) this->ups_failed_->publish_state(value_ups_failed_ == 1);
        if (this->ups_type_standby_) this->ups_type_standby_->publish_state(value_ups_type_standby_ == 1);
        if (this->test_in_progress_) this->test_in_progress_->publish_state(value_test_in_progress_ == 1);
        if (this->shutdown_active_) this->shutdown_active_->publish_state(value_shutdown_active_ == 1);
        if (this->beeper_on_) this->beeper_on_->publish_state(value_beeper_on_ == 1);
        if (this->beeper_switch_) this->beeper_switch_->publish_state(value_beeper_on_ == 1);

        if (this->quick_test_switch_) this->quick_test_switch_->publish_state(value_test_in_progress_ == 1);
        if (this->deep_test_switch_) this->deep_test_switch_->publish_state(value_test_in_progress_ == 1);
        if (this->ten_minutes_test_switch_) this->ten_minutes_test_switch_->publish_state(value_test_in_progress_ == 1);

        this->state_ = STATE_IDLE;
        break;

      case POLLING_F:
        if (this->ac_output_rating_voltage_) this->ac_output_rating_voltage_->publish_state(value_ac_output_rating_voltage_);
        if (this->ac_output_rating_current_) this->ac_output_rating_current_->publish_state(value_ac_output_rating_current_);
        if (this->battery_rating_voltage_) this->battery_rating_voltage_->publish_state(value_battery_rating_voltage_);
        if (this->ac_output_rating_frequency_) this->ac_output_rating_frequency_->publish_state(value_ac_output_rating_frequency_);
        this->state_ = STATE_IDLE;
        break;

      case POLLING_I:
        // Ya se publicó en STATE_POLL_CHECKED
        this->state_ = STATE_IDLE;
        break;
    }
  }

  // === Decodificación de respuestas ===
  if (this->state_ == STATE_POLL_CHECKED) {
    char tmp[POWERMUST_READ_BUFFER_LENGTH];
    strncpy(tmp, (const char*)this->read_buffer_, this->read_pos_);
    tmp[this->read_pos_] = '\0';

    switch (this->used_polling_commands_[this->last_polling_command_].identifier) {
      case POLLING_Q1: {
        ESP_LOGD(TAG, "Decode Q1");
        char temp_str[8] = {0};
        char status_bits[16] = {0};
        int items_assigned = sscanf(tmp, "(%f %f %f %d %f %f %7s %15s",
               &value_grid_voltage_,
               &value_grid_fault_voltage_,
               &value_ac_output_voltage_,
               &value_ac_output_load_percent_,
               &value_grid_frequency_,
               &value_battery_voltage_,
               temp_str,
               status_bits);

        if (items_assigned < 8) {
          ESP_LOGW(TAG, "sscanf Q1 falló! Asignó %d/8. Trama: '%s'", items_assigned, tmp);
          this->state_ = STATE_IDLE;
          break;
        }

        // Temperatura
        if (strcmp(temp_str, "--.-") == 0 || strcmp(temp_str, "?.?") == 0) {
          value_temperature_ = NAN;
        } else {
          value_temperature_ = strtof(temp_str, nullptr);
        }

        // Bits de estado
        size_t len = strlen(status_bits);
        value_utility_fail_ = (len >= 1 && status_bits[0] == '1') ? 1 : 0;
        value_battery_low_ = (len >= 2 && status_bits[1] == '1') ? 1 : 0;
        value_bypass_active_ = (len >= 3 && status_bits[2] == '1') ? 1 : 0;
        value_ups_failed_ = (len >= 4 && status_bits[3] == '1') ? 1 : 0;
        value_ups_type_standby_ = (len >= 5 && status_bits[4] == '1') ? 1 : 0;
        value_test_in_progress_ = (len >= 6 && status_bits[5] == '1') ? 1 : 0;
        value_shutdown_active_ = (len >= 7 && status_bits[6] == '1') ? 1 : 0;
        value_beeper_on_ = (len >= 8 && status_bits[7] == '1') ? 1 : 0;

        ESP_LOGD(TAG, "Q1 → Grid:%.1fV Out:%.1fV Load:%d%% Temp:%s Beeper:%s",
                 value_grid_voltage_, value_ac_output_voltage_, value_ac_output_load_percent_,
                 temp_str, value_beeper_on_ ? "ON" : "OFF");

        if (this->last_q1_) {
          char *cr = strchr(tmp, '\r');
          if (cr) *cr = '\0';
          this->last_q1_->publish_state(tmp);
        }

        this->state_ = STATE_POLL_DECODED;
        break;
      }

      case POLLING_F: {
        ESP_LOGD(TAG, "Decode F");
        int items = sscanf(tmp, "#%f %d %f %f",
                           &value_ac_output_rating_voltage_,
                           &value_ac_output_rating_current_,
                           &value_battery_rating_voltage_,
                           &value_ac_output_rating_frequency_);

        if (items < 4) {
          ESP_LOGW(TAG, "sscanf F falló! Asignó %d/4. Trama: '%s'", items, tmp);
        } else {
          ESP_LOGD(TAG, "F → OutV:%.1fV Cur:%dA BatV:%.2fV Freq:%.1fHz",
                   value_ac_output_rating_voltage_, value_ac_output_rating_current_,
                   value_battery_rating_voltage_, value_ac_output_rating_frequency_);
        }

        if (this->last_f_) {
          char *cr = strchr(tmp, '\r');
          if (cr) *cr = '\0';
          this->last_f_->publish_state(tmp);
        }

        this->state_ = STATE_POLL_DECODED;
        break;
      }

      case POLLING_I: {
        ESP_LOGD(TAG, "Decode I");
        char clean_tmp[POWERMUST_READ_BUFFER_LENGTH];
        strncpy(clean_tmp, tmp, sizeof(clean_tmp) - 1);
        clean_tmp[sizeof(clean_tmp) - 1] = '\0';

        char *cr = strchr(clean_tmp, '\r');
        if (cr) *cr = '\0';

        ESP_LOGD(TAG, "UPS Info: %s", clean_tmp);

        if (this->ups_info_) {
          this->ups_info_->publish_state(clean_tmp);
        }

        this->state_ = STATE_POLL_DECODED;
        break;
      }

      default:
        this->state_ = STATE_IDLE;
        break;
    }
    return;
  }

  // === Finalización de polling (NAK o fin) ===
  if (this->state_ == STATE_POLL_COMPLETE) {
    if (this->read_pos_ >= 4 && memcmp(this->read_buffer_, "(NAK", 4) == 0) {
      ESP_LOGW(TAG, "Polling command failed (NAK)");
      this->state_ = STATE_IDLE;
      return;
    }
    this->state_ = STATE_POLL_CHECKED;
    return;
  }

  // === Recepción de bytes ===
  if (this->state_ == STATE_POLL || this->state_ == STATE_COMMAND) {
    while (this->available()) {
      uint8_t byte;
      this->read_byte(&byte);

      if (this->read_pos_ >= POWERMUST_READ_BUFFER_LENGTH) {
        this->read_pos_ = 0;
        this->empty_uart_buffer_();
      }

      this->read_buffer_[this->read_pos_++] = byte;

      if (byte == 0x0D) {  // \r
        this->read_buffer_[this->read_pos_] = '\0';
        this->empty_uart_buffer_();

        if (this->state_ == STATE_POLL) {
          this->state_ = STATE_POLL_COMPLETE;
        } else if (this->state_ == STATE_COMMAND) {
          this->state_ = STATE_COMMAND_COMPLETE;
        }
      }
    }
  }

  // === Timeout de comando ===
  if (this->state_ == STATE_COMMAND) {
    if (millis() - this->command_start_millis_ > COMMAND_TIMEOUT) {
      ESP_LOGW(TAG, "Command timeout: %s", this->command_queue_[this->command_queue_position_].c_str());
      this->state_ = STATE_COMMAND_COMPLETE;
    }
  }

  // === Timeout de polling ===
  if (this->state_ == STATE_POLL) {
    if (millis() - this->command_start_millis_ > COMMAND_TIMEOUT) {
      ESP_LOGW(TAG, "Polling timeout: %s", (char*)this->used_polling_commands_[this->last_polling_command_].command);
      this->state_ = STATE_IDLE;
    }
  }
}

// === Funciones auxiliares ===
uint8_t Powermust::check_incoming_length_(uint8_t length) {
  return (this->read_pos_ - 3 == length) ? 1 : 0;
}

uint8_t Powermust::send_next_command_() {
  if (this->command_queue_[this->command_queue_position_].empty()) {
    return 0;
  }

  std::string cmd = this->command_queue_[this->command_queue_position_];
  this->state_ = STATE_COMMAND;
  this->command_start_millis_ = millis();
  this->empty_uart_buffer_();
  this->read_pos_ = 0;

  this->write_str(cmd.c_str());
  this->write(0x0D);

  ESP_LOGD(TAG, "Sending command from queue: %s with length %d", cmd.c_str(), (int)cmd.length());
  return 1;
}

void Powermust::send_next_poll_() {
  this->last_polling_command_ = (this->last_polling_command_ + 1) % 15;
  while (this->used_polling_commands_[this->last_polling_command_].length == 0) {
    this->last_polling_command_ = (this->last_polling_command_ + 1) % 15;
    if (this->last_polling_command_ == 0) return;  // No hay comandos
  }

  auto &cmd = this->used_polling_commands_[this->last_polling_command_];
  this->state_ = STATE_POLL;
  this->command_start_millis_ = millis();
  this->empty_uart_buffer_();
  this->read_pos_ = 0;

  this->write_array(cmd.command, cmd.length);
  this->write(0x0D);

  ESP_LOGD(TAG, "Sending polling command: %s (len=%d)", (char*)cmd.command, cmd.length);
}

void Powermust::queue_command_(const char *command, uint8_t length) {
  for (int i = 0; i < COMMAND_QUEUE_LENGTH; i++) {
    int pos = (this->command_queue_position_ + i) % COMMAND_QUEUE_LENGTH;
    if (this->command_queue_[pos].empty()) {
      this->command_queue_[pos] = std::string(command, length);
      ESP_LOGD(TAG, "Command queued: %s at pos %d", command, pos);
      return;
    }
  }
  ESP_LOGW(TAG, "Command queue full, dropping: %s", command);
}

void Powermust::switch_command(const std::string &command) {
  ESP_LOGD(TAG, "got command: %s", command.c_str());
  queue_command_(command.c_str(), command.length());
}

void Powermust::dump_config() {
  ESP_LOGCONFIG(TAG, "Powermust:");
  ESP_LOGCONFIG(TAG, "  Used polling commands:");
  for (auto &cmd : this->used_polling_commands_) {
    if (cmd.length > 0) {
      ESP_LOGCONFIG(TAG, "    %s", (char*)cmd.command);
    }
  }

  LOG_SENSOR("", "Grid Voltage", this->grid_voltage_);
  LOG_SENSOR("", "Grid Fault Voltage", this->grid_fault_voltage_);
  LOG_SENSOR("", "AC Output Voltage", this->ac_output_voltage_);
  LOG_SENSOR("", "AC Output Load Percent", this->ac_output_load_percent_);
  LOG_SENSOR("", "Grid Frequency", this->grid_frequency_);
  LOG_SENSOR("", "Battery Voltage", this->battery_voltage_);
  LOG_SENSOR("", "Temperature", this->temperature_);
  LOG_SENSOR("", "AC Output Rating Voltage", this->ac_output_rating_voltage_);
  LOG_SENSOR("", "AC Output Rating Current", this->ac_output_rating_current_);
  LOG_SENSOR("", "Battery Rating Voltage", this->battery_rating_voltage_);
  LOG_SENSOR("", "AC Output Rating Frequency", this->ac_output_rating_frequency_);

  LOG_BINARY_SENSOR("", "Utility Fail", this->utility_fail_);
  LOG_BINARY_SENSOR("", "Battery Low", this->battery_low_);
  LOG_BINARY_SENSOR("", "Bypass Active", this->bypass_active_);
  LOG_BINARY_SENSOR("", "UPS Failed", this->ups_failed_);
  LOG_BINARY_SENSOR("", "UPS Type Standby", this->ups_type_standby_);
  LOG_BINARY_SENSOR("", "Test In Progress", this->test_in_progress_);
  LOG_BINARY_SENSOR("", "Shutdown Active", this->shutdown_active_);
  LOG_BINARY_SENSOR("", "Beeper On", this->beeper_on_);

  LOG_TEXT_SENSOR("", "Last Q1", this->last_q1_);
  LOG_TEXT_SENSOR("", "Last F", this->last_f_);
  LOG_TEXT_SENSOR("", "UPS Information", this->ups_info_);
}

void Powermust::update() {
  // Opcional: forzar polling
}

void Powermust::add_polling_command_(const char *command, ENUMPollingCommand polling_command) {
  for (auto &used : this->used_polling_commands_) {
    if (used.length == 0) {
      size_t len = strlen(command);
      used.command = new uint8_t[len + 1];
      memcpy(used.command, command, len);
      used.command[len] = '\0';
      used.length = len;
      used.identifier = polling_command;
      used.errors = 0;
      return;
    }
    if (used.length == strlen(command) && memcmp(used.command, command, used.length) == 0) {
      return;  // Ya existe
    }
  }
}

}  // namespace powermust
}  // namespace esphome
