#include "esphome/core/log.h"
#include "toshiba_uart.h"
#include <vector>

namespace esphome {
namespace toshiba_uart {

// Command queue with acknowledgment-based flow control
// Commands are sent, then we wait for ack (80:A1) before sending next
const uint32_t COMMAND_TIMEOUT_MS = 3000;     // Timeout waiting for ack
const uint32_t MIN_COMMAND_GAP_MS = 300;      // Minimum gap between commands
const int MAX_QUEUED_COMMANDS = 10;
const int MAX_COMMAND_LENGTH = 16;

struct QueuedCommand {
  uint8_t data[MAX_COMMAND_LENGTH];
  int length;
  bool valid;
};

QueuedCommand command_queue[MAX_QUEUED_COMMANDS];
int queue_head = 0;
int queue_tail = 0;
uint32_t last_command_time = 0;
bool command_pending = false;       // True when waiting for acknowledgment
uint32_t command_sent_time = 0;     // When the pending command was sent

std::vector< int > sensor_arr;
int current_sensor = 0;
int sensor_count = 0;

int current_slow_sensor = 0;
int slow_sensor_count = 0;

int update_slow_sensors_counter = 0;

static const char *TAG = "toshiba";

int i = 1;
int msg_len = 0;
uint8_t prev_c = 0;
uint8_t c_now = 0;
uint8_t msg [20];
uint8_t msg_start [3];

uint8_t msg_from;
uint8_t msg_to;

bool status_msg = false;

int zone1_target_temp_prev = 0;
int zone1_target_temp = 0;

int zone1_water_temp_prev = 0;
int zone1_water_temp = 0;

int hotwater_temp_prev = 0;
int hotwater_temp = 0;

int wanted_zone1_target_temp = 0;
int wanted_hotwater_water_temp = 0;

int some_temp_1_prev = 0;
int some_temp_1 = 0;
int some_temp_2_prev = 0;
int some_temp_2 = 0;
int some_temp_3_prev = 0;
int some_temp_3 = 0;

bool pump_state_known = false;
bool auto_mode_active;

bool zone1_pumpin;
bool heat_resistor_heating;
bool zone1_active;
bool zone1_active_prev;

bool hotwater_pumpin;
bool hotwater_resistor_heating;
bool hotwater_active;
bool hotwater_active_prev;

bool cooling_mode;
bool heating_mode;

// Mode change tracking - prevents UI flicker while waiting for mode to change
bool mode_change_pending = false;
bool mode_change_target = false;      // Target mode: true=cooling, false=heating
uint32_t mode_change_sent_time = 0;
uint8_t mode_change_retry_count = 0;
const uint32_t MODE_CHANGE_RETRY_MS = 3000;     // Retry command after 3s if not confirmed
const uint32_t MODE_CHANGE_MIN_INTERVAL_MS = 5000;  // Minimum 5s between mode changes
const uint8_t MODE_CHANGE_MAX_RETRIES = 3;

bool query_data = false;

void ToshibaUART::setup() {
  if (this->header_ambient_temp_sensor_){
    sensor_arr.push_back(0);
    sensor_count++;
  }
  if (this->condensed_temp_sensor_){
    sensor_arr.push_back(1);
    sensor_count++;
  }
  if (this->inlet_temp_sensor_){
    sensor_arr.push_back(2);
    sensor_count++;
  }
  if (this->outlet_temp_sensor_){
    sensor_arr.push_back(3);
    sensor_count++;
  }
  if (this->heater_outlet_temp_sensor_){
    sensor_arr.push_back(4);
    sensor_count++;
  }
  if (this->hw_cylinder_temp_sensor_){
    sensor_arr.push_back(5);
    sensor_count++;
  }
  if (this->mixing_valve_sensor_){
    sensor_arr.push_back(6);
    sensor_count++;
  }
  if (this->low_pressure_sensor_){
    sensor_arr.push_back(7);
    sensor_count++;
  }
  if (this->heat_exchange_temp_sensor_){
    sensor_arr.push_back(8);
    sensor_count++;
  }
  if (this->outside_air_temp_sensor_){
    sensor_arr.push_back(9);
    sensor_count++;
  }
  if (this->discharge_temp_sensor_){
    sensor_arr.push_back(10);
    sensor_count++;
  }
  if (this->suction_temp_sensor_){
    sensor_arr.push_back(11);
    sensor_count++;
  }
  if (this->heat_sink_temp_sensor_){
    sensor_arr.push_back(12);
    sensor_count++;
  }
  if (this->oudoor_unit_current_sensor_){
    sensor_arr.push_back(13);
    sensor_count++;
  }
  if (this->heat_exchanger_oil_temp_sensor_){
    sensor_arr.push_back(14);
    sensor_count++;
  }
  if (this->compressor_freq_sensor_){
    sensor_arr.push_back(15);
    sensor_count++;
  }
  if (this->fan_rpm_sensor_){
    sensor_arr.push_back(16);
    sensor_count++;
  }
  if (this->outdoor_pmv_sensor_){
    sensor_arr.push_back(17);
    sensor_count++;
  }
  if (this->discharge_pressure_sensor_){
    sensor_arr.push_back(18);
    sensor_count++;
  }
  if (this->micro_computer_energized_uptime_sensor_){
    sensor_arr.push_back(19);
    slow_sensor_count++;
  }
  if (this->hw_compressor_uptime_sensor_){
    sensor_arr.push_back(20);
    slow_sensor_count++;
  }
  if (this->cooling_compressor_uptime_sensor_){
    sensor_arr.push_back(21);
    slow_sensor_count++;
  }
  if (this->heating_compressor_uptime_sensor_){
    sensor_arr.push_back(22);
    slow_sensor_count++;
  }
  if (this->ac_pump_uptime_sensor_){
    sensor_arr.push_back(23);
    slow_sensor_count++;
  }
  if (this->hw_cylinder_heater_operation_uptime_sensor_){
    sensor_arr.push_back(24);
    slow_sensor_count++;
  }
  if (this->backup_heater_operation_uptime_sensor_){
    sensor_arr.push_back(25);
    slow_sensor_count++;
  }
  if (this->booster_heater_operation_uptime_sensor_){
    sensor_arr.push_back(26);
    slow_sensor_count++;
  }
}



void ToshibaUART::set_zone1_state(bool state) {
  if (pump_state_known && zone1_active != state){
    if(state){
      ESP_LOGI(TAG,"Queueing ZONE1 ON");
      queue_command(INST_ZONE1_ON, sizeof(INST_ZONE1_ON));
    }
    else{
      ESP_LOGI(TAG,"Queueing ZONE1 OFF");
      queue_command(INST_ZONE1_OFF, sizeof(INST_ZONE1_OFF));
    }
    pump_state_known = false;
  }
  else if ( zone1_active == state ){
    this->zone1_switch_switch_->publish_state(zone1_active);
  }
}

void ToshibaUART::set_hotwater_state(bool state) {
  ESP_LOGD(TAG,"Hotwater switch called: state=%d, pump_state_known=%d, hotwater_active=%d", state, pump_state_known, hotwater_active);
  if (pump_state_known && hotwater_active != state){
    if(state){
      ESP_LOGI(TAG,"Queueing HOTWATER ON");
      queue_command(INST_HOTWATER_ON, sizeof(INST_HOTWATER_ON));
    }
    else{
      ESP_LOGI(TAG,"Queueing HOTWATER OFF");
      queue_command(INST_HOTWATER_OFF, sizeof(INST_HOTWATER_OFF));
    }
    pump_state_known = false;
  }
  else if ( hotwater_active == state ){
    ESP_LOGD(TAG,"Hotwater already in requested state, republishing");
    this->hotwater_switch_switch_->publish_state(hotwater_active);
  }
  else {
    ESP_LOGW(TAG,"Cannot change hotwater state - pump_state_known=%d", pump_state_known);
  }

}


void ToshibaUART::update() {
  current_sensor ++;
  if ( update_slow_sensors_counter == 10 ){    
    if (current_sensor == slow_sensor_count + sensor_count){
      current_sensor = 0;
      update_slow_sensors_counter = 0;
    }
  } 
  else {
    update_slow_sensors_counter++;
    if (current_sensor == sensor_count){
      current_sensor = 0;
    }
  }
  request_data(CODE_REQUEST_DATA[sensor_arr[current_sensor]]);
  query_data = true;
  ESP_LOGD(TAG,"Requesting data");

}

void ToshibaUART::set_zone1_target_temp(float value) {
  if (zone1_active) {
    // Temperature encoding works for both heating and cooling: (temp + 16) * 2
    uint8_t temp_target_value = (value + 16) * 2;

    // Set the mode byte: 0x01 for cooling, 0x02 for heating
    INST_SET_ZONE1_TEMP[8] = cooling_mode ? 0x01 : 0x02;
    INST_SET_ZONE1_TEMP[9] = temp_target_value;
    INST_SET_ZONE1_TEMP[11] = (hotwater_temp + 16) * 2;
    INST_SET_ZONE1_TEMP[12] = temp_target_value;
    INST_SET_ZONE1_TEMP[13] = return_checksum(INST_SET_ZONE1_TEMP,sizeof(INST_SET_ZONE1_TEMP)); // second last, checksum

    ESP_LOGI(TAG,"Queueing Zone1 temp to %.1f°C (mode byte: 0x%02X, temp byte: 0x%02X)",
             value, INST_SET_ZONE1_TEMP[8], temp_target_value);

    queue_command(INST_SET_ZONE1_TEMP, sizeof(INST_SET_ZONE1_TEMP));

    // Store for retry mechanism
    last_zone1_temp_command_time = millis();
    last_zone1_temp_command_value = value;
    zone1_temp_command_pending = true;

    pump_state_known = false;
    wanted_zone1_target_temp = 0;
  }
  else{
    wanted_zone1_target_temp = value;
  }

}

void ToshibaUART::set_hotwater_target_temp(float value) {
  if (hotwater_active) {
    INST_SET_HOTWATER_TEMP[11] = (value + 16) * 2;
    INST_SET_HOTWATER_TEMP[13] = return_checksum(INST_SET_HOTWATER_TEMP,sizeof(INST_SET_HOTWATER_TEMP)); // second last, checksum

    ESP_LOGI(TAG,"Queueing Hotwater temp to %.1f°C", value);
    queue_command(INST_SET_HOTWATER_TEMP, sizeof(INST_SET_HOTWATER_TEMP));
    pump_state_known = false;
    wanted_hotwater_water_temp = 0;
  }
  else{ // Handle if hotwater was not active, the pump would go berserk if values were written without it running
    wanted_hotwater_water_temp = value;
  }
}

uint8_t ToshibaUART::return_checksum(uint8_t msg[], int len) {
  uint8_t Checksum = 0;
  for (int i = 2; i < (len-2); i++ ){
    Checksum = Checksum + (msg[i] % 265);
  }
  return Checksum;
}

// Queue a command for ack-based sending
void ToshibaUART::queue_command(const uint8_t* cmd, int len) {
  uint32_t now = millis();
  bool queue_empty = (queue_head == queue_tail);

  // Allow immediate send if: queue empty AND not waiting for ack AND enough gap
  bool first_command = (last_command_time == 0);
  bool gap_ok = (now - last_command_time >= MIN_COMMAND_GAP_MS);

  if (queue_empty && !command_pending && (first_command || gap_ok)) {
    // Send immediately
    ESP_LOGI(TAG, "Sending command immediately");
    this->write_array(cmd, len);
    this->flush();
    last_command_time = now;
    command_pending = true;
    command_sent_time = now;
    return;
  }

  // Add to queue
  int next_tail = (queue_tail + 1) % MAX_QUEUED_COMMANDS;
  if (next_tail == queue_head) {
    ESP_LOGW(TAG, "Command queue full, dropping command");
    return;
  }

  if (len > MAX_COMMAND_LENGTH) {
    ESP_LOGW(TAG, "Command too long (%d bytes), dropping", len);
    return;
  }

  memcpy(command_queue[queue_tail].data, cmd, len);
  command_queue[queue_tail].length = len;
  command_queue[queue_tail].valid = true;
  queue_tail = next_tail;
  ESP_LOGD(TAG, "Command queued (queue size: %d)", (queue_tail - queue_head + MAX_QUEUED_COMMANDS) % MAX_QUEUED_COMMANDS);
}

// Process queued commands with ack-based flow control
void ToshibaUART::process_command_queue() {
  uint32_t now = millis();

  // Check for timeout on pending command
  if (command_pending && (now - command_sent_time >= COMMAND_TIMEOUT_MS)) {
    ESP_LOGW(TAG, "Command timeout after %dms, no ack received - proceeding", COMMAND_TIMEOUT_MS);
    command_pending = false;
  }

  // Don't send if still waiting for ack
  if (command_pending) {
    return;
  }

  // Check if queue is empty
  if (queue_head == queue_tail) {
    return;
  }

  // Ensure minimum gap between commands
  if (now - last_command_time < MIN_COMMAND_GAP_MS) {
    return;
  }

  // Send next command
  if (command_queue[queue_head].valid) {
    ESP_LOGI(TAG, "Sending queued command");
    this->write_array(command_queue[queue_head].data, command_queue[queue_head].length);
    this->flush();
    last_command_time = now;
    command_pending = true;
    command_sent_time = now;
    command_queue[queue_head].valid = false;
  }

  queue_head = (queue_head + 1) % MAX_QUEUED_COMMANDS;
}

void ToshibaUART::set_cooling_mode(bool state) {
  uint32_t now = millis();
  ESP_LOGI(TAG,"Cooling mode switch called: requested=%d, current cooling_mode=%d, mode_change_pending=%d",
           state, cooling_mode, mode_change_pending);

  // If already changing to this mode, ignore duplicate request
  if (mode_change_pending && mode_change_target == state) {
    ESP_LOGI(TAG,"Mode change already pending for this state, ignoring");
    this->cooling_mode_switch_switch_->publish_state(state);  // Keep UI showing target
    return;
  }

  // Enforce minimum interval between mode changes
  if (mode_change_sent_time > 0 && (now - mode_change_sent_time < MODE_CHANGE_MIN_INTERVAL_MS)) {
    ESP_LOGW(TAG,"Mode change too soon (wait %dms), ignoring",
             MODE_CHANGE_MIN_INTERVAL_MS - (now - mode_change_sent_time));
    // Republish current target or actual state
    this->cooling_mode_switch_switch_->publish_state(mode_change_pending ? mode_change_target : cooling_mode);
    return;
  }

  // Check if we actually need to change
  if (cooling_mode == state && !mode_change_pending) {
    ESP_LOGI(TAG,"Cooling mode already in requested state (%d)", cooling_mode);
    this->cooling_mode_switch_switch_->publish_state(cooling_mode);
    return;
  }

  // Send the mode change command
  if(state){
    ESP_LOGI(TAG,"Sending COOLING MODE command");
    queue_command(INST_COOLING_MODE_ON, sizeof(INST_COOLING_MODE_ON));
  }
  else{
    ESP_LOGI(TAG,"Sending HEATING MODE command");
    queue_command(INST_HEATING_MODE_ON, sizeof(INST_HEATING_MODE_ON));
  }

  // Track the pending mode change
  mode_change_pending = true;
  mode_change_target = state;
  mode_change_sent_time = now;
  mode_change_retry_count = 0;

  // Immediately show the target state in UI (don't let status messages override)
  this->cooling_mode_switch_switch_->publish_state(state);
}

void ToshibaUART::set_auto_mode(bool state) {
  if (pump_state_known && auto_mode_active != state){
    if(state){
      ESP_LOGI(TAG,"Queueing AUTO MODE ON");
      queue_command(INST_AUTO_ON, sizeof(INST_AUTO_ON));
    }
    else{
      ESP_LOGI(TAG,"Queueing AUTO MODE OFF");
      queue_command(INST_AUTO_OFF, sizeof(INST_AUTO_OFF));
    }
    pump_state_known = false;
  }
  else if ( auto_mode_active == state ){
    this->auto_mode_switch_switch_->publish_state(auto_mode_active);
  }
}


void ToshibaUART::request_data(uint8_t request_code) {
  INST_REQUEST_DATA[9] = request_code;
  INST_REQUEST_DATA[10] = return_checksum(INST_REQUEST_DATA,sizeof(INST_REQUEST_DATA));
  this->write_array(INST_REQUEST_DATA,sizeof(INST_REQUEST_DATA));
  this->flush();
  //ESP_LOGD(TAG,"Request = 0x%02x, current sensor = %d, chcksum = 0x%02x", INST_REQUEST_DATA[7],sensor_arr[current_sensor],temp);
}


void ToshibaUART::publish_sensor(int sensor_index, int16_t value) {
  float temp;
  switch (sensor_index)  {
    case 9:
      if (this->outside_air_temp_sensor_){
        this->outside_air_temp_sensor_->publish_state(value);
      }
      break;

    case 13:
      if (this->oudoor_unit_current_sensor_){
        temp = value / 10;
        this->oudoor_unit_current_sensor_->publish_state(temp);
      }
      break;
    case 22:
      if (this->heating_compressor_uptime_sensor_){
        this->heating_compressor_uptime_sensor_->publish_state(value * 100);
      }
      break;
    default:
      ESP_LOGD(TAG,"current sensor = %d not found",current_sensor);
        
}
}

void ToshibaUART::publish_states() {
#ifdef USE_SWITCH
  if (this->zone1_switch_switch_) {
    this->zone1_switch_switch_->publish_state(zone1_active);
  }
  if (this->hotwater_switch_switch_) {
    this->hotwater_switch_switch_->publish_state(hotwater_active);
  }
  if (this->auto_mode_switch_switch_) {
    this->auto_mode_switch_switch_->publish_state(auto_mode_active);
  }
  if (this->cooling_mode_switch_switch_) {
    // While mode change is pending, show target state not actual state
    if (mode_change_pending) {
      this->cooling_mode_switch_switch_->publish_state(mode_change_target);
    } else {
      this->cooling_mode_switch_switch_->publish_state(cooling_mode);
    }
  }
#endif

#ifdef USE_BINARY_SENSOR
  if (this->pump_onoff_binary_sensor_) {
    this->pump_onoff_binary_sensor_->publish_state(hotwater_active | zone1_active);
  }
  if (this->heat_pump_heating_binary_sensor_) {
    this->heat_pump_heating_binary_sensor_->publish_state(zone1_pumpin);
  }
  if (this->heat_resistor_heating_binary_sensor_) {
    this->heat_resistor_heating_binary_sensor_->publish_state(heat_resistor_heating);
  }
  if (this->heat_pump_onoff_binary_sensor_) {
    this->heat_pump_onoff_binary_sensor_->publish_state(zone1_active);
  }
  if (this->hotwater_pump_heating_binary_sensor_) {
    this->hotwater_pump_heating_binary_sensor_->publish_state(hotwater_pumpin);
  }
  if (this->hotwater_resistor_heating_binary_sensor_) {
    this->hotwater_resistor_heating_binary_sensor_->publish_state(hotwater_resistor_heating);
  }
  if (this->hotwater_pump_onoff_binary_sensor_) {
    this->hotwater_pump_onoff_binary_sensor_->publish_state(hotwater_active);
  }
#endif

#ifdef USE_SENSOR
  if (this->zone1_water_temp_sensor_ && (zone1_water_temp != zone1_water_temp_prev)) {
    this->zone1_water_temp_sensor_->publish_state(zone1_water_temp);
  }
  // if (this->hotwater_target_temp_sensor_ && (hotwater_temp != hotwater_temp_prev)) {
  //   this->hotwater_target_temp_sensor_->publish_state(hotwater_temp);
  // }
#endif

#ifdef USE_NUMBER
  if ((this->zone1_target_temp_number_ && zone1_target_temp != zone1_target_temp_prev)) {
    this->zone1_target_temp_number_->publish_state(zone1_target_temp);
  }
  if ((this->hotwater_target_temp_number_ && hotwater_temp != hotwater_temp_prev)) {
    this->hotwater_target_temp_number_->publish_state(hotwater_temp);
  }
#endif
}

void ToshibaUART::loop() {
  // Process any queued commands (ack-based with 3s timeout fallback)
  process_command_queue();

  status_msg = false;
  // Need at least 3 bytes for the header (F0:F0:length) before attempting to read
  // At 2400 baud, each byte takes ~4ms, so partial messages can arrive between loop iterations
  while (available() >= 3) {
    read_array(msg_start, 3);

    // Validate header
    if (msg_start[0] != 0xF0 || msg_start[1] != 0xF0) {
      // Not a valid header - try to resync by looking for F0:F0
      // Skip this byte and continue (we already consumed 3 bytes though)
      ESP_LOGD(TAG, "Invalid header: %02X:%02X:%02X, resyncing", msg_start[0], msg_start[1], msg_start[2]);
      continue;
    }

    msg_len = msg_start[2];

    // Validate length
    int remaining = msg_len - 3;
    if (remaining <= 0 || remaining > 17) {
      ESP_LOGW(TAG, "Invalid message length: %d", msg_len);
      continue;
    }

    // Check if we have enough bytes for the payload before reading
    if (available() < remaining) {
      // Not enough data yet - at 2400 baud this can happen
      // Unfortunately we've already consumed the header, so we need to wait
      // Use a short blocking wait since the data should be arriving
      uint32_t wait_start = millis();
      while (available() < remaining && (millis() - wait_start) < 50) {
        delay(1);  // Wait up to 50ms for rest of message
      }
      if (available() < remaining) {
        ESP_LOGW(TAG, "Timeout waiting for payload: have %d, need %d", available(), remaining);
        continue;
      }
    }

    read_array(msg, remaining);
    msg_from = msg[0];
    msg_to = msg[1];
    if ((msg_from == MSG_ID_PUMP) && (msg[4] == 0x31) ) { // This is status message from pump
      zone1_active              = msg[5] & (1);
      hotwater_active_prev      = hotwater_active;
      hotwater_active           = msg[5] & (1 << 1);
      auto_mode_active          = msg[6] & (1 << 2);
      heat_resistor_heating     = msg[7] & (1 << 0);
      zone1_pumpin              = msg[7] & (1 << 1);
      hotwater_resistor_heating = msg[7] & (1 << 2);
      hotwater_pumpin           = msg[7] & (1 << 3);
      zone1_active_prev         = zone1_active;
      zone1_active              = msg[5] & (1);

      cooling_mode              = msg[5] & (1 << 5);
      heating_mode              = msg[5] & (1 << 6);

      hotwater_temp_prev = hotwater_temp;
      hotwater_temp = ((msg[8])/2-16);

      zone1_target_temp_prev = zone1_target_temp;
      zone1_target_temp = ((msg[9])/2-16);

      if (msg_len > 18){ // only in longer messages
        zone1_water_temp_prev = zone1_water_temp;
        zone1_water_temp = ((msg[13])/2-16);
      }

      pump_state_known          = true;

      // Check if pending mode change has been confirmed
      if (mode_change_pending) {
        if (cooling_mode == mode_change_target) {
          ESP_LOGI(TAG, "Mode change confirmed: cooling_mode=%d", cooling_mode);
          mode_change_pending = false;
        }
      }

      publish_states();
    } else if ((msg[3] == 0x80) && (msg[4] == 0xA1)) {
      // Command acknowledgment received (80:A1)
      if (command_pending) {
        uint32_t response_time = millis() - command_sent_time;
        ESP_LOGI(TAG, "Command acknowledged in %dms", response_time);
        command_pending = false;
      }
    } else if ((msg[3] == 0x80) && (msg[4] == 0x5C)) {
      publish_sensor(sensor_arr[current_sensor],encode_uint16(msg[5],msg[6]));
      ESP_LOGD(TAG,"current sensor = %d value = %d",current_sensor,encode_uint16(msg[5],msg[6]));
      // If we're receiving valid sensor responses, communication is working
      if (!pump_state_known) {
        ESP_LOGI(TAG,"Sensor data received - enabling command sending");
        pump_state_known = true;
      }
    }
  }
  if (!zone1_active_prev && zone1_active && wanted_zone1_target_temp){
    zone1_active_prev = true;
    set_zone1_target_temp(wanted_zone1_target_temp);
  }
  if (!hotwater_active_prev && hotwater_active && wanted_hotwater_water_temp){
    hotwater_active_prev = true;
    set_hotwater_target_temp(wanted_hotwater_water_temp);
  }
  
  // Retry temperature command if pump hasn't acknowledged within 3 seconds
  if (zone1_temp_command_pending && (millis() - last_zone1_temp_command_time > 3000)) {
    if (zone1_target_temp != last_zone1_temp_command_value) {
      ESP_LOGW(TAG,"Zone1 temp command not confirmed by pump after 3s, retrying (wanted: %.1f°C, current: %.1f°C)",
               last_zone1_temp_command_value, zone1_target_temp);
      // Resend the command
      uint8_t temp_target_value = (last_zone1_temp_command_value + 16) * 2;
      INST_SET_ZONE1_TEMP[8] = cooling_mode ? 0x01 : 0x02;
      INST_SET_ZONE1_TEMP[9] = temp_target_value;
      INST_SET_ZONE1_TEMP[11] = (hotwater_temp + 16) * 2;
      INST_SET_ZONE1_TEMP[12] = temp_target_value;
      INST_SET_ZONE1_TEMP[13] = return_checksum(INST_SET_ZONE1_TEMP,sizeof(INST_SET_ZONE1_TEMP));
      this->write_array(INST_SET_ZONE1_TEMP,sizeof(INST_SET_ZONE1_TEMP));
      this->flush();
      last_zone1_temp_command_time = millis();
    } else {
      // Temperature confirmed, clear pending flag
      zone1_temp_command_pending = false;
    }
  }

  // Retry mode change if not confirmed after 3 seconds
  if (mode_change_pending && (millis() - mode_change_sent_time >= MODE_CHANGE_RETRY_MS)) {
    if (cooling_mode != mode_change_target) {
      mode_change_retry_count++;
      if (mode_change_retry_count > MODE_CHANGE_MAX_RETRIES) {
        ESP_LOGE(TAG, "Mode change failed after %d retries, giving up", MODE_CHANGE_MAX_RETRIES);
        mode_change_pending = false;
        // Update switch to show actual state
        if (this->cooling_mode_switch_switch_) {
          this->cooling_mode_switch_switch_->publish_state(cooling_mode);
        }
      } else {
        ESP_LOGW(TAG, "Mode change not confirmed after 3s, retry %d/%d (target=%d, actual=%d)",
                 mode_change_retry_count, MODE_CHANGE_MAX_RETRIES, mode_change_target, cooling_mode);
        // Resend the mode command
        if (mode_change_target) {
          queue_command(INST_COOLING_MODE_ON, sizeof(INST_COOLING_MODE_ON));
        } else {
          queue_command(INST_HEATING_MODE_ON, sizeof(INST_HEATING_MODE_ON));
        }
        mode_change_sent_time = millis();
      }
    } else {
      // Mode confirmed
      ESP_LOGI(TAG, "Mode change confirmed on retry check");
      mode_change_pending = false;
    }
  }

}

void ToshibaUART::dump_config(){
    ESP_LOGCONFIG(TAG, "Toshiba UART component");
    this->check_uart_settings(2400,1,esphome::uart::UART_CONFIG_PARITY_NONE,8);
}

}  // namespace toshiba_uart
}  // namespace esphome