#include "motor_control.h"
#include "hardware_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include <math.h>

static const char *TAG = "MOTOR_CTRL";

// Global state variables
motor_group_t active_group = GROUP_NONE;
uint16_t active_motors = 0x0000;
uint8_t mcp_porta = 0x00;
uint8_t mcp_portb = 0x00;
bool sync_mode = false;
uint8_t expansion_relay_state = 0x00;

// External references
extern SemaphoreHandle_t hardware_mutex;
extern i2c_master_dev_handle_t mcp_handle;
extern bool extension_startup_protect;

void motor_control_init(void) {
    ESP_LOGI(TAG, "Initializing motor control system");
    
    // PWM initialization
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Configure PWM channels for each motor
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t pwm1_config = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motors[i].pwm1_channel,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = motors[i].pwm1_pin,
            .duty = 0,
            .hpoint = 0
        };
        
        ret = ledc_channel_config(&pwm1_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PWM1 channel %d config failed: %s", i, esp_err_to_name(ret));
            continue;
        }
        
        ledc_channel_config_t pwm2_config = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motors[i].pwm2_channel,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = motors[i].pwm2_pin,
            .duty = 0,
            .hpoint = 0
        };
        
        ret = ledc_channel_config(&pwm2_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PWM2 channel %d config failed: %s", i, esp_err_to_name(ret));
            continue;
        }
    }
    
    // Configure sensor pins for closed-loop motors
    for (int i = 0; i < 7; i++) {
        if (motors[i].type == MOTOR_TYPE_CLOSED_LOOP && motors[i].sensor_pin >= 0) {
            gpio_config_t io_conf = {
                .intr_type = GPIO_INTR_DISABLE,
                .mode = GPIO_MODE_INPUT,
                .pin_bit_mask = (1ULL << motors[i].sensor_pin),
                .pull_down_en = 0,
                .pull_up_en = 0,
            };
            
            ret = gpio_config(&io_conf);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "GPIO config for sensor pin %d failed: %s", 
                        motors[i].sensor_pin, esp_err_to_name(ret));
            }
        }
    }
    
    ESP_LOGI(TAG, "Motor control system initialized");
}

bool is_motor_active(int motor_id) {
    if (motor_id < 0 || motor_id >= 11) {
        ESP_LOGW(TAG, "Invalid motor ID: %d", motor_id);
        return false;
    }
    return motor_states[motor_id].is_active;
}

void emergency_stop_motor(int motor_id) {
    if (motor_id < 0 || motor_id >= 11) {
        ESP_LOGW(TAG, "Invalid motor ID for emergency stop: %d", motor_id);
        return;
    }
    
    motor_states[motor_id].speed = 0;
    motor_states[motor_id].target_speed = 0;
    motor_states[motor_id].is_moving = false;
    motor_states[motor_id].max_speed = MAX_SPEED;
    
    // Stop PWM outputs
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm1_channel, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm2_channel, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm1_channel);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm2_channel);
    
    ESP_LOGI(TAG, "Emergency stop applied to motor %d (%s)", motor_id, motors[motor_id].name);
}

void update_pcnt_direction(int motor_id, int16_t new_speed) {
    if (motor_id >= 7 || motors[motor_id].type != MOTOR_TYPE_CLOSED_LOOP) {
        return;
    }
    
    if (!motors[motor_id].pcnt_unit || !motors[motor_id].pcnt_channel) {
        return;
    }
    
    motor_state_t *state = &motor_states[motor_id];
    int16_t new_direction = (new_speed > 0) ? 1 : (new_speed < 0) ? -1 : 0;
    
    if (state->last_direction != new_direction && new_direction != 0) {
        esp_err_t ret;
        
        if (new_direction > 0) {
            ret = pcnt_channel_set_edge_action(motors[motor_id].pcnt_channel, 
                PCNT_CHANNEL_EDGE_ACTION_HOLD, 
                PCNT_CHANNEL_EDGE_ACTION_INCREASE);
            state->pcnt_inverted = false;
        } else {
            ret = pcnt_channel_set_edge_action(motors[motor_id].pcnt_channel, 
                PCNT_CHANNEL_EDGE_ACTION_HOLD,
                PCNT_CHANNEL_EDGE_ACTION_DECREASE);
            state->pcnt_inverted = true;
        }
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PCNT direction update failed for motor %d: %s", 
                    motor_id, esp_err_to_name(ret));
        } else {
            ESP_LOGD(TAG, "Motor %d PCNT direction updated: %s", 
                    motor_id, new_direction > 0 ? "INCREASE" : "DECREASE");
        }
    }
    
    state->last_direction = new_direction;
}

void update_motor_speed_smooth(int motor_id, int16_t target_speed) {
    if (!is_motor_active(motor_id)) {
        return;
    }
    
    motor_state_t *state = &motor_states[motor_id];
    int16_t current = state->speed;
    int16_t target = target_speed;
    
    // Update PCNT direction for closed-loop motors
    if (motor_id < 7 && motors[motor_id].type == MOTOR_TYPE_CLOSED_LOOP) {
        update_pcnt_direction(motor_id, target);
    }
    
    // Clamp target speed
    if (target > MAX_SPEED) target = MAX_SPEED;
    if (target < -MAX_SPEED) target = -MAX_SPEED;
    
    // Apply speed ramping
    int16_t diff = target - current;
    
    if (abs(diff) <= SPEED_RAMP_RATE) {
        state->speed = target;
    } else {
        if (diff > 0) {
            state->speed += SPEED_RAMP_RATE;
        } else {
            state->speed -= SPEED_RAMP_RATE;
        }
    }
    
    // Apply minimum speed threshold
    if (abs(state->speed) < MIN_SPEED && target == 0) {
        state->speed = 0;
    }
    
    // Set PWM duty cycle
    uint32_t duty = abs(state->speed);
    if (duty > 255) duty = 255;  // 8-bit PWM resolution
    
    esp_err_t ret1, ret2;
    
    if (state->speed > 0) {
        ret1 = ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm1_channel, duty);
        ret2 = ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm2_channel, 0);
    } else if (state->speed < 0) {
        ret1 = ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm1_channel, 0);
        ret2 = ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm2_channel, duty);
    } else {
        ret1 = ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm1_channel, 0);
        ret2 = ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm2_channel, 0);
    }
    
    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm1_channel);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm2_channel);
    } else {
        ESP_LOGE(TAG, "PWM update failed for motor %d", motor_id);
    }
}

void set_motor_speed(int motor_id, int16_t speed) {
    if (!is_motor_active(motor_id)) {
        if (speed != 0) {
            ESP_LOGW(TAG, "Motor %d (%s) not active", motor_id, motors[motor_id].name);
        }
        return;
    }
    
    // Update PCNT direction for closed-loop motors
    if (motor_id < 7 && motors[motor_id].type == MOTOR_TYPE_CLOSED_LOOP) {
        update_pcnt_direction(motor_id, speed);
    }
    
    motor_states[motor_id].speed = speed;
    motor_states[motor_id].target_speed = speed;
    
    // Clamp speed
    uint32_t duty = abs(speed);
    if (duty > 250) duty = 250;
    
    esp_err_t ret1, ret2;
    
    if (speed > 0) {
        ret1 = ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm1_channel, duty);
        ret2 = ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm2_channel, 0);
    } else if (speed < 0) {
        ret1 = ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm1_channel, 0);
        ret2 = ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm2_channel, duty);
    } else {
        ret1 = ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm1_channel, 0);
        ret2 = ledc_set_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm2_channel, 0);
    }
    
    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm1_channel);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motors[motor_id].pwm2_channel);
    } else {
        ESP_LOGE(TAG, "PWM update failed for motor %d", motor_id);
    }
}

void set_expansion_relay(uint8_t relay_mask) {
    expansion_relay_state = relay_mask;
    
    // Update MCP portB bits 0-2 for expansion relays
    if (relay_mask & 0x01) mcp_portb |= 0x01;
    else mcp_portb &= ~0x01;
    
    if (relay_mask & 0x02) mcp_portb |= 0x02;
    else mcp_portb &= ~0x02;
    
    if (relay_mask & 0x04) mcp_portb |= 0x04;
    else mcp_portb &= ~0x04;
    
    esp_err_t ret = mcp23017_write(MCP23017_OLATB, mcp_portb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Expansion relay update failed: %s", esp_err_to_name(ret));
    }
    
    vTaskDelay(pdMS_TO_TICKS(50));
}

void select_motor_group(motor_group_t group) {
    if (hardware_mutex == NULL) {
        ESP_LOGE(TAG, "Hardware mutex not initialized");
        return;
    }
    
    if (xSemaphoreTake(hardware_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take hardware mutex");
        return;
    }
    
    ESP_LOGI(TAG, "Selecting motor group %d", group);
    
    // Stop all motors and clear movement flags
    for (int i = 0; i < 11; i++) {
        set_motor_speed(i, 0);
        motor_states[i].is_active = false;
        motor_states[i].is_moving = false;
        motor_states[i].target_speed = 0;
        motor_states[i].max_speed = MAX_SPEED;
        motor_states[i].last_direction = 0;
        motor_states[i].pcnt_inverted = false;
    }
    
    sync_state.enabled = false;
    sync_state.is_moving = false;
    
    // Disable H-bridges
    mcp_portb &= ~0x80;
    mcp23017_write(MCP23017_OLATB, mcp_portb);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Disable power
    mcp_porta &= ~0x10;
    mcp23017_write(MCP23017_OLATA, mcp_porta);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Clear relay selection
    mcp_porta &= ~0x07;
    mcp23017_write(MCP23017_OLATA, mcp_porta);
    set_expansion_relay(0x00);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Clean up old PCNT units
    for (int i = 0; i < 7; i++) {
        if (motors[i].pcnt_unit) {
            pcnt_unit_stop(motors[i].pcnt_unit);
            pcnt_unit_disable(motors[i].pcnt_unit);
            if (motors[i].pcnt_channel) {
                pcnt_del_channel(motors[i].pcnt_channel);
                motors[i].pcnt_channel = NULL;
            }
            pcnt_del_unit(motors[i].pcnt_unit);
            motors[i].pcnt_unit = NULL;
        }
    }
    
    // Set new selection
    active_group = group;
    active_motors = 0x0000;
    
    switch (group) {
        case GROUP_LEGS:
            mcp_porta |= 0x04;  // A2
            active_motors = 0x000F;  // Motors 0-3
            for (int i = 0; i <= 3; i++) {
                motor_states[i].is_active = true;
            }
            break;
            
        case GROUP_ELEVATION_PANELS:
            mcp_porta |= 0x02;  // A1
            active_motors = 0x07F0;  // Motors 4-10
            motor_states[4].is_active = true;
            motor_states[5].is_active = true;
            motor_states[7].is_active = true;
            motor_states[8].is_active = true;
            motor_states[9].is_active = true;
            motor_states[10].is_active = true;
            set_expansion_relay(0x07);
            break;
            
        case GROUP_SLEW:
            mcp_porta |= 0x01;  // A0
            active_motors = 0x0040;  // Motor 6
            motor_states[6].is_active = true;
            break;
            
        case GROUP_NONE:
        default:
            // No relays set, no motors active
            break;
    }
    
    if (group != GROUP_NONE) {
        // Apply relay settings
        esp_err_t ret = mcp23017_write(MCP23017_OLATA, mcp_porta);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Relay selection failed: %s", esp_err_to_name(ret));
            xSemaphoreGive(hardware_mutex);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Enable power
        mcp_porta |= 0x10;
        ret = mcp23017_write(MCP23017_OLATA, mcp_porta);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Power enable failed: %s", esp_err_to_name(ret));
            xSemaphoreGive(hardware_mutex);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Enable H-bridges
        mcp_portb |= 0x80;
        ret = mcp23017_write(MCP23017_OLATB, mcp_portb);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "H-bridge enable failed: %s", esp_err_to_name(ret));
        }
  
        // Setup PCNT for closed-loop motors
        for (int i = 0; i < 7; i++) {
            if (motor_states[i].is_active && motors[i].type == MOTOR_TYPE_CLOSED_LOOP) {
                pcnt_unit_config_t unit_config = {
                    .high_limit = INT16_MAX,
                    .low_limit = INT16_MIN,
                };
                
                ret = pcnt_new_unit(&unit_config, &motors[i].pcnt_unit);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "PCNT unit creation failed for motor %d: %s", i, esp_err_to_name(ret));
                    continue;
                }
                
                pcnt_glitch_filter_config_t filter_config = {
                    .max_glitch_ns = 10000,
                };
                pcnt_unit_set_glitch_filter(motors[i].pcnt_unit, &filter_config);
                
                pcnt_chan_config_t chan_config = {
                    .edge_gpio_num = motors[i].sensor_pin,
                    .level_gpio_num = -1,
                };
                
                ret = pcnt_new_channel(motors[i].pcnt_unit, &chan_config, &motors[i].pcnt_channel);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "PCNT channel creation failed for motor %d: %s", i, esp_err_to_name(ret));
                    continue;
                }
                
                pcnt_channel_set_edge_action(motors[i].pcnt_channel, 
                    PCNT_CHANNEL_EDGE_ACTION_INCREASE, 
                    PCNT_CHANNEL_EDGE_ACTION_HOLD);
                
                pcnt_unit_enable(motors[i].pcnt_unit);
                pcnt_unit_start(motors[i].pcnt_unit);
                pcnt_unit_clear_count(motors[i].pcnt_unit);
                
                motor_states[i].pulse_count = 0;
                motor_states[i].pcnt_initialized = true;
                motor_states[i].max_speed = MAX_SPEED;
                motor_states[i].last_direction = 0;
                motor_states[i].pcnt_inverted = false;
                
                ESP_LOGD(TAG, "Motor %d: PCNT initialized - offset=%ld, position=%.1f", 
                        i, motor_states[i].pulse_offset, motor_states[i].position);
            }
        }
    }
    
    xSemaphoreGive(hardware_mutex);
    
    const char* group_name = (group == GROUP_LEGS) ? "Legs" : 
                           (group == GROUP_ELEVATION_PANELS) ? "Elevation/Panels" : 
                           (group == GROUP_SLEW) ? "Slew" : "None";
    
    ESP_LOGI(TAG, "Selected group %d (%s), active motors: 0x%04X", 
            group, group_name, active_motors);
}

void update_positions(void) {
    for (int i = 0; i < 7; i++) {
        if (motors[i].pcnt_unit && motors[i].type == MOTOR_TYPE_CLOSED_LOOP && motor_states[i].is_active) {
            int raw_count;
            esp_err_t ret = pcnt_unit_get_count(motors[i].pcnt_unit, &raw_count);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "PCNT read failed for motor %d: %s", i, esp_err_to_name(ret));
                continue;
            }
            
            int32_t adjusted_count = raw_count + motor_states[i].pulse_offset;
            int32_t delta = adjusted_count - motor_states[i].pulse_count;
            
            // Handle counter overflow/underflow
            if (delta > 30000) {
                delta = delta - 65536;
                ESP_LOGD(TAG, "Motor %d: Underflow detected, adjusted delta: %ld", i, delta);
            } else if (delta < -30000) {
                delta = delta + 65536;
                ESP_LOGD(TAG, "Motor %d: Overflow detected, adjusted delta: %ld", i, delta);
            }
            
            motor_states[i].pulse_count = adjusted_count;
            
            if (delta != 0) {
                float pos_change = (float)delta / motor_states[i].pulses_per_unit;
                motor_states[i].position += pos_change;
            }
        }
    }
}

void calculate_sync_speeds(void) {
    if (!sync_state.enabled || sync_state.motor_mask == 0) {
        return;
    }
    
    float sum_pos = 0;
    float max_pos = -999999;
    float min_pos = 999999;
    int count = 0;
    
    // Calculate average position and error
    for (int i = 0; i < 7; i++) {
        if ((sync_state.motor_mask & (1 << i)) && motor_states[i].is_active) {
            float pos = motor_states[i].position;
            sum_pos += pos;
            if (pos > max_pos) max_pos = pos;
            if (pos < min_pos) min_pos = pos;
            count++;
        }
    }
    
    if (count == 0) {
        return;
    }
    
    sync_state.avg_position = sum_pos / count;
    sync_state.max_error = max_pos - min_pos;
    
    // Check if movement is complete
    if (sync_state.max_error <= sync_state.tolerance && 
        fabs(sync_state.avg_position - sync_state.target_position) <= sync_state.tolerance) {
        sync_state.is_moving = false;
        for (int i = 0; i < 7; i++) {
            if (sync_state.motor_mask & (1 << i)) {
                motor_states[i].target_speed = 0;
                motor_states[i].is_moving = false;
            }
        }
        return;
    }
    
    // Calculate synchronized speeds
    for (int i = 0; i < 7; i++) {
        if ((sync_state.motor_mask & (1 << i)) && motor_states[i].is_active) {
            float pos_error = sync_state.target_position - motor_states[i].position;
            float sync_error = sync_state.avg_position - motor_states[i].position;
            
            float distance = fabs(pos_error);
            float base_speed = motor_states[i].max_speed;
            
            // Apply speed ramping for approach
            if (distance < APPROACH_DISTANCE) {
                base_speed = motor_states[i].max_speed * (distance / APPROACH_DISTANCE);
                if (base_speed < MIN_SPEED) base_speed = MIN_SPEED;
            }
            
            if (distance < FINE_APPROACH_DISTANCE) {
                base_speed = MIN_SPEED + (distance / FINE_APPROACH_DISTANCE) * (motor_states[i].max_speed/4 - MIN_SPEED);
            }
            
            // Apply synchronization correction
            float sync_correction = sync_error * 10.0f;
            if (sync_correction > 50) sync_correction = 50;
            if (sync_correction < -50) sync_correction = -50;
            
            int16_t speed = (int16_t)base_speed;
            if (pos_error < 0) speed = -speed;
            
            speed += (int16_t)sync_correction;
            
            // Enforce minimum speed
            if ((pos_error > 0 && speed < MIN_SPEED) || (pos_error < 0 && speed > -MIN_SPEED)) {
                speed = (pos_error > 0) ? MIN_SPEED : -MIN_SPEED;
            }
            
            motor_states[i].target_speed = speed;
            motor_states[i].is_moving = true;
            
            // Debug output for key motors
            if (i == 0 || (i == 4 && active_group == GROUP_ELEVATION_PANELS)) {
                ESP_LOGD(TAG, "Sync %s: pos=%.2f, target=%.2f, avg=%.2f, speed=%d, sync_err=%.2f", 
                        motors[i].name, motor_states[i].position, sync_state.target_position,
                        sync_state.avg_position, speed, sync_error);
            }
        }
    }
}

void position_control_task(void *arg) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // External references for tracking
    extern bool auto_tracking;
    extern bool time_synced;
    extern bool boot_flag;
    extern float sun_el;
    extern TaskHandle_t auto_tracking_handle;
    
    ESP_LOGI(TAG, "Position control task started");
    
    while (1) {
        // Check for auto-deployment on boot
        if (sun_el > 10.0f && time_synced && auto_tracking && boot_flag) {
            ESP_LOGI(TAG, "Main Deploy task");
            if (auto_tracking_handle != NULL) {
                xTaskNotifyGive(auto_tracking_handle);
            }
            boot_flag = false;
        }

        // Update position readings
        update_positions();
        
        // Skip motor control if no group selected or startup protection active
        if (active_group == GROUP_NONE || extension_startup_protect) {
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(50));
            continue;
        }
        
        // Handle synchronized movement
        if (sync_state.enabled) {
            calculate_sync_speeds();
        }
        
        // Process individual motor movements
        for (int i = 0; i < 7; i++) {
            if (motors[i].type == MOTOR_TYPE_CLOSED_LOOP) {
                if (!is_motor_active(i)) {
                    if (motor_states[i].is_moving) {
                        motor_states[i].is_moving = false;
                        motor_states[i].target_speed = 0;
                        set_motor_speed(i, 0);
                        ESP_LOGW(TAG, "Motor %d (%s) movement cancelled - not in active group", 
                                i, motors[i].name);
                    }
                    continue;
                }
                
                // Handle individual position control (non-synchronized)
                if (!sync_state.enabled && motor_states[i].is_moving && is_motor_active(i)) {
                    float error = motor_states[i].target_position - motor_states[i].position;
                    float distance = fabs(error);
                    float tolerance = (i < 4) ? SYNC_TOLERANCE_LEG : SYNC_TOLERANCE_ELEV;
                    
                    if (distance < tolerance) {
                        motor_states[i].target_speed = 0;
                        motor_states[i].is_moving = false;
                        ESP_LOGI(TAG, "Motor %d (%s) reached target: %.1f", 
                                i, motors[i].name, motor_states[i].position);
                    } else {
                        int16_t speed = motor_states[i].max_speed;
                        
                        // Apply approach speed reduction
                        if (distance < APPROACH_DISTANCE) {
                            speed = (int16_t)(motor_states[i].max_speed * (distance / APPROACH_DISTANCE));
                            if (speed < MIN_SPEED) speed = MIN_SPEED;
                        }
                        
                        motor_states[i].target_speed = (error > 0) ? speed : -speed;
                    }
                }
                
                // Apply smooth speed control
                update_motor_speed_smooth(i, motor_states[i].target_speed);
            }
        }
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(50));
    }
}