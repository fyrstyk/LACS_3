#include "sun_tracking.h"
#include "motor_control.h"
#include "hardware_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <time.h>
#include <sys/time.h>

static const char *TAG = "SUN_TRACK";

// Global state variables
bool auto_tracking = false;
bool time_synced = false;
bool tracking_in_progress = false;
bool deploy_in_progress = false;
bool stow_in_progress = false;
bool stow_flag = true;
bool extensions_deployed = false;
bool extension_startup_protect = false;
bool fixed_elevation_flag = false;
bool auto_dep_flag = true;
bool deploy_done_flag = false;
bool boot_flag = true;
bool Deploy_All_API_Enable_Flag = true;
float sun_az, sun_el;
float latitude = 52.77f;
float longitude = -0.38f;
float home_bearing = 0.0f;
TaskHandle_t auto_tracking_handle = NULL;

void calculate_sun_position(float *azimuth, float *elevation) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    // Julian day calculation
    int year = timeinfo.tm_year + 1900;
    int month = timeinfo.tm_mon + 1;
    int day = timeinfo.tm_mday;
    
    if (month <= 2) {
        year--;
        month += 12;
    }
    
    int a = year / 100;
    int b = 2 - a + (a / 4);
    double jd = floor(365.25 * (year + 4716)) + floor(30.6001 * (month + 1)) + day + b - 1524.5;
    double hour_decimal = timeinfo.tm_hour + timeinfo.tm_min / 60.0 + timeinfo.tm_sec / 3600.0;
    jd += hour_decimal / 24.0;
    
    // Solar calculations (simplified NOAA algorithm)
    double jc = (jd - 2451545.0) / 36525.0;
    double l0 = fmod(280.46646 + jc * (36000.76983 + jc * 0.0003032), 360.0);
    double m = fmod(357.51011 + jc * (35999.05010 - 0.0001537 * jc), 360.0);
    double m_rad = m * M_PI / 180.0;
    
    double c = sin(m_rad) * (1.914602 - jc * (0.004817 + 0.000014 * jc)) +
               sin(2 * m_rad) * (0.019993 - 0.000101 * jc) +
               sin(3 * m_rad) * 0.000289;
    
    double sun_true_long = l0 + c;
    double epsilon = 23.439 - 0.00000036 * jc;
    double epsilon_rad = epsilon * M_PI / 180.0;
    double lambda_rad = sun_true_long * M_PI / 180.0;
    
    double declin = asin(sin(epsilon_rad) * sin(lambda_rad)) * 180.0 / M_PI;
    
    // Hour angle
    double time_decimal = timeinfo.tm_hour + timeinfo.tm_min / 60.0 + timeinfo.tm_sec / 3600.0;
    double solar_time = time_decimal + (4.0 * longitude) / 60.0;
    double hour_angle = 15.0 * (solar_time - 12.0);
    
    // Convert to horizontal coordinates
    double lat_rad = latitude * M_PI / 180.0;
    double declin_rad = declin * M_PI / 180.0;
    double hour_angle_rad = hour_angle * M_PI / 180.0;
    
    double zenith = acos(sin(lat_rad) * sin(declin_rad) + 
                        cos(lat_rad) * cos(declin_rad) * cos(hour_angle_rad));
    
    *elevation = 90.0f - zenith * 180.0 / M_PI;
    
    double azimuth_rad = atan2(sin(hour_angle_rad), 
                              cos(hour_angle_rad) * sin(lat_rad) - tan(declin_rad) * cos(lat_rad));
    *azimuth = fmod(azimuth_rad * 180.0 / M_PI + 180.0, 360.0);
}

void deploy_task(void *arg) {
    ESP_LOGI(TAG, "Deploy task triggered - Deploy_All_API_Enable_Flag: %s", 
             Deploy_All_API_Enable_Flag ? "ENABLED" : "DISABLED");
    
    if (Deploy_All_API_Enable_Flag) {
        ESP_LOGI(TAG, "Deploy All mode - Checking time sync and synchronized movement");
        
        if (time_synced) {
            time_t now = time(NULL);
            struct tm timeinfo;
            localtime_r(&now, &timeinfo);
            
            if (timeinfo.tm_year > (2020 - 1900)) {
                char time_str[64];
                strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);
                ESP_LOGI(TAG, "✓ Time is synced: %s", time_str);
            } else {
                ESP_LOGW(TAG, "⚠ Time appears invalid but proceeding with deployment");
            }
        } else {
            ESP_LOGW(TAG, "⚠ Time not synced, but proceeding with deployment");
        }
        
        if (!sync_mode) {
            sync_mode = true;
            ESP_LOGI(TAG, "✓ Synchronized movement mode enabled");
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "✓ Deploy All setup complete - proceeding with deployment sequence");
    }
    
    uint32_t extension_start_time = 0;
    const uint32_t EXTENSION_TIME_MS = 180000;  // 3 minutes in milliseconds
    
    while (tracking_in_progress) {
        ESP_LOGI(TAG, "Waiting for tracking to complete before deploying");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }   
    
    deploy_in_progress = true;
    ESP_LOGI(TAG, "Starting deployment sequence [%s]", 
             Deploy_All_API_Enable_Flag ? "DEPLOY ALL MODE" : "STANDARD MODE");
    
    // Calculate sun position
    float sun_az_local, sun_el_local;
    calculate_sun_position(&sun_az_local, &sun_el_local);
    sun_az = sun_az_local;
    sun_el = sun_el_local;
    ESP_LOGI(TAG, "Sun position calculated: Az=%.1f°, El=%.1f°", sun_az, sun_el);
    
    if (sun_el <= 10.0f) {
        ESP_LOGW(TAG, "Sun elevation too low (%.1f°) - deployment cancelled", sun_el);
        deploy_in_progress = false;
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Sun elevation acceptable (%.1f°) - continuing with deployment", sun_el);
    
    // Calculate targets
    float target_azimuth = sun_az - home_bearing;
    while (target_azimuth > 270.0f) target_azimuth -= 360.0f;
    while (target_azimuth < -270.0f) target_azimuth += 360.0f;
    float optimal_tilt = 90.0f - sun_el;
    if (optimal_tilt > 65.0f) {
        optimal_tilt = 65.0f;
    }
    
    ESP_LOGI(TAG, "Deploy targets: Azimuth=%.1f°, Tilt=%.1f°", target_azimuth, optimal_tilt);
    
    // FIRST: Move elevation with synchronized movement
    ESP_LOGI(TAG, "Step 1: Selecting elevation/panels motor group");
    select_motor_group(GROUP_ELEVATION_PANELS);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ESP_LOGI(TAG, "Step 2: Setting up synchronized elevation movement");
    sync_state.enabled = true;
    sync_state.motor_mask = 0x0030;  // Motors 4 and 5 (elevation)
    sync_state.target_position = optimal_tilt;
    sync_state.tolerance = SYNC_TOLERANCE_ELEV;
    sync_state.is_moving = true;
    
    motor_states[4].max_speed = MAX_SPEED;
    motor_states[5].max_speed = MAX_SPEED;
    
    ESP_LOGI(TAG, "Step 3: Starting extension deployment and elevation movement");
    ESP_LOGI(TAG, "Deploying extension panels (3 minutes)");
    set_motor_speed(7, 200);
    set_motor_speed(8, 200);
    set_motor_speed(9, 200);
    set_motor_speed(10, 200);
    extension_start_time = esp_timer_get_time() / 1000;
    
    ESP_LOGI(TAG, "Step 4: Waiting for elevation and extensions to complete");
    int wait_count = 0;
    bool elevation_complete = false;
    
    while ((!elevation_complete || extension_start_time > 0) && deploy_in_progress) {
        vTaskDelay(pdMS_TO_TICKS(100));
        
        if (!deploy_in_progress) {
            ESP_LOGI(TAG, "Deployment cancelled by user");
            sync_state.enabled = false;
            sync_state.is_moving = false;
            if (extension_start_time > 0) {
                set_motor_speed(7, 0);
                set_motor_speed(8, 0);
                set_motor_speed(9, 0);
                set_motor_speed(10, 0);
            }
            vTaskDelete(NULL);
            return;
        }
        
        if (!elevation_complete && !sync_state.is_moving) {
            elevation_complete = true;
            ESP_LOGI(TAG, "Elevation synchronized at %.1f° (error: %.2f°)", 
                    sync_state.avg_position, sync_state.max_error);
        }
        
        if (extension_start_time > 0) {
            uint32_t elapsed = (esp_timer_get_time() / 1000) - extension_start_time;
            
            if (elapsed >= EXTENSION_TIME_MS) {
                set_motor_speed(7, 0);
                set_motor_speed(8, 0);
                set_motor_speed(9, 0);
                set_motor_speed(10, 0);
                extensions_deployed = true;
                extension_start_time = 0;
                ESP_LOGI(TAG, "Extension panels deployed");
            } else if (wait_count % 100 == 0) {
                ESP_LOGI(TAG, "Extensions: %lu seconds remaining", 
                        (EXTENSION_TIME_MS - elapsed) / 1000);
            }
        }
        
        wait_count++;
        if (wait_count > 3600) {
            ESP_LOGW(TAG, "Deployment timeout");
            break;
        }
    }
    
    sync_state.enabled = false;
    
    // SECOND: Move slew
    ESP_LOGI(TAG, "Step 5: Moving slew to %.1f°", target_azimuth);
    select_motor_group(GROUP_SLEW);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    motor_states[6].target_position = target_azimuth;
    motor_states[6].is_moving = true;
    motor_states[6].max_speed = MAX_SPEED;
    
    wait_count = 0;
    while (motor_states[6].is_moving && wait_count < 600 && deploy_in_progress) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_count++;
        
        if (!deploy_in_progress) {
            ESP_LOGI(TAG, "Deployment cancelled during slew movement");
            motor_states[6].is_moving = false;
            motor_states[6].target_speed = 0;
            vTaskDelete(NULL);
            return;
        }
    }
    
    if (Deploy_All_API_Enable_Flag) {
        ESP_LOGI(TAG, "Deploy All COMPLETE! Time synced ✓, Sync mode enabled ✓");
        ESP_LOGI(TAG, "Final positions: Sun Az=%.1f°, Array Az=%.1f°, Sun El=%.1f°, Array Tilt=%.1f°", 
                sun_az, motor_states[6].position, sun_el, sync_state.avg_position);
    } else {
        ESP_LOGI(TAG, "Deploy complete!");
        ESP_LOGI(TAG, "Final positions: Sun Az=%.1f°, Array Az=%.1f°, Sun El=%.1f°, Array Tilt=%.1f°", 
                sun_az, motor_states[6].position, sun_el, sync_state.avg_position);
    }
    
    deploy_in_progress = false;
    vTaskDelete(NULL);
}

void stow_task(void *arg) {
    uint32_t extension_start_time = 0;
    const uint32_t EXTENSION_TIME_MS = 180000;
    
    while (tracking_in_progress) {
        ESP_LOGI(TAG, "Waiting for tracking to complete before stowing");
        vTaskDelay(pdMS_TO_TICKS(1000));
    } 
    stow_in_progress = true;
    ESP_LOGI(TAG, "Starting stow sequence");
    
    select_motor_group(GROUP_ELEVATION_PANELS);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    sync_state.enabled = true;
    sync_state.motor_mask = 0x0030;
    sync_state.target_position = 0;
    sync_state.tolerance = SYNC_TOLERANCE_ELEV;
    sync_state.is_moving = true;
    
    motor_states[4].max_speed = MAX_SPEED;
    motor_states[5].max_speed = MAX_SPEED;
    
    ESP_LOGI(TAG, "Retracting extension panels (5 minutes)");
    set_motor_speed(7, -200);
    set_motor_speed(8, -200);
    set_motor_speed(9, -200);
    set_motor_speed(10, -200);
    extension_start_time = esp_timer_get_time() / 1000;
    
    int wait_count = 0;
    bool elevation_complete = false;
    
    while ((!elevation_complete || extension_start_time > 0) && stow_in_progress) {
        vTaskDelay(pdMS_TO_TICKS(100));
        
        if (!stow_in_progress) {
            ESP_LOGI(TAG, "Stow cancelled by user");
            sync_state.enabled = false;
            sync_state.is_moving = false;
            if (extension_start_time > 0) {
                set_motor_speed(7, 0);
                set_motor_speed(8, 0);
                set_motor_speed(9, 0);
                set_motor_speed(10, 0);
            }
            vTaskDelete(NULL);
            return;
        }
        
        if (!elevation_complete && !sync_state.is_moving) {
            elevation_complete = true;
            ESP_LOGI(TAG, "Elevation at stow position (synchronized)");
        }
        
        if (extension_start_time > 0) {
            uint32_t elapsed = (esp_timer_get_time() / 1000) - extension_start_time;
            if (elapsed >= EXTENSION_TIME_MS) {
                set_motor_speed(7, 0);
                set_motor_speed(8, 0);
                set_motor_speed(9, 0);
                set_motor_speed(10, 0);
                extensions_deployed = false;
                extension_start_time = 0;
                ESP_LOGI(TAG, "Extension panels retracted");
            } else if (wait_count % 100 == 0) {
                ESP_LOGI(TAG, "Retracting: %lu seconds remaining", 
                        (EXTENSION_TIME_MS - elapsed) / 1000);
            }
        }
        
        wait_count++;
        if (wait_count > 3600) {
            ESP_LOGW(TAG, "Stow timeout");
            break;
        }
    }
    
    sync_state.enabled = false;
    
    ESP_LOGI(TAG, "Moving slew to home position");
    select_motor_group(GROUP_SLEW);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    motor_states[6].target_position = 0;
    motor_states[6].is_moving = true;
    motor_states[6].max_speed = MAX_SPEED;
    
    wait_count = 0;
    while (motor_states[6].is_moving && wait_count < 600 && stow_in_progress) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_count++;
        
        if (!stow_in_progress) {
            ESP_LOGI(TAG, "Stow cancelled during slew movement");
            motor_states[6].is_moving = false;
            motor_states[6].target_speed = 0;
            vTaskDelete(NULL);
            return;
        }
    }
    
    ESP_LOGI(TAG, "Stow complete!");
    stow_in_progress = false;
    auto_dep_flag = true;
    vTaskDelete(NULL);
}

void auto_tracking_task(void *arg) {
    auto_tracking_handle = xTaskGetCurrentTaskHandle();
    
    while (1) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(900000));
        
        if (auto_tracking && time_synced && !tracking_in_progress && !deploy_in_progress && !stow_in_progress) {
            ESP_LOGI(TAG, "Auto tracking check - extensions_deployed: %d", extensions_deployed);
            
            float sun_az_local, sun_el_local;
            calculate_sun_position(&sun_az_local, &sun_el_local);
            sun_az = sun_az_local;
            sun_el = sun_el_local;
            float optimal_tilt;
            
            if (sun_el > 10.0f) {
                stow_flag = true;
                
                if (auto_dep_flag == true && !extensions_deployed) {
                    ESP_LOGI(TAG, "Auto-deploy triggered - Sun elevation is %.1f°", sun_el);
                    xTaskCreate(deploy_task, "deploy", 4096, NULL, 4, NULL);
                    auto_dep_flag = false;
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    continue;
                }
                
                if (extensions_deployed) {
                    tracking_in_progress = true;
                    ESP_LOGI(TAG, "Starting tracking update");
                    
                    float target_azimuth = sun_az - home_bearing;
                    while (target_azimuth > 270.0f) target_azimuth -= 360.0f;
                    while (target_azimuth < -270.0f) target_azimuth += 360.0f;
                    
                    optimal_tilt = 90.0f - sun_el;
                    if (optimal_tilt > 65.0f) {
                        optimal_tilt = 65.0f;
                    }
                    
                    select_motor_group(GROUP_ELEVATION_PANELS);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    
                    sync_state.enabled = true;
                    sync_state.motor_mask = 0x0030;
                    sync_state.target_position = optimal_tilt;
                    sync_state.tolerance = SYNC_TOLERANCE_ELEV;
                    sync_state.is_moving = true;
                    motor_states[4].max_speed = MAX_SPEED;
                    motor_states[5].max_speed = MAX_SPEED;
                    
                    ESP_LOGI(TAG, "Tracking: Moving elevation to %.1f°", optimal_tilt);
                    
                    int wait_count = 0;
                    while (sync_state.is_moving && wait_count < 600 && tracking_in_progress) {
                        vTaskDelay(pdMS_TO_TICKS(100));
                        wait_count++;
                        if (!tracking_in_progress) {
                            sync_state.enabled = false;
                            sync_state.is_moving = false;
                            break;
                        }
                    }
                    
                    sync_state.enabled = false;
                    if (!tracking_in_progress) {
                        ESP_LOGI(TAG, "Tracking cancelled");
                        continue;
                    }
                    
                    select_motor_group(GROUP_SLEW);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    motor_states[6].target_position = target_azimuth;
                    motor_states[6].is_moving = true;
                    motor_states[6].max_speed = MAX_SPEED;
                    ESP_LOGI(TAG, "Tracking: Moving slew to %.1f°", target_azimuth);
                    
                    wait_count = 0;
                    while (motor_states[6].is_moving && wait_count < 600 && tracking_in_progress) {
                        vTaskDelay(pdMS_TO_TICKS(100));
                        wait_count++;
                    }
                    
                    ESP_LOGI(TAG, "Tracking update complete");
                    tracking_in_progress = false;
                } else {
                    ESP_LOGI(TAG, "Waiting for extensions to deploy before tracking");
                }
            }
            else if (sun_el < 10.0f && stow_flag == true) {
                xTaskCreate(stow_task, "stow", 4096, NULL, 4, NULL);
                stow_flag = false;
            }
            else {
                ESP_LOGI(TAG, "Sun too low for tracking (%.1f°)", sun_el);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(900000));
    }
}

void raise_legs_task(void *arg) {
    uint32_t params = (uint32_t)arg;
    float target_position = (float)(params & 0xFFFF) / 100.0f;
    int motor_speed = (params >> 16) & 0xFF;
    
    ESP_LOGI(TAG, "=== RAISE LEGS TASK STARTED ===");
    ESP_LOGI(TAG, "Target absolute position: %.1fmm, Speed: %d", target_position, motor_speed);
    
    if (!time_synced) {
        time_t current_timestamp = time(NULL);
        if (current_timestamp < 1640995200) {
            current_timestamp = 1751875200;
        }
        
        struct timeval tv = {
            .tv_sec = current_timestamp,
            .tv_usec = 0
        };
        settimeofday(&tv, NULL);
        time_synced = true;
        
        struct tm timeinfo;
        localtime_r(&current_timestamp, &timeinfo);
        char time_str[64];
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);
        ESP_LOGI(TAG, "✓ Time synced automatically for raise legs: %s (timestamp: %lld)", 
                time_str, (long long)current_timestamp);
    }
    
    if (!sync_mode) {
        sync_mode = true;
        ESP_LOGI(TAG, "✓ Synchronized movement mode enabled for raise legs operation");
    }
    
    ESP_LOGI(TAG, "Step 3: Selecting legs motor group");
    select_motor_group(GROUP_LEGS);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ESP_LOGI(TAG, "Step 4: Enabling synchronized movement for legs");
    sync_state.enabled = true;
    sync_state.motor_mask = 0x000F;
    sync_state.target_position = target_position;
    sync_state.tolerance = SYNC_TOLERANCE_LEG;
    sync_state.is_moving = true;
    
    ESP_LOGI(TAG, "Step 5: Setting speed %d and target position %.1fmm for all leg motors", motor_speed, target_position);
    for (int i = 0; i < 4; i++) {
        motor_states[i].max_speed = motor_speed;
        motor_states[i].target_position = target_position;
        motor_states[i].is_moving = true;
        ESP_LOGI(TAG, "Leg %d: Current=%.1fmm, Target=%.1fmm", 
                i, motor_states[i].position, target_position);
    }
    
    ESP_LOGI(TAG, "Step 6: Waiting for legs to reach target position");
    
    int wait_count = 0;
    const int max_wait = 1200;
    
    while (sync_state.is_moving && wait_count < max_wait) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_count++;
        
        if (wait_count % 50 == 0) {
            ESP_LOGI(TAG, "Moving legs to position... Avg position: %.1fmm, Target: %.1fmm, Error: %.2fmm", 
                    sync_state.avg_position, sync_state.target_position, sync_state.max_error);
        }
    }
    
    ESP_LOGI(TAG, "Step 7: Disabling synchronized movement");
    sync_state.enabled = false;
    sync_state.is_moving = false;
    
    ESP_LOGI(TAG, "Step 8: Deselecting motor group (turning off relays)");
    select_motor_group(GROUP_NONE);
    
    if (wait_count >= max_wait) {
        ESP_LOGW(TAG, "=== LEGS MOVEMENT TIMEOUT ===");
    } else {
        ESP_LOGI(TAG, "=== LEGS MOVEMENT COMPLETED ===");
        ESP_LOGI(TAG, "Final average position: %.1fmm, Max error: %.2fmm", 
                sync_state.avg_position, sync_state.max_error);
    }
    
    ESP_LOGI(TAG, "✓ Synchronized movement mode remains enabled");
    
    vTaskDelete(NULL);
}

void drop_legs_task(void *arg) {
    uint32_t params = (uint32_t)arg;
    float target_position = (float)(params & 0xFFFF) / 100.0f;
    int motor_speed = (params >> 16) & 0xFF;
    
    ESP_LOGI(TAG, "=== DROP LEGS TASK STARTED ===");
    ESP_LOGI(TAG, "Target absolute position: %.1fmm, Speed: %d", target_position, motor_speed);
    
    if (!time_synced) {
        time_t current_timestamp = time(NULL);
        if (current_timestamp < 1640995200) {
            current_timestamp = 1751875200;
        }
        
        struct timeval tv = {
            .tv_sec = current_timestamp,
            .tv_usec = 0
        };
        settimeofday(&tv, NULL);
        time_synced = true;
        
        struct tm timeinfo;
        localtime_r(&current_timestamp, &timeinfo);
        char time_str[64];
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);
        ESP_LOGI(TAG, "✓ Time synced automatically for drop legs: %s (timestamp: %lld)", 
                time_str, (long long)current_timestamp);
    }
    
    if (!sync_mode) {
        sync_mode = true;
        ESP_LOGI(TAG, "✓ Synchronized movement mode enabled for drop legs operation");
    }
    
    ESP_LOGI(TAG, "Step 3: Selecting legs motor group");
    select_motor_group(GROUP_LEGS);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ESP_LOGI(TAG, "Step 4: Current positions and absolute target");
    for (int i = 0; i < 4; i++) {
        ESP_LOGI(TAG, "Leg %d: Current=%.1fmm, Target=%.1fmm", 
                i, motor_states[i].position, target_position);
    }
    
    ESP_LOGI(TAG, "Step 5: Enabling synchronized movement for legs");
    sync_state.enabled = true;
    sync_state.motor_mask = 0x000F;
    sync_state.target_position = target_position;
    sync_state.tolerance = SYNC_TOLERANCE_LEG;
    sync_state.is_moving = true;
    
    ESP_LOGI(TAG, "Step 6: Setting speed %d and target position %.1fmm for all leg motors", motor_speed, target_position);
    for (int i = 0; i < 4; i++) {
        motor_states[i].max_speed = motor_speed;
        motor_states[i].target_position = target_position;
        motor_states[i].is_moving = true;
    }
    
    ESP_LOGI(TAG, "Step 7: Waiting for legs to reach target position");
    
    int wait_count = 0;
    const int max_wait = 1200;
    
    while (sync_state.is_moving && wait_count < max_wait) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_count++;
        
        if (wait_count % 50 == 0) {
            ESP_LOGI(TAG, "Moving legs to position... Avg position: %.1fmm, Target: %.1fmm, Error: %.2fmm", 
                    sync_state.avg_position, sync_state.target_position, sync_state.max_error);
        }
    }
    
    ESP_LOGI(TAG, "Step 8: Disabling synchronized movement");
    sync_state.enabled = false;
    sync_state.is_moving = false;
    
    ESP_LOGI(TAG, "Step 9: Deselecting motor group (turning off relays)");
    select_motor_group(GROUP_NONE);
    
    if (wait_count >= max_wait) {
        ESP_LOGW(TAG, "=== LEGS MOVEMENT TIMEOUT ===");
    } else {
        ESP_LOGI(TAG, "=== LEGS MOVEMENT COMPLETED ===");
        ESP_LOGI(TAG, "Final average position: %.1fmm, Max error: %.2fmm", 
                sync_state.avg_position, sync_state.max_error);
    }
    
    ESP_LOGI(TAG, "✓ Synchronized movement mode remains enabled");
    
    vTaskDelete(NULL);
}