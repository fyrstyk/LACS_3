#ifndef SUN_TRACKING_H
#define SUN_TRACKING_H

#include <stdbool.h>
#include <stdint.h>

// Function prototypes
void calculate_sun_position(float *azimuth, float *elevation);
void auto_tracking_task(void *arg);
void deploy_task(void *arg);
void stow_task(void *arg);
void raise_legs_task(void *arg);
void drop_legs_task(void *arg);

// Global state variables
extern bool auto_tracking;
extern bool time_synced;
extern bool tracking_in_progress;
extern bool deploy_in_progress;
extern bool stow_in_progress;
extern bool stow_flag;
extern bool extensions_deployed;
extern bool extension_startup_protect;
extern bool fixed_elevation_flag;
extern bool auto_dep_flag;
extern bool deploy_done_flag;
extern bool boot_flag;
extern bool Deploy_All_API_Enable_Flag;
extern float sun_az;
extern float sun_el;
extern float latitude;
extern float longitude;
extern float home_bearing;
extern TaskHandle_t auto_tracking_handle;

#endif // SUN_TRACKING_H