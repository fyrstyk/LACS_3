#ifndef STORAGE_H
#define STORAGE_H

#include "nvs.h"

// Function prototypes
void storage_init(void);
void save_calibration(void);
void load_calibration(void);
void save_home_bearing(void);
void load_home_bearing(void);
void save_location(void);
void load_location(void);

// Global NVS handle
extern nvs_handle_t nvs_handle_storage;

#endif // STORAGE_H