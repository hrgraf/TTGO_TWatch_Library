// ------------------------------------------------------------------------
// Watch GUI
// Copyright (c) 2020 by H.R.Graf
// ------------------------------------------------------------------------

#ifndef __GUI_H__
#define __GUI_H__

// logging
//#define MY_LOG(...)
//#define MY_LOG LV_LOG_USER
#ifdef _WIN32
#include "stdio.h"
#define MY_LOG(fmt, ...) printf(fmt "\n", __VA_ARGS__)
#else
#include "HardwareSerial.h"
#define MY_LOG(...) { Serial.printf(__VA_ARGS__); Serial.println(""); }
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ------------------------------------------------------------------------
// Watch HW

void update_time(bool update_date);

uint16_t get_bat_level(void);
uint16_t get_bat_charging(void);

uint32_t get_free_mem(void);

void get_accel(lv_point_t *dir);

void wifi_list_add(const char *ssid);
void wifi_connect_status(bool result);
void ntp_sync_time(void);

void play_sound(void);
//bool run_audio(void);

// ------------------------------------------------------------------------
// Watch GUI

void updateBatteryCharge(void);
void updateBatteryCalc(void);
void updateBatteryLevel(void);

void updateStepCounter(uint32_t counter);

void updateTime(uint16_t hour, uint16_t min, uint16_t sec);
void updateDate(uint16_t year, uint16_t month, uint16_t day, uint16_t weekday);

void setupGui(void);
void showHome(void);
void showApp(uint16_t corner);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // __GUI_H__

// ------------------------------------------------------------------------
