#ifndef ADAFRUIT_SYSTEM_INIT_H
#define ADAFRUIT_SYSTEM_INIT_H


#include "Arduino.h"
#include <stdio.h>


#include "max6675.h"
#include "AiEsp32RotaryEncoder.h"

#include <SPI.h>
#include <TFT_eSPI.h>

#include <Wire.h>
#include <INA226.h>

#include <ESP32Servo.h>

/* PWM GPIO PIN */
#define PWM_PIN 27

/* EC Rotary encoder */
#define ROTARY_ENCODER_A_PIN        33
#define ROTARY_ENCODER_B_PIN        32
#define ROTARY_ENCODER_BUTTON_PIN   0
#define ROTARY_ENCODER_VCC_PIN     -1
#define ROTARY_ENCODER_STEPS        2  //try 1,2 or 4 to get expected behaviour

#define HOT_MODE_STANDBY        0
#define HOT_MODE_PRE_HEAT       1
#define HOT_MODE_KEEP_WARM      2
#define HOT_MODE_WELD           3
#define HOT_MODE_COLD           4
#define HOT_MODE_MAX            5

typedef struct {
    float voltage;
    float current;
    float power;
} __attribute__ ((packed)) ina226_t;

typedef struct {
    /* last value */
    ina226_t last_ina226;
    float last_temputer;
    uint32_t last_encoder;
    uint32_t last_target_temputer;
    float last_pwm_precent;
    uint32_t last_holt_mode;
    /* current value */
    ina226_t ina226;
    float temputer;
    uint32_t encoder;
    uint32_t target_temputer;
    float pwm_precent;
    uint32_t holt_mode;
} __attribute__ ((packed)) system_info_t;

extern system_info_t system_info;
extern TFT_eSPI tft;

extern bool g_key_is_press;

void parameter_init(void);
void pwm_init(void);
void max6675_init(void);
void encoder_init(void);
void ina226_init(void);
void st7789_init(void);

void update_encoder_key(void);
void update_power_sensor(void);
void update_temputer_sensor(void);
void update_pwm_out(float duty);

#endif


