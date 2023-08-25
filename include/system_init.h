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

/* task cycle ms */
#define DATA_TASK_CYCLE             100.0f
#define UI_TASK_CYCLE               20.0f

/* weld reflow soldering T */
#define TASK_CYCLE                  (UI_TASK_CYCLE / 1000.0f)

/* weld reflow soldering mode */
typedef enum {
    HOT_MODE_STANDBY = 0,
    HOT_MODE_PRE_HEAT,
    HOT_MODE_KEEP_WARM,
    HOT_MODE_WELD,
    HOT_MODE_COLD,
    HOT_MODE_MAX,
} weld_heat_mode;

/* power IC patamerter */
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
    float last_target_temputer;
    float last_pwm_precent;
    uint32_t last_holt_mode;
    /* current value */
    ina226_t ina226;
    float temputer;
    uint32_t encoder;
    float target_temputer;
    float pwm_precent;
    uint32_t holt_mode;
} __attribute__ ((packed)) system_info_t;

typedef struct {
    float target_temp_min;
	float target_temp;
    float target_temp_max;

    float heat_rate_min;
	float heat_rate;
    float heat_rate_max;

    unsigned long timer_start;
	uint32_t heat_sec;
} __attribute__ ((packed)) solder_parameter_t;

extern system_info_t system_info;
extern TFT_eSPI tft;

void parameter_init(void);
void pwm_init(void);
void max6675_init(void);
void encoder_init(void);
void ina226_init(void);
void st7789_init(void);

bool update_encoder_key(void);
void update_power_sensor(void);
void update_temputer_sensor(void);
void update_pwm_out(float duty);

#endif


