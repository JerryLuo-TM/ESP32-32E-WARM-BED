#include "system_init.h"


/* MAX6675 soft SPI */
int8_t thermoDO = 17;
int8_t thermoCS = 16;
int8_t thermoCLK = 4;

ESP32PWM pwm;
INA226 ina(Wire);
TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, 
                                                            ROTARY_ENCODER_B_PIN, 
                                                            ROTARY_ENCODER_BUTTON_PIN, 
                                                            ROTARY_ENCODER_VCC_PIN, 
                                                            ROTARY_ENCODER_STEPS);


void IRAM_ATTR readEncoderISR()
{
	rotaryEncoder.readEncoder_ISR();
}


void parameter_init(void)
{
	system_info.last_temputer = -1;
	system_info.last_ina226.voltage = -1;
	system_info.last_ina226.current = -1;
	system_info.last_ina226.power = -1;
    system_info.last_target_temputer = -1;
	system_info.last_encoder = 1;
	system_info.last_pwm_precent = 1.0f;
	system_info.last_holt_mode = HOT_MODE_MAX;

	system_info.temputer = 0;
	system_info.ina226.voltage = 0;
	system_info.ina226.current = 0;
	system_info.ina226.power = 0;
    system_info.target_temputer = 0;
	system_info.encoder = 0;
	system_info.pwm_precent = 0.0f;
	system_info.holt_mode = HOT_MODE_STANDBY;

	/* origin target temputer */
    system_info.target_temputer = 65;
}

void pwm_init(void)
{
	// pinMode(PWM_PIN, OUTPUT);
	pwm.attachPin(PWM_PIN, 20000, 10); // 2KHz 8 bit
}


void max6675_init(void)
{
	Serial.println("max6675_init");
}


void encoder_init(void)
{
	Serial.println("encoder_init");

	rotaryEncoder.begin();
	rotaryEncoder.setup(readEncoderISR);
	rotaryEncoder.setBoundaries(0, 1000, true);
	rotaryEncoder.setAcceleration(0);  /* 0 or 1 means disabled acceleration */
}


void ina226_init(void)
{
	Serial.println("ina226_init");

	Wire.begin();

	bool success = ina.begin();	// Default INA226 address is 0x40
	if(!success) {
		Serial.println("INA226 Connection error");
		while(1);
	}

	// Configure INA226
	ina.configure(	INA226_AVERAGES_1, 
					INA226_BUS_CONV_TIME_1100US, 
					INA226_SHUNT_CONV_TIME_1100US, 
					INA226_MODE_SHUNT_BUS_CONT);

	// Calibrate INA226. Rshunt = 0.01 ohm, Max excepted current = 4A
	ina.calibrate(0.005, 16);

	// Enable Power Over-Limit Alert
	ina.enableOverPowerLimitAlert();
	ina.setPowerLimit(0.130);
	ina.setAlertLatch(true);
}


void st7789_init(void)
{
	Serial.println("st7789_init");

	tft.init();
	tft.setRotation(1);
	tft.setTextColor(TFT_WHITE, TFT_BLACK, true);

    tft.fillScreen(TFT_BLACK);
	tft.drawCentreString("THERM MODE", 120, 4, 4);
}

void update_encoder_key(void)
{
	static unsigned long lastTimePressed = 0;

	if (rotaryEncoder.encoderChanged()) {
		system_info.encoder = rotaryEncoder.readEncoder();
		Serial.printf("Encoder Value: %d \r\n", system_info.encoder);
	}

	if (rotaryEncoder.isEncoderButtonClicked()) {
		if (millis() - lastTimePressed < 500) {
            return;
        }
        Serial.print("button pressed ");
	}
}


void update_temputer_sensor(void)
{
	static uint32_t count = 0;

	count += 1;
	if ((count % 3) == 0) {
		system_info.temputer = thermocouple.readCelsius();
	}
}


void update_power_sensor(void)
{
	system_info.ina226.voltage = ina.readBusVoltage();
	system_info.ina226.current = ina.readShuntCurrent();
	system_info.ina226.power = ina.readBusPower();
}

void update_pwm_out(float duty)
{
	if (duty < 0.0) {
		pwm.writeScaled(0.0);
	} else if (duty > 1.0) {
		pwm.writeScaled(1.0);
	} else {
		pwm.writeScaled(duty);
	}
}

