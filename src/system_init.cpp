#include "system_init.h"


/* MAX6675 soft SPI */
int8_t thermoDO = 17;
int8_t thermoCS = 16;
int8_t thermoCLK = 4;

ESP32PWM pwm;
ESP32PWM pwm_fan;
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
	last_system_info.temputer = -1;
	last_system_info.ina226.voltage = -1;
	last_system_info.ina226.current = -1;
	last_system_info.ina226.power = -1;
    last_system_info.target_temputer = -1;
	last_system_info.encoder = 0;
	last_system_info.pwm_precent = 1.0f;
	last_system_info.holt_mode = HOT_MODE_MAX;

	system_info.temputer = 0;
	system_info.ina226.voltage = 0;
	system_info.ina226.current = 0;
	system_info.ina226.power = 0;
    system_info.target_temputer = 0;
	system_info.encoder = 0;
	system_info.pwm_precent = 0.0f;
	system_info.holt_mode = HOT_MODE_STANDBY;

	/* origin target temputer */
    system_info.target_temputer = 25;
}

void pwm_init(void)
{
	// pinMode(PWM_PIN, OUTPUT);
	pwm.attachPin(PWM_PIN, 20000, 10); // 2KHz 8 bit
	pwm.writeScaled(0.0f);

	/* 风扇 */
	pwm_fan.attachPin(14, 20000, 10); // 2KHz 8 bit
	pwm_fan.writeScaled(0.0f);
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

/*
	0: not active
	1: avtive 
*/
bool update_encoder_value(void)
{
	if (rotaryEncoder.encoderChanged()) {
		system_info.encoder = rotaryEncoder.readEncoder();
		Serial.printf("[Encoder Value] last_Value:%d  Now_Value:%d \r\n", 
								last_system_info.encoder, system_info.encoder);
		return true;
	}

	return false;
}

ButtonState update_button_status(void)
{
	ButtonState status;
	static bool previous_butt_state = false;
	uint8_t butt_state = rotaryEncoder.isEncoderButtonDown();

	if (butt_state && !previous_butt_state) {
		previous_butt_state = true;
		// Serial.printf("[Encoder Key] %d Button Pushed \r\n", status);
		status = BUT_PUSHED;
	} else if (!butt_state && previous_butt_state) {
		previous_butt_state = false;
		// Serial.printf("[Encoder Key] %d Button Released \r\n", status);
		status = BUT_RELEASED;
	} else {
		status = (butt_state ? BUT_DOWN : BUT_UP);
		// Serial.printf("[Encoder Key] %d %s \r\n", status, (butt_state ? "BUT_DOWN" : "BUT_UP"));
	}

	return status; 
}

void update_temputer_sensor(void)
{
	system_info.temputer = thermocouple.readCelsius();
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
		pwm.writeScaled(0.0f);
	} else if (duty > 1.0) {
		pwm.writeScaled(1.0f);
	} else {
		pwm.writeScaled(duty);
	}
}

void update_pwm_fan_out(float duty)
{
	if (duty < 0.0) {
		pwm_fan.writeScaled(0.0f);
	} else if (duty > 1.0) {
		pwm_fan.writeScaled(1.0f);
	} else {
		pwm_fan.writeScaled(duty);
	}
}

void EEPROM_init(void)
{
	Serial.println("\r\n EEPROM_init \r\n");

	if (!EEPROM.begin(EEPROM_SIZE)) {
		Serial.printf("failed to initialise EEPROM \r\n");
	}

	/* only for test */
	// Serial.println(" bytes read from Flash . Values are:");
	// for (int i = 0; i < EEPROM_SIZE; i++) {
	// 	EEPROM.write(i, i);
	// 	Serial.printf(byte(EEPROM.read(i))); Serial.print(" ");
	// }
}


