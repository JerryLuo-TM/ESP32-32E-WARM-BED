#include <Arduino.h>
#include <TaskScheduler.h>
#include "system_init.h"

#include "font/hwkt_hz24.h"
#include "font/hwkt_hz32.h"
#include "font/fzchsjt_zm24.h"
#include "font/fzchsjt_zm64.h"

system_info_t system_info;
Scheduler runner;

/* OS */
void Task_Data_Callback();
void Task_GUI_Callback();
Task task_data(100, TASK_FOREVER, &Task_Data_Callback);  //100ms
Task task_gui(20, TASK_FOREVER, &Task_GUI_Callback);	 //20ms

void setup()
{
	Serial.begin(115200);

	parameter_init();

	pwm_init();

	encoder_init();

	max6675_init();

	ina226_init();

	st7789_init();

	/* START TASK SCHEDULE */
	runner.init();
	runner.addTask(task_data);
	runner.addTask(task_gui);
	task_data.enable();
	task_gui.enable();
}

void gui_thermost_mode_init(void)
{
    tft.fillScreen(TFT_BLACK);

	// tft.drawFastHLine(0, 40, 240, TFT_RED);
	// tft.drawFastHLine(0, 112, 240, TFT_RED);
	// tft.drawFastHLine(0, 184, 240, TFT_RED);
	// tft.drawFastVLine(120, 0, 240, TFT_RED);

	tft.setTextDatum(TL_DATUM);/* 左顶 */
	tft.loadFont(hwkt_32);
	tft.setTextColor(TFT_GOLD, TFT_BLACK, true);
	tft.setCursor(44, 4);
	tft.println("恒 温 模 式");
	tft.unloadFont();

	tft.loadFont(hwkt_24);
	tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
	tft.setCursor(1, 50);  tft.println("当前");
	tft.setCursor(1, 80);  tft.println("温度");
	tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
	tft.setCursor(1, 122); tft.println("设定");
	tft.setCursor(1, 152); tft.println("温度");
	tft.setTextColor(TFT_ORANGE, TFT_BLACK, true);
	tft.setCursor(1, 186); tft.println("电压");
	tft.setCursor(1, 214); tft.println("电流");
	tft.setTextColor(TFT_ORANGE, TFT_BLACK, true);
	tft.setCursor(124, 186); tft.println("功率");
	tft.setCursor(124, 214); tft.println("输出");
	tft.unloadFont();
}

double Kp = 5.0, Ki = 0.001, Kd = 1.0;
double last_value = 0, value = 0;
double error_Temp = 0, Output_Heat = 0;
double dt = 0.02;

void gui_thermost_mode_refresh(void)
{
	char buf[50];
	float pwm_duty = 0.0;

	update_encoder_key();

	/* 温控部分 PID */
	//系统误差
	value = (float)system_info.target_temputer - system_info.temputer;
	last_value = value;
	error_Temp += value * dt;
	if (error_Temp > 500) {error_Temp = 500;}
	if (error_Temp < 0.0) {error_Temp = 0.0;}
	Output_Heat = Kp * value + Ki * error_Temp + Kd * (value - last_value);
	Serial.printf("pwm_out: %f", Output_Heat);
	if (Output_Heat > 100) {Output_Heat = 100;}
	if (Output_Heat < 0.0) {Output_Heat = 0.001;}
	system_info.pwm_precent = (int32_t)Output_Heat;
	Serial.printf("system_info.pwm_precent: %f \r\n", system_info.pwm_precent);

	/* 旋转编码器 处理 */
	if (system_info.last_encoder != system_info.encoder) {
		if (system_info.last_encoder != 1000) {
			if (system_info.last_encoder < system_info.encoder) {
				system_info.target_temputer += 1;
				// system_info.pwm_precent += 1.0f;
			} else {
				system_info.target_temputer -= 1;
				// system_info.pwm_precent -= 1.0f;
			}
		} else {
			if (system_info.encoder == 0) {
				system_info.target_temputer += 1;
				// system_info.pwm_precent += 1.0f;
			} else if (system_info.encoder < 1000) {
				system_info.target_temputer -= 1;
				// system_info.pwm_precent -= 1.0f;
			}
		}
		if (system_info.target_temputer > 230) {system_info.target_temputer = 230;}
		else if (system_info.target_temputer < 20) {system_info.target_temputer = 20;}

		// if (system_info.pwm_precent > 100.0f) {system_info.pwm_precent = 100.0f;}
		// else if (system_info.pwm_precent < 0.0f) {system_info.pwm_precent = 0.0f;}
	}

	/* 当前温度 */
	if (system_info.last_temputer != system_info.temputer) {
		tft.setTextColor(TFT_CYAN, TFT_BLACK, true);
		tft.setTextDatum(TL_DATUM);/* 左上 */
		tft.loadFont(fzchsjt_64);
		sprintf(buf, "%03d℃", (int32_t)system_info.temputer);
		tft.drawString(buf, 62, 52);
		tft.unloadFont();
	}

	/* 设定温度*/
	if (system_info.last_target_temputer != system_info.target_temputer) {
		tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK, true);
		tft.setTextDatum(TL_DATUM);/* 左上 */
		tft.loadFont(fzchsjt_64);
		sprintf(buf, "%03d℃", (int32_t)system_info.target_temputer);
		tft.drawString(buf, 62, 124);
		tft.unloadFont();
	}

	/* 电压 */
	if (system_info.last_ina226.voltage != system_info.ina226.voltage) {
		tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
		tft.setTextDatum(TR_DATUM);/* 右上 */
		tft.loadFont(fzchsjt_24);
		sprintf(&buf[0], " ");
		if ((int32_t)system_info.ina226.voltage > 9) {sprintf(&buf[1], "%01d", (int32_t)system_info.ina226.voltage/10%10);} else {sprintf(&buf[1], " ");}
		sprintf(&buf[2], "%01d", (int32_t)(system_info.ina226.voltage)%10);
		sprintf(&buf[3], ".");
		sprintf(&buf[4], "%01d", (int32_t)(system_info.ina226.voltage*10.0)%10);
		buf[5] = 'V';
		buf[6] = '\0';
		tft.drawString(buf, 120, 186);
		tft.unloadFont();
	}

	/* 电流 */
	if (system_info.last_ina226.current != system_info.ina226.current) {
		tft.setTextColor(TFT_SKYBLUE, TFT_BLACK, true);
		tft.setTextDatum(TR_DATUM);/* 右上 */
		tft.loadFont(fzchsjt_24);
		sprintf(&buf[0], " ");
		if ((int32_t)system_info.ina226.current > 9) {sprintf(&buf[1], "%01d", (int32_t)system_info.ina226.current/10%10);} else {sprintf(&buf[1], " ");}
		sprintf(&buf[2], "%01d", (int32_t)system_info.ina226.current%10);
		sprintf(&buf[3], ".");
		sprintf(&buf[4], "%01d", (int32_t)(system_info.ina226.current*10.0)%10);
		buf[5] = 'A';
		buf[6] = '\0';
		tft.drawString(buf, 120, 214);
		tft.unloadFont();
	}

	/* 功率 */
	if (system_info.last_ina226.power != system_info.ina226.power) {
		tft.setTextColor(TFT_MAGENTA, TFT_BLACK, true);
		tft.setTextDatum(TR_DATUM);/* 右上 */
		tft.loadFont(fzchsjt_24);
		sprintf(&buf[0], "%03dW", ((int32_t)system_info.ina226.power));
		tft.drawString(buf, 240, 186);
		tft.unloadFont();
	}

	/* 百分比 */
	if (system_info.last_pwm_precent != system_info.pwm_precent) {
		tft.setTextColor(TFT_RED, TFT_BLACK, true);
		tft.setTextDatum(TR_DATUM);/* 右上 */
		tft.loadFont(fzchsjt_24);
		sprintf(&buf[0], " ");
		if ((int32_t)system_info.pwm_precent > 99) {sprintf(&buf[1], "%01d", (int32_t)system_info.pwm_precent/100%10);} else {sprintf(&buf[1], " ");}
		if ((int32_t)system_info.pwm_precent > 9) {sprintf(&buf[2], "%01d", (int32_t)system_info.pwm_precent/10%10);} else {sprintf(&buf[2], " ");}
		if ((int32_t)system_info.pwm_precent != 0) {sprintf(&buf[3], "%01d", (int32_t)system_info.pwm_precent%10);} else {sprintf(&buf[3], "0");}
		buf[4] = '%';
		buf[5] = '\0';
		tft.drawString(buf, 240, 214);
		tft.unloadFont();
	}

	/* 0.0 - 1.0 */
	update_pwm_out(system_info.pwm_precent/100.0f);

	system_info.last_temputer = system_info.temputer;
    system_info.last_target_temputer = system_info.target_temputer;
	system_info.last_ina226.voltage = system_info.ina226.voltage;
	system_info.last_ina226.current = system_info.ina226.current;
	system_info.last_ina226.power = system_info.ina226.power;
	system_info.last_encoder = system_info.encoder;
	system_info.last_pwm_precent = system_info.pwm_precent;
}

void gui_reflow_solder_mode_init(void)
{
	uint32_t sx, sy, ex, ey, len;

    tft.fillScreen(TFT_BLACK);

	tft.setTextDatum(TL_DATUM);/* 左顶 */
	tft.loadFont(hwkt_32);
	tft.setTextColor(TFT_GOLD, TFT_BLACK, true);
	tft.setCursor(24, 4);
	tft.println("回 流 焊 模 式");
	tft.unloadFont();

	// sx = 30; sy = 210; len = tft.width() - sx;
	// tft.drawFastHLine(sx, sy, len, TFT_RED);
	// tft.drawFastVLine(40, 40, 240, TFT_RED);

	// sx = 40; sy = 210;
	// ex = sx + (float)(160 - 25) / 1.2f * CELL_TIME;
	// ey = sy - (float)(160 - 25) * CELL_TEMP;
	// tft.drawLine(sx, sy, ex, ey, TFT_GREEN);

	// sx = ex; sy = ey;
	// ex = sx + (float)(180 - 160) / 0.4f * CELL_TIME;
	// ey = sy - (float)(180 - 160) * CELL_TEMP;
	// tft.drawLine(sx, sy, ex, ey, TFT_BLUE);

	// sx = ex; sy = ey;
	// ex = sx + (float)(217 - 180) / 3.0f * CELL_TIME;
	// ey = sy - (float)(217 - 180) * CELL_TEMP;
	// tft.drawLine(sx, sy, ex, ey, TFT_RED);

	// sx = ex; sy = ey;
	// ex = sx + (float)(240 - 217) / 0.75f * CELL_TIME;
	// ey = sy - (float)(240 - 217) * CELL_TEMP;
	// tft.drawLine(sx, sy, ex, ey, TFT_WHITE);

	// sx = ex; sy = ey;
	// ex = 240-1;
	// ey = 210;
	// tft.drawLine(sx, sy, ex, ey, TFT_WHITE);

	tft.drawFastHLine(0, 180 - 6, 240, TFT_DARKGREY);
	tft.drawFastHLine(0, 180 - 5, 240, TFT_DARKGREY);
	tft.drawFastHLine(0, 180 - 4, 240, TFT_DARKGREY);
	tft.loadFont(hwkt_24);
	tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
	tft.setCursor(1, 183);	tft.println("电压");
	tft.setTextColor(TFT_VIOLET, TFT_BLACK, true);
	tft.setCursor(1, 183 + 30);	tft.println("功率");

	tft.setTextColor(TFT_CYAN, TFT_BLACK, true);
	tft.setCursor(130 + 1, 183);	tft.println("温度");
	tft.setTextColor(TFT_ORANGE, TFT_BLACK, true);
	tft.setCursor(130 + 1, 183 + 30);	tft.println("状态"); // 状态 [ 待机 预热 恒温 回流 降温]
	tft.unloadFont();
}

bool g_key_is_press = false;
bool g_weld_running = false;
unsigned long g_weld_timestamp_start, g_weld_timestamp;

uint32_t one_sec_count;

//20ms
void gui_reflow_solder_mode_refresh(void)
{
	char buf[50];
	static unsigned long timestamps_start = 0, timestamps_seconds = 0;
	int32_t post_x, post_y;

	update_encoder_key();

	/* 旋转编码器 处理 */
	// if (system_info.last_encoder != system_info.encoder) {
	// 	if (system_info.last_encoder < system_info.encoder) {
	// 		system_info.holt_mode += 1;
	// 	} else if (system_info.last_encoder > system_info.encoder) {
	// 		system_info.holt_mode -= 1;
	// 	}

	/* 旋转编码器按钮 处理 */
	if (g_key_is_press == true) {
		if ((g_weld_running == false)
			&& (system_info.holt_mode == HOT_MODE_STANDBY)
			&& (system_info.temputer < 100)) {
			g_weld_running = true;
			system_info.holt_mode = HOT_MODE_PRE_HEAT;
			timestamps_start = millis();
			g_weld_timestamp_start = timestamps_start;
			tft.fillRect(0, 50, 240, 120, TFT_BLACK);
		}
		g_key_is_press = false;
	}

/*
	预热 20℃ -> 150℃	50S     1 ~ 3  ℃/S
	恒温 150℃			 90S     60 ~ 120 S
	回流 150℃ -> 235℃	30S		1 ~ 3 ℃/S
	降温 235℃ -> 20℃	50S		2 ~ 4 ℃/S
*/

	if (system_info.holt_mode == HOT_MODE_STANDBY) {
		system_info.target_temputer = 20;
	} else if (system_info.holt_mode == HOT_MODE_PRE_HEAT) {
		timestamps_seconds = (millis() - timestamps_start) / 1000;
		system_info.target_temputer += 2.5f;
		if (system_info.target_temputer > 160) {system_info.target_temputer = 160;}
		if ((system_info.temputer > 155) && (timestamps_seconds > 50)) {
			system_info.holt_mode += 1;
			timestamps_start = millis(); // update timestamps
		}
	} else if (system_info.holt_mode == HOT_MODE_KEEP_WARM) {
		timestamps_seconds = (millis() - timestamps_start) / 1000;
		system_info.target_temputer = 160;
		if (timestamps_seconds > 90) {
			system_info.holt_mode += 1;
			timestamps_start = millis();
		}
	} else if (system_info.holt_mode == HOT_MODE_WELD) {
		timestamps_seconds = (millis() - timestamps_start) / 1000;
		system_info.target_temputer += 2.5f;
		if (system_info.target_temputer > 235) {system_info.target_temputer = 235;}
		if ((system_info.temputer > 230) && (timestamps_seconds > 30)) {
			system_info.holt_mode += 1;
			timestamps_start = millis(); // update timestamps
		}
	} else if (system_info.holt_mode == HOT_MODE_COLD) {
		timestamps_seconds = (millis() - timestamps_start) / 1000;
		system_info.target_temputer = 60;
		if ((system_info.temputer <= 60) && (timestamps_seconds > 60)) {
			g_weld_running = false;
			system_info.holt_mode = HOT_MODE_STANDBY;
		}
	} else {
		// error
		system_info.target_temputer = 20;
		Serial.printf("mode error %d", system_info.holt_mode);
	}

		if (system_info.holt_mode >= HOT_MODE_MAX) {
			system_info.holt_mode = HOT_MODE_STANDBY;
		}

	/* 画点 */
	if (g_weld_running == true) {
		g_weld_timestamp = (millis() - g_weld_timestamp_start) / 1000;
		if ((g_weld_timestamp % 1) == 0) {
			post_x = g_weld_timestamp / 1;
			post_y = (int32_t)(170.0f - ((float)(system_info.temputer) / 2.5f));
			if (post_x >= 240) {post_x = 240;}
			if (post_y <= 50)  {post_y = 50;}
			tft.drawPixel(post_x, post_y, TFT_WHITE, 0xFF, TFT_BLACK);
		}
	}

	//系统误差
	value = (double)system_info.target_temputer -  (double)system_info.temputer;
	last_value = value;
	error_Temp += value * dt;
	if (error_Temp > 500) {error_Temp = 500;}
	if (error_Temp < 0.0) {error_Temp = 0.0;}
	Output_Heat = Kp * value + /* Ki * error_Temp */ + Kd * (value - last_value);
	// Serial.printf("pwm_out: %f", Output_Heat);
	if (Output_Heat > 100) {Output_Heat = 100;}
	if (Output_Heat < 0.0) {Output_Heat = 0.001;}
	system_info.pwm_precent = (int32_t)Output_Heat;
	// Serial.printf("system_info.pwm_precent: %f \r\n", system_info.pwm_precent);

	/* 电压 */
	if (system_info.last_ina226.voltage != system_info.ina226.voltage) {
		tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
		tft.setTextDatum(TR_DATUM);/* 右上 */
		tft.loadFont(fzchsjt_24);
		sprintf(&buf[0], " ");
		if ((int32_t)system_info.ina226.voltage > 9) {
			sprintf(&buf[1], "%01d", (int32_t)(system_info.ina226.voltage)/10%10);
			sprintf(&buf[2], "%01d", (int32_t)(system_info.ina226.voltage)%10);
		} else {
			sprintf(&buf[1], " ");
			sprintf(&buf[2], "%01d", (int32_t)(system_info.ina226.voltage)%10);
		}
		sprintf(&buf[3], ".");
		sprintf(&buf[4], "%01d", (int32_t)(system_info.ina226.voltage*10.0)%10);
		buf[5] = 'V';
		buf[6] = '\0';
		tft.drawString(buf, 126, 183);
		tft.unloadFont();
	}

	/* 功率 */
	if (system_info.last_ina226.power != system_info.ina226.power) {
		tft.setTextColor(TFT_VIOLET, TFT_BLACK, true);
		tft.setTextDatum(TR_DATUM);/* 右上 */
		tft.loadFont(fzchsjt_24);
		sprintf(&buf[0], " ");
		if ((int32_t)system_info.ina226.power > 99) {
			sprintf(&buf[1], "%01d", (int32_t)(system_info.ina226.power)/100%10);
			sprintf(&buf[2], "%01d", (int32_t)(system_info.ina226.power)/10%10);
			sprintf(&buf[3], "%01d", (int32_t)(system_info.ina226.power)%10);
		} else if ((int32_t)system_info.ina226.power > 9) {
			sprintf(&buf[1], "0");
			sprintf(&buf[2], "%01d", (int32_t)(system_info.ina226.power)/10%10);
			sprintf(&buf[3], "%01d", (int32_t)(system_info.ina226.power)%10);
		} else {
			sprintf(&buf[1], "0");
			sprintf(&buf[2], "0");
			sprintf(&buf[3], "%01d", (int32_t)(system_info.ina226.power)%10);
		}
		buf[4] = 'W';
		buf[5] = '\0';
		tft.drawString(buf, 126, 183 + 30);
		tft.unloadFont();
	}

	/* 温度 */
	if (system_info.last_temputer != system_info.temputer) {
		tft.setTextColor(TFT_CYAN, TFT_BLACK, true);
		tft.setTextDatum(TR_DATUM);/* 右上 */
		tft.loadFont(fzchsjt_24);
		sprintf(&buf[0], " ");
		if ((int32_t)system_info.temputer > 99) {
			sprintf(&buf[1], "%01d", (int32_t)(system_info.temputer)/100%10);
			sprintf(&buf[2], "%01d", (int32_t)(system_info.temputer)/10%10);
			sprintf(&buf[3], "%01d", (int32_t)(system_info.temputer)%10);
		} else if ((int32_t)system_info.temputer > 9) {
			sprintf(&buf[1], " ");
			sprintf(&buf[2], "%01d", (int32_t)(system_info.temputer)/10%10);
			sprintf(&buf[3], "%01d", (int32_t)(system_info.temputer)%10);
		} else {
			sprintf(&buf[1], " ");
			sprintf(&buf[2], " ");
			sprintf(&buf[3], "%01d", (int32_t)(system_info.temputer)%10);
		}
		// sprintf(&buf[4], "℃");
		tft.drawString(buf, 236, 183);
		tft.unloadFont();
	}

	/* 状态 */
	if (system_info.last_holt_mode != system_info.holt_mode) {
		tft.loadFont(hwkt_24);
		if (system_info.holt_mode == HOT_MODE_STANDBY) {
			tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
			tft.setCursor(190, 183 + 30);	tft.println("待机");
		} else if (system_info.holt_mode == HOT_MODE_PRE_HEAT) {
			tft.setTextColor(TFT_YELLOW, TFT_BLACK, true);
			tft.setCursor(190, 183 + 30);	tft.println("预热");
		} else if (system_info.holt_mode == HOT_MODE_KEEP_WARM) {
			tft.setTextColor(TFT_ORANGE, TFT_BLACK, true);
			tft.setCursor(190, 183 + 30);	tft.println("恒温");
		} else if (system_info.holt_mode == HOT_MODE_WELD) {
			tft.setTextColor(TFT_RED, TFT_BLACK, true);
			tft.setCursor(190, 183 + 30);	tft.println("回流");
		} else if (system_info.holt_mode == HOT_MODE_COLD) {
			tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK, true);
			tft.setCursor(190, 183 + 30);	tft.println("降温");
		} else {
			tft.setTextColor(TFT_MAGENTA, TFT_BLACK, true);
			tft.setCursor(190, 183 + 30);	tft.println("错误");
		}
		tft.unloadFont();
	}


	/* 0.0 - 1.0 */
	update_pwm_out(system_info.pwm_precent/100.0f);

	/* 打印状态 */
	one_sec_count += 1;
	if (((one_sec_count % 50) == 0) && (system_info.holt_mode != HOT_MODE_STANDBY)) {
		Serial.printf("x: %d  y: %d diff:%f pwm_pre:%f \r\n", post_x, post_y, value, system_info.pwm_precent);
		Serial.printf("current mode: %d g_weld_timestamp:%d second: %d target_temp: %d \r\n",
																				system_info.holt_mode,
																				g_weld_timestamp,
																				timestamps_seconds, 
																				system_info.target_temputer);
	}

	system_info.last_temputer = system_info.temputer;
    system_info.last_target_temputer = system_info.target_temputer;
	system_info.last_ina226.voltage = system_info.ina226.voltage;
	system_info.last_ina226.current = system_info.ina226.current;
	system_info.last_ina226.power = system_info.ina226.power;
	system_info.last_encoder = system_info.encoder;
	system_info.last_pwm_precent = system_info.pwm_precent;
	system_info.last_holt_mode = system_info.holt_mode;
}

/* 10Hz */
void Task_Data_Callback()
{
	// Serial.printf("millis:%d \r\n", millis());

	/* MAX6675 data rate 250ms */
	update_temputer_sensor();

	update_power_sensor();
}

/* 50Hz */
void Task_GUI_Callback()
{
	static uint32_t gui_page_select = 2;
	static uint32_t gui_page_current = 0;

	if (gui_page_select == 1) {
		gui_thermost_mode_init();
		gui_page_select = 0;
		gui_page_current = 1;
	} else if (gui_page_select == 2) {
		gui_reflow_solder_mode_init();
		gui_page_select = 0;
		gui_page_current = 2;
	}

	if (gui_page_current == 1) {
		gui_thermost_mode_refresh();
	} else if (gui_page_current == 2) {
		gui_reflow_solder_mode_refresh();
	}
}



void loop()
{
	runner.execute();
}
