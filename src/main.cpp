#include <Arduino.h>
#include <TaskScheduler.h>
#include "system_init.h"

#include "font/hwkt_hz24.h"
#include "font/hwkt_hz32.h"
#include "font/hwkt_hz48.h"
#include "font/fzchsjt_zm24.h"
#include "font/fzchsjt_zm64.h"

/* 系统参数*/
system_info_t system_info;
system_info_t last_system_info;

/* OS */
Scheduler runner;
void Task_Data_Callback();
void Task_GUI_Callback();
Task task_data(DATA_TASK_CYCLE, TASK_FOREVER, &Task_Data_Callback);	// 100ms
Task task_gui(UI_TASK_CYCLE, TASK_FOREVER, &Task_GUI_Callback);		// 20ms

/* ui 界面参数 */
uint32_t gui_page_select = 0;
uint32_t gui_page_current = 0;
bool main_page_need_init = true;	// 菜单页初始化标志

/* 回流焊参数*/
solder_parameter_t solder_parameter[HOT_MODE_MAX] = {
	{20.0,  30.0,  40.0,  0.0, 0.0, 0.0, 0, 0xFFFFFFFF},	/* HOT_MODE_STANDBY */
	{130.0, 140.0, 150.0, 1.0, 1.5, 3.0, 0, 90},			/* HOT_MODE_PRE_HEAT */
	{130.0, 160.0, 150.0, 0.0, 0.4, 1.0, 0, 60},			/* HOT_MODE_KEEP_WARM */
	{205.0, 225.0, 245.0, 1.0, 2.0, 3.0, 0, 45},			/* HOT_MODE_WELD */
	{0.0,    50.0,  60.0, 0.0, 0.0, 0.0, 0, 0xFFFFFFFF},	/* HOT_MODE_COLD */
};

bool g_solder_is_running = false;
unsigned long g_solder_timer_start, g_solder_time;

/* PID */
//Define Variables we'll be connecting to
double Setpoint = 0, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 4.5, Ki = 0.000, Kd = 0.5;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

/* 系统初始化 */
void setup()
{
	Serial.begin(115200);

	parameter_init();

	pwm_init();

	encoder_init();

	max6675_init();

	ina226_init();

	st7789_init();

	/* pid */
	myPID.SetMode(AUTOMATIC); 	//turn the PID on

	/* START TASK SCHEDULE */
	runner.init();

	runner.addTask(task_data);
	runner.addTask(task_gui);

	task_data.enable();
	task_gui.enable();
}

/* 恒温模式 初始化 */
void gui_thermost_mode_init(void)
{
	parameter_init();

	/* 必要参数初始化 */
	system_info.target_temputer = solder_parameter[system_info.holt_mode].target_temp;

    tft.fillScreen(TFT_BLACK);

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

/* 恒温模式 页面刷新 */
void gui_thermost_mode_refresh(void)
{
	/* PID 参数 */
	double Output_Heat = 0;
	/* 界面参数 */
	char buf[50];
	static bool long_press_flag = false;
	static unsigned long key_press_timestamp;
	static uint32_t one_sec_count = 0; // 周期串口打印计数值
	ButtonState button_status;
	button_status = update_button_status();
	//Serial.printf("[Encoder Key] value: %d \r\n", status);

	/* 旋转编码器 按钮 处理 */
	if ((button_status == BUT_PUSHED) && (long_press_flag == false)) {
		long_press_flag = true;
		key_press_timestamp = millis();
	} else if ((button_status == BUT_DOWN) && (long_press_flag == true)) {
		if ((millis() - key_press_timestamp) > LONG_PRESS_RETURN_TIME) {
			/* 长按退出 */
			gui_page_select = 0;
			gui_page_current = 0;
			main_page_need_init = true;		// 刷新主页
			system_info.pwm_precent = 0;	// 关停输出
			return;
		}
	}  else if (button_status == BUT_RELEASED) {
		/* 短按 */
	} else {
		long_press_flag = false;
	}

	/* 旋转编码器 处理 */
	update_encoder_value();
	if (last_system_info.encoder != system_info.encoder) {
		if ((last_system_info.encoder == 0) && (system_info.encoder == 1000)) {
			// nothing
		} else if ((last_system_info.encoder == 1000) && (system_info.encoder == 0)) {
			system_info.target_temputer += 1;
		} else {
			if (last_system_info.encoder < system_info.encoder) {
				system_info.target_temputer += 1;
			} else {
				system_info.target_temputer -= 1;
			}
		}

		if (system_info.target_temputer > 230) {
			system_info.target_temputer = 230;
		} else if (system_info.target_temputer < 20) {
			system_info.target_temputer = 20;
		}
	}

	/* 温控部分 PID */
	Input = (double)system_info.temputer;
	Setpoint = (double)system_info.target_temputer;
	myPID.Compute();
	Output_Heat = Output;
	if (Output_Heat > 100.0f) { Output_Heat = 100.0f; }
	if (Output_Heat < 0.0f) { Output_Heat = 0.0f; }
	system_info.pwm_precent = Output_Heat;

	/* 当前温度 */
	if (last_system_info.temputer != system_info.temputer) {
		tft.setTextColor(TFT_CYAN, TFT_BLACK, true);
		tft.setTextDatum(TL_DATUM);/* 左上 */
		tft.loadFont(fzchsjt_64);
		sprintf(buf, "%03d℃", (int32_t)system_info.temputer);
		tft.drawString(buf, 62, 52);
		tft.unloadFont();
	}

	/* 设定温度*/
	if (last_system_info.target_temputer != system_info.target_temputer) {
		tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK, true);
		tft.setTextDatum(TL_DATUM);/* 左上 */
		tft.loadFont(fzchsjt_64);
		sprintf(buf, "%03d℃", (int32_t)system_info.target_temputer);
		tft.drawString(buf, 62, 124);
		tft.unloadFont();
	}

	/* 电压 */
	if (last_system_info.ina226.voltage != system_info.ina226.voltage) {
		tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
		tft.setTextDatum(TR_DATUM);/* 右上 */
		tft.loadFont(fzchsjt_24);
		sprintf(&buf[0], " ");
		if ((int32_t)system_info.ina226.voltage > 9) {
			sprintf(&buf[1], "%01d", (int32_t)system_info.ina226.voltage / 10 % 10);
		} else {
			sprintf(&buf[1], " ");
		}
		sprintf(&buf[2], "%01d", (int32_t)(system_info.ina226.voltage) % 10);
		sprintf(&buf[3], ".");
		sprintf(&buf[4], "%01d", (int32_t)(system_info.ina226.voltage * 10.0) % 10);
		buf[5] = 'V';
		buf[6] = '\0';
		tft.drawString(buf, 120, 186);
		tft.unloadFont();
	}

	/* 电流 */
	if (last_system_info.ina226.current != system_info.ina226.current) {
		tft.setTextColor(TFT_SKYBLUE, TFT_BLACK, true);
		tft.setTextDatum(TR_DATUM);/* 右上 */
		tft.loadFont(fzchsjt_24);
		sprintf(&buf[0], " ");
		if ((int32_t)system_info.ina226.current > 9) {
			sprintf(&buf[1], "%01d", (int32_t)system_info.ina226.current / 10 % 10);
		} else {
			sprintf(&buf[1], " ");
		}
		sprintf(&buf[2], "%01d", (int32_t)system_info.ina226.current % 10);
		sprintf(&buf[3], ".");
		sprintf(&buf[4], "%01d", (int32_t)(system_info.ina226.current * 10.0) % 10);
		buf[5] = 'A';
		buf[6] = '\0';
		tft.drawString(buf, 120, 214);
		tft.unloadFont();
	}

	/* 功率 */
	if (last_system_info.ina226.power != system_info.ina226.power) {

		tft.setTextColor(TFT_MAGENTA, TFT_BLACK, true);
		tft.setTextDatum(TR_DATUM);/* 右上 */
		tft.loadFont(fzchsjt_24);
		sprintf(&buf[0], " ");
		if ((int32_t)system_info.ina226.power > 99) {
			sprintf(&buf[1], "1");
			sprintf(&buf[2], "0");
			sprintf(&buf[3], "0");
		} else if ((int32_t)system_info.ina226.power > 9) {
			sprintf(&buf[1], " ");
			sprintf(&buf[2], "%01d", (int32_t)system_info.ina226.power / 10 % 10);
			sprintf(&buf[3], "%01d", (int32_t)system_info.ina226.power % 10);
		} else {
			sprintf(&buf[1], " ");
			sprintf(&buf[2], " ");
			sprintf(&buf[3], "%01d", (int32_t)system_info.ina226.power % 10);
		}
		buf[4] = 'W';
		buf[5] = '\0';
		tft.drawString(buf, 240, 186);
		tft.unloadFont();
	
		// tft.setTextColor(TFT_MAGENTA, TFT_BLACK, true);
		// tft.setTextDatum(TR_DATUM);/* 右上 */
		// tft.loadFont(fzchsjt_24);
		// sprintf(&buf[0], "%03dW", ((int32_t)system_info.ina226.power));
		// tft.drawString(buf, 240, 186);
		// tft.unloadFont();
	}

	/* 百分比 */
	if (last_system_info.pwm_precent != system_info.pwm_precent) {
		tft.setTextColor(TFT_RED, TFT_BLACK, true);
		tft.setTextDatum(TR_DATUM);/* 右上 */
		tft.loadFont(fzchsjt_24);
		sprintf(&buf[0], " ");
		if ((int32_t)system_info.pwm_precent > 99) {
			sprintf(&buf[1], "1");
			sprintf(&buf[2], "0");
			sprintf(&buf[3], "0");
		} else if ((int32_t)system_info.pwm_precent > 9) {
			sprintf(&buf[1], " ");
			sprintf(&buf[2], "%01d", (int32_t)system_info.pwm_precent / 10 % 10);
			sprintf(&buf[3], "%01d", (int32_t)system_info.pwm_precent % 10);
		} else {
			sprintf(&buf[1], " ");
			sprintf(&buf[2], " ");
			sprintf(&buf[3], "%01d", (int32_t)system_info.pwm_precent % 10);
		}
		buf[4] = '%';
		buf[5] = '\0';
		tft.drawString(buf, 240, 214);
		tft.unloadFont();
	}

	/* 打印状态 */
	if (((one_sec_count++ % 50) == 0) && (system_info.holt_mode != HOT_MODE_STANDBY)) {
		Serial.printf("PID origin_out:%f  pwm_out:%f \r\n", Output, Output_Heat);
	}

	memcpy(&last_system_info, &system_info, sizeof(system_info_t));
}

/* 回流焊模式 初始化*/
void gui_reflow_solder_mode_init(void)
{
	uint32_t sx, sy, ex, ey, len;

	parameter_init();

	/* 必要参数初始化 */
	system_info.holt_mode = HOT_MODE_STANDBY;
	system_info.target_temputer = solder_parameter[system_info.holt_mode].target_temp;

    tft.fillScreen(TFT_BLACK);
	tft.setTextDatum(TL_DATUM); /* 左上 */
	tft.loadFont(hwkt_32);
	tft.setTextColor(TFT_GOLD, TFT_BLACK, true);
	tft.setCursor(24, 4);
	tft.println("回 流 焊 模 式");
	tft.unloadFont();

	/* 分界线 */
	tft.drawFastHLine(0, 180 - 6, 240, TFT_DARKGREY);
	tft.drawFastHLine(0, 180 - 5, 240, TFT_DARKGREY);
	tft.drawFastHLine(0, 180 - 4, 240, TFT_DARKGREY);

	/* 参数 */
	tft.loadFont(hwkt_24);
	tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
	tft.setCursor(1, 183);	tft.println("电压");
	tft.setTextColor(TFT_VIOLET, TFT_BLACK, true);
	tft.setCursor(1, 183 + 30);	tft.println("功率");

	tft.setTextColor(TFT_CYAN, TFT_BLACK, true);
	tft.setCursor(130 + 1, 183);	tft.println("温度");
	tft.setTextColor(TFT_ORANGE, TFT_BLACK, true);
	tft.setCursor(130 + 1, 183 + 30);	tft.println("状态");
	tft.unloadFont();
}

/* 回流焊模式 刷新页面 20ms */  
void gui_reflow_solder_mode_refresh(void)
{
	/* PID 参数 */
	static double Output_Heat = 0;
	/* 必须参数 */
	char str_buffer[50];
	static uint32_t one_sec_count = 0;		// 周期串口打印计数值
	static unsigned long split_time = 0; 	// 阶段运行时间
	int64_t post_x, post_y;					// 描点坐标
	/* 编码器按键处理 */
	static bool long_press_flag = false;
	static unsigned long key_press_timestamp;
	ButtonState button_status;
	button_status = update_button_status();

	/* 旋转编码器 按钮 处理 */
	if ((button_status == BUT_PUSHED) && (long_press_flag == false)) {
		long_press_flag = true;
		key_press_timestamp = millis();
	} else if ((button_status == BUT_DOWN) && (long_press_flag == true)) {
		if ((millis() - key_press_timestamp) > LONG_PRESS_RETURN_TIME) {
			/* 长按退出 */
			gui_page_select = 0;
			gui_page_current = 0;
			main_page_need_init = true;
			g_solder_is_running = false;
			return;
		}
	} else if (button_status == BUT_RELEASED) {
		/* 短按启动 */
		if ((g_solder_is_running == false) && (system_info.holt_mode == HOT_MODE_STANDBY) && (system_info.temputer < 50.0)) {
			system_info.holt_mode = HOT_MODE_PRE_HEAT;
			system_info.target_temputer = system_info.temputer;
			solder_parameter[system_info.holt_mode].timer_start = millis();
			g_solder_timer_start = millis();
			g_solder_is_running = true;
			tft.fillRect(0, 50, 240, 120, TFT_BLACK);
		} 
		//TODO: 增加无法启动提示
	} else {
		long_press_flag = false;
	}

	/* 回流焊过程 */
	if (system_info.holt_mode == HOT_MODE_STANDBY) {
		system_info.target_temputer = solder_parameter[system_info.holt_mode].target_temp;
	} else {
		/* 阶段运行时间 单位/秒 */
		split_time = (millis() - solder_parameter[system_info.holt_mode].timer_start) / 1000;
		/* Adjust target temperature, temperature rise */
		system_info.target_temputer += (solder_parameter[system_info.holt_mode].heat_rate * 0.02f);
		/* limit the maximum temperature */
		if (system_info.target_temputer > solder_parameter[system_info.holt_mode].target_temp) {
			system_info.target_temputer = solder_parameter[system_info.holt_mode].target_temp;
		}
		/* The interval is over, ready to move on to the next stage */
		if (system_info.holt_mode == HOT_MODE_COLD) {
			if (system_info.temputer < (solder_parameter[system_info.holt_mode].target_temp)) {
				/* next mode */
				system_info.holt_mode = HOT_MODE_STANDBY;
			}
		} else if ( (system_info.temputer > (solder_parameter[system_info.holt_mode].target_temp - 5.0)) 
					&& (split_time > solder_parameter[system_info.holt_mode].heat_sec)) {
			/* next mode */
			system_info.holt_mode += 1;
			/* update timestamps */
			solder_parameter[system_info.holt_mode].timer_start = millis(); 
		}
	}

	if (system_info.holt_mode >= HOT_MODE_MAX) {
		system_info.holt_mode = HOT_MODE_STANDBY;
		g_solder_is_running = false;
	}

	/* 系统误差 */
	Input = (double)system_info.temputer;
	Setpoint = (double)system_info.target_temputer;
	myPID.Compute();
	Output_Heat = Output;
	if (Output_Heat > 100.0f) { Output_Heat = 100.0f; }
	if (Output_Heat < 0.0f) { Output_Heat = 0.0f; }
	system_info.pwm_precent = Output_Heat;

	/* 温度曲线 */
	if (g_solder_is_running == true) {
		g_solder_time = (millis() - g_solder_timer_start) / 1000;
		if (((g_solder_time * 10) % 15) == 0) {
			post_x = (g_solder_time * 10) / 15;
			post_y = (int32_t)(170.0f - ((float)(system_info.temputer) / 2.0f));
			if (post_x > (TFT_WIDTH - 1)) {post_x = (TFT_WIDTH - 1);}
			if (post_y <= 50)  {post_y = 50;}
			tft.drawPixel(post_x, post_y, TFT_WHITE, 0xFF, TFT_BLACK);
		}
	}

	/* 电压 */
	if (last_system_info.ina226.voltage != system_info.ina226.voltage) {
		tft.setTextColor(TFT_GREEN, TFT_BLACK, true);
		tft.setTextDatum(TR_DATUM);/* 右上 */
		tft.loadFont(fzchsjt_24);
		sprintf(&str_buffer[0], " ");
		if ((int32_t)system_info.ina226.voltage > 9) {
			sprintf(&str_buffer[1], "%01d", (int32_t)(system_info.ina226.voltage)/10%10);
			sprintf(&str_buffer[2], "%01d", (int32_t)(system_info.ina226.voltage)%10);
		} else {
			sprintf(&str_buffer[1], " ");
			sprintf(&str_buffer[2], "%01d", (int32_t)(system_info.ina226.voltage)%10);
		}
		sprintf(&str_buffer[3], ".");
		sprintf(&str_buffer[4], "%01d", (int32_t)(system_info.ina226.voltage*10.0)%10);
		str_buffer[5] = 'V';
		str_buffer[6] = '\0';
		tft.drawString(str_buffer, 126, 183);
		tft.unloadFont();
	}

	/* 功率 */
	if (last_system_info.ina226.power != system_info.ina226.power) {
		tft.setTextColor(TFT_VIOLET, TFT_BLACK, true);
		tft.setTextDatum(TR_DATUM);/* 右上 */
		tft.loadFont(fzchsjt_24);
		sprintf(&str_buffer[0], " ");
		if ((int32_t)system_info.ina226.power > 99) {
			sprintf(&str_buffer[1], "%01d", (int32_t)(system_info.ina226.power)/100%10);
			sprintf(&str_buffer[2], "%01d", (int32_t)(system_info.ina226.power)/10%10);
			sprintf(&str_buffer[3], "%01d", (int32_t)(system_info.ina226.power)%10);
		} else if ((int32_t)system_info.ina226.power > 9) {
			sprintf(&str_buffer[1], "0");
			sprintf(&str_buffer[2], "%01d", (int32_t)(system_info.ina226.power)/10%10);
			sprintf(&str_buffer[3], "%01d", (int32_t)(system_info.ina226.power)%10);
		} else {
			sprintf(&str_buffer[1], "0");
			sprintf(&str_buffer[2], "0");
			sprintf(&str_buffer[3], "%01d", (int32_t)(system_info.ina226.power)%10);
		}
		str_buffer[4] = 'W';
		str_buffer[5] = '\0';
		tft.drawString(str_buffer, 126, 183 + 30);
		tft.unloadFont();
	}

	/* 温度 */
	if (last_system_info.temputer != system_info.temputer) {
		tft.setTextColor(TFT_CYAN, TFT_BLACK, true);
		tft.setTextDatum(TR_DATUM);/* 右上 */
		tft.loadFont(fzchsjt_24);
		sprintf(&str_buffer[0], " ");
		if ((int32_t)system_info.temputer > 99) {
			sprintf(&str_buffer[1], "%01d", (int32_t)(system_info.temputer)/100%10);
			sprintf(&str_buffer[2], "%01d", (int32_t)(system_info.temputer)/10%10);
			sprintf(&str_buffer[3], "%01d", (int32_t)(system_info.temputer)%10);
		} else if ((int32_t)system_info.temputer > 9) {
			sprintf(&str_buffer[1], " ");
			sprintf(&str_buffer[2], "%01d", (int32_t)(system_info.temputer)/10%10);
			sprintf(&str_buffer[3], "%01d", (int32_t)(system_info.temputer)%10);
		} else {
			sprintf(&str_buffer[1], " ");
			sprintf(&str_buffer[2], " ");
			sprintf(&str_buffer[3], "%01d", (int32_t)(system_info.temputer)%10);
		}
		// sprintf(&str_buffer[4], "℃");
		tft.drawString(str_buffer, 236, 183);
		tft.unloadFont();
	}

	/* 状态 */
	if (last_system_info.holt_mode != system_info.holt_mode) {
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

	/* 打印状态 */
	if (((one_sec_count++ % 50) == 0) && (system_info.holt_mode != HOT_MODE_STANDBY)) {
		Serial.printf("------ 回流焊模式 ------\r\n");
		Serial.printf("[SOLDER] x: %d  y: %d \r\n", post_x, post_y);
		Serial.printf("[SOLDER] solder_mode[%d] solder_time[%d] target_temp[%f] \r\n", system_info.holt_mode, g_solder_time, system_info.target_temputer);
		Serial.printf("[SOLDER] PID origin_out:%f  pwm_out:%f \r\n", Output, Output_Heat);
		Serial.printf("\r\n");
	}

	memcpy(&last_system_info, &system_info, sizeof(system_info_t));
}

/* 10Hz */
void Task_Data_Callback()
{
	static uint32_t count = 0;

	/* MAX6675 data rate 250ms */
	if (((count++) % 4) == 0) {
		update_temputer_sensor();
	}

	update_power_sensor();
}

void gui_main_select_page()
{
	static uint32_t last_point_page_number = (GUI_MAX_INDEX - 1);
	static uint32_t point_page_number = GUI_MIN_INDEX;
	/* 编码器按键临时变量 */
	static bool press_flag = false;
	static unsigned long key_press_timestamp;
	ButtonState button_status;

	/* 记录页面 */
	last_system_info.encoder = system_info.encoder;
	last_point_page_number = point_page_number;

	/* 更新按键状态 */
	button_status = update_button_status();

	/* 旋转编码器 按钮 处理 */
	if ((button_status == BUT_PUSHED) && (press_flag == false)) {
		press_flag = true;
	} else if ((button_status == BUT_RELEASED) && (press_flag == true)) {
		press_flag = false;
		gui_page_select = point_page_number;
	}

	/* 旋转编码器 旋钮 处理 */
	update_encoder_value();
	if (last_system_info.encoder != system_info.encoder) {
		if ((last_system_info.encoder == 0) && (system_info.encoder == 1000)) {
			// nothing
		} else if ((last_system_info.encoder == 1000) && (system_info.encoder == 0)) {
			point_page_number -= 1;
		} else {
			if (last_system_info.encoder < system_info.encoder) {
				point_page_number -= 1;
			} else {
				point_page_number += 1;
			}
		}
	}

	/* 选择光标限幅 */
	if (point_page_number >= GUI_MAX_INDEX) {
		point_page_number = GUI_MIN_INDEX;
	} else if (point_page_number < GUI_MIN_INDEX) {
		point_page_number = GUI_MAX_INDEX - 1;
	}

	if (main_page_need_init == true) {
		main_page_need_init = false; /* reset init flag */
		tft.fillScreen(TFT_BLACK);   /* main page draw */
		tft.setTextDatum(CC_DATUM);
		tft.loadFont(hwkt_48);
		tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
		tft.setCursor(50, 6);
		tft.println("菜");
		tft.setCursor(130, 6);
		tft.println("单");
		tft.unloadFont();

		tft.loadFont(hwkt_32);
		tft.setTextColor(TFT_WHITE, TFT_BLACK, true);

		tft.setTextColor(TFT_BLACK, TFT_BLACK, true);
		tft.setCursor(5, 70 + (last_point_page_number - GUI_MIN_INDEX) * 40);
		tft.println("    ");

		tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
		tft.setCursor(5, 70 + (point_page_number - GUI_MIN_INDEX) * 40);
		tft.println("●");

		tft.setCursor(40, 70);   tft.println("恒 温 模 式");

		tft.setCursor(40, 110);  tft.println("回 流 焊 模 式");

		tft.setCursor(40, 150);  tft.println("参 数 设 置");

		tft.setCursor(40, 190);  tft.println("系 统 设 置");
		tft.unloadFont();
	}

	if (last_point_page_number != point_page_number) {
		Serial.printf("last page: %d  page: %d\r\n", last_point_page_number, point_page_number);

		tft.loadFont(hwkt_32);

		tft.setTextColor(TFT_BLACK, TFT_BLACK, true);
		tft.setCursor(5, 70 + (last_point_page_number - GUI_MIN_INDEX) * 40);
		tft.println("    ");

		tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
		tft.setCursor(5, 70 + (point_page_number - GUI_MIN_INDEX) * 40);
		tft.println("●");

		tft.unloadFont();
	}

	if (gui_page_select == GUI_THREAD_MODE_INDEX) {
		gui_thermost_mode_init();
		gui_page_select = 0;
		gui_page_current = 1;
	} else if (gui_page_select == GUI_REFLOW_SOLDER_INDEX) {
		gui_reflow_solder_mode_init();
		gui_page_select = 0;
		gui_page_current = 2;
	} else if (gui_page_select == GUI_PARAMETER_CONFIG_INDEX) {
		;
	} else if (gui_page_select == GUI_SYSTEM_CONFIG_INDEX) {
		;
	}
}

/* 50Hz */
void Task_GUI_Callback()
{
	if (gui_page_current == 0) {
		gui_main_select_page();
	} else if (gui_page_current == 1) {
		gui_thermost_mode_refresh();
	} else if (gui_page_current == 2) {
		gui_reflow_solder_mode_refresh();
	}

	/* 0.0 - 1.0 */
	update_pwm_out(system_info.pwm_precent / 100.0f);
}



void loop()
{
	runner.execute();
}
