#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_handle.hpp"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_dsp.h"

#include "esp_log.h"
#include "esp32_pinout.h"

// Debug configs ------------------------------------------------------------------------------------------------------
// #define MEASURE_ADC_FREQ

// Hardware -----------------------------------------------------------------------------------------------------------

// Geekworm Easy Kit ESP32-C1
#define LED_BUILTIN_RIGHT (gpio_num_t) GPIO0
#define LED_BUILTIN_LEFT (gpio_num_t) GPIO27
#define BTN_USER (gpio_num_t) GPIO0
#define BTN_PRESSED_LEVEL 0

#define AUDIO_ADC_CH ADC_CHANNEL_6
#define AUDIO_ADC_UNIT ADC_UNIT_1
#define AUDIO_ADC_ATTEN ADC_ATTEN_DB_6

// Application configs ------------------------------------------------------------------------------------------------
#define STORAGE_NAMESPACE "ESPViz"
#define ESP_INTR_FLAG_DEFAULT 0
#define DEBOUNCE_MS 100
#define BTN_LONG_PRESS_MS 2000
#define BTN_LONG_LONG_PRESS_MS 15000

// Inline Helpers -----------------------------------------------------------------------------------------------------
#define delay(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#define millis() esp_timer_get_time() / 1000ULL;
#define micros() esp_timer_get_time();

// Global variables ---------------------------------------------------------------------------------------------------
static const char *APP_NAME = "ESPViz";
esp_err_t err;
static uint8_t s_led_state = 0;
TaskHandle_t task1Handler = NULL;
TaskHandle_t task2Handler = NULL;
TaskHandle_t taskADCHandler = NULL;
TaskHandle_t taskFFTHandler = NULL;
TaskHandle_t taskBtnISRHandler = NULL;
static xQueueHandle btn_evt_queue = nullptr;
std::shared_ptr<nvs::NVSHandle> storageHandler;

// LEDs ---------------------------------------------------------------------------------------------------------------
void setLED() {
	gpio_pad_select_gpio(LED_BUILTIN_LEFT);
	gpio_set_direction(LED_BUILTIN_LEFT, GPIO_MODE_OUTPUT);
	gpio_set_level(LED_BUILTIN_LEFT, s_led_state);
}

static void toggle_led() {
	s_led_state = !s_led_state;
	ESP_LOGI(APP_NAME, "LED: %s", s_led_state == true ? "ON" : "OFF");
	gpio_set_level(LED_BUILTIN_LEFT, s_led_state);
}

// Storage ------------------------------------------------------------------------------------------------------------
static void initStorage();

static void resetStorage() {
	ESP_LOGI(APP_NAME, "Resetting...");
	ESP_ERROR_CHECK(nvs_flash_erase());

	initStorage();

	// Write
	ESP_LOGI(APP_NAME, "Updating restart counter in NVS ... ");
	err = storageHandler->set_item("restart_counter", 0);
	ESP_LOGI(APP_NAME, "%s", (err != ESP_OK) ? "Failed!" : "Done");

	// Commit written value.
	// After setting any values, nvs_commit() must be called to ensure changes are written
	// to flash storage. Implementations may write to storage at other times,
	// but this is not guaranteed.
	ESP_LOGI(APP_NAME, "Committing updates in NVS...");
	err = storageHandler->commit();
	ESP_LOGI(APP_NAME, "%s", (err != ESP_OK) ? "Failed!" : "Done");

	ESP_LOGI(APP_NAME, "Restarting now...");
	esp_restart();
}

static void initStorage() {
	// Initialize NVS
	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		resetStorage();
	}
	ESP_ERROR_CHECK(err);

	// Open
	ESP_LOGI(APP_NAME, "Opening Non-Volatile Storage (NVS) handle... ");
	esp_err_t result;
	// Handle will automatically close when going out of scope or when it's reset.
	storageHandler = nvs::open_nvs_handle(STORAGE_NAMESPACE, NVS_READWRITE, &result);
	if (err != ESP_OK) {
		ESP_LOGI(APP_NAME, "Error (%s) opening NVS handle!", esp_err_to_name(err));
	}
	ESP_LOGI(APP_NAME, "Done");
}

static void increaseRestartCounterStorage() {
	// Read
	ESP_LOGI(APP_NAME, "Reading restart counter from NVS...");
	int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
	err = storageHandler->get_item("restart_counter", restart_counter);
	if (err != ESP_OK) {
		ESP_LOGI(APP_NAME, "Error (%s) reading!", esp_err_to_name(err));
		resetStorage();
	}
	ESP_LOGI(APP_NAME, "Done");
	ESP_LOGI(APP_NAME, "Restart counter = %d", restart_counter);

	// Write
	restart_counter++;
	ESP_LOGI(APP_NAME, "Updating restart counter in NVS ... ");
	err = storageHandler->set_item("restart_counter", restart_counter);
	ESP_LOGI(APP_NAME, "%s", (err != ESP_OK) ? "Failed!" : "Done");

	// Commit written value.
	// After setting any values, nvs_commit() must be called to ensure changes are written
	// to flash storage. Implementations may write to storage at other times,
	// but this is not guaranteed.
	ESP_LOGI(APP_NAME, "Committing updates in NVS...");
	err = storageHandler->commit();
	ESP_LOGI(APP_NAME, "%s", (err != ESP_OK) ? "Failed!" : "Done");
}

// Buttons ------------------------------------------------------------------------------------------------------------
typedef struct {
	gpio_num_t gpio_num;
	uint8_t level;
	uint32_t time;
} button_isr_queue_data_t;

void IRAM_ATTR button_isr_handler(void *arg) {
	button_isr_queue_data_t data;
	auto gpio_num = (uint32_t) arg;
	data.gpio_num = (gpio_num_t) gpio_num;
	data.level = gpio_get_level(data.gpio_num);
	data.time = millis();
	xQueueSendFromISR(btn_evt_queue, &data, nullptr);
	xTaskResumeFromISR(taskBtnISRHandler);
}

void button_short_press() {
	toggle_led();
}

void button_long_press() {
	ESP_LOGI(APP_NAME, "Button long press.");
}

void button_long_long_press() {
	resetStorage();
}

void button_task(void *arg) {
	button_isr_queue_data_t data;
	uint32_t lastTime = 0;
	while (true) {
		vTaskSuspend(NULL);
		while (xQueueReceive(btn_evt_queue, &data, portMAX_DELAY)) {
			ESP_LOGI(APP_NAME, "GPIO[%d] intr, val: %d", data.gpio_num, data.level);

			uint32_t timeDiff = data.time - lastTime;
			if (data.level == BTN_PRESSED_LEVEL && timeDiff > DEBOUNCE_MS) {
				lastTime = data.time;
			} else if (data.level != BTN_PRESSED_LEVEL) {
				if (timeDiff < BTN_LONG_PRESS_MS) {
					button_short_press();
				} else if (timeDiff > BTN_LONG_PRESS_MS && timeDiff < BTN_LONG_LONG_PRESS_MS) {
					button_long_press();
				} else {
					button_long_long_press();
				}
			}
		}
	}
	vTaskDelete(NULL);
}

void setBtn() {
	gpio_pad_select_gpio(BTN_USER);
	gpio_set_direction(BTN_USER, GPIO_MODE_INPUT);
	gpio_pullup_en(BTN_USER);

	// create a queue to handle gpio event from isr
	btn_evt_queue = xQueueCreate(10, sizeof(button_isr_queue_data_t));

	xTaskCreate(button_task, "button_task", 2048, NULL, 10, &taskBtnISRHandler);

	// install ISR service with default configuration
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	gpio_isr_handler_add(BTN_USER, button_isr_handler, (void *) BTN_USER);
	gpio_set_intr_type(BTN_USER, GPIO_INTR_ANYEDGE);
}

static uint8_t isUsrBtnPressed() {
	return gpio_get_level(BTN_USER) == BTN_PRESSED_LEVEL;
}

// ADC ----------------------------------------------------------------------------------------------------------------
// #define N_SAMPLES 64
// #define N_SAMPLES 128
// #define N_SAMPLES 256
// #define N_SAMPLES 512
#define N_SAMPLES 1024
// #define N_SAMPLES 2048

void audioFFT();

uint16_t adc_val;
uint32_t adc_mean;
float adc_raw[N_SAMPLES];
void adcTask(void *arg) {
	ESP_ERROR_CHECK(adc1_config_width((adc_bits_width_t) ADC_WIDTH_BIT_DEFAULT));
	ESP_ERROR_CHECK(adc1_config_channel_atten((adc1_channel_t) AUDIO_ADC_CH, AUDIO_ADC_ATTEN));

	while (1) {
#ifdef MEASURE_ADC_FREQ
		uint64_t xStart = micros();
#endif

		adc_mean = 0;
		for (uint16_t i = 0; i < N_SAMPLES; i++) {
			adc_val = adc1_get_raw((adc1_channel_t) AUDIO_ADC_CH);
			adc_mean += adc_val;
			adc_raw[i] = adc_val;
		}
		adc_mean >>= 6;
		for (uint16_t i = 0; i < N_SAMPLES; i++) {
			adc_raw[i] -= adc_mean;
		}
		audioFFT();
		// vTaskResume(taskFFTHandler);
		// delay(1000);
		vTaskDelay(1);

#ifdef MEASURE_ADC_FREQ
		uint64_t xEnd = micros();
		uint32_t timeDiff = xEnd - xStart;
		ESP_LOGI(APP_NAME, "ADC time N_SAMPLES sample: %dus", timeDiff);
		float freq = N_SAMPLES .0 * 1000000.0 / timeDiff;
		ESP_LOGI(APP_NAME, "ADC Freq: %0.2fkHz", freq / 1000);
		vTaskDelay(pdMS_TO_TICKS(1000));
#endif
	}
}

int N = N_SAMPLES;
// Window coefficients
__attribute__((aligned(16))) float wind[N_SAMPLES];
// working complex array
__attribute__((aligned(16))) float y_cf[N_SAMPLES * 2];
// Pointers to result arrays
float *y1_cf = &y_cf[0];

// void audioFFT(void *arg) {
void audioFFT() {
	// while (true) {
		esp_err_t err = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
		if (err != ESP_OK) {
			ESP_LOGE(APP_NAME, "Not possible to initialize FFT. Error = %i", err);
			return;
		}

		// Generate hann window
		dsps_wind_hann_f32(wind, N);
		// dsps_tone_gen_f32(adc_raw, N, 10.0, 0.2, 0);

		for (int i = 0; i < N; i++) {
			y_cf[i * 2 + 0] = adc_raw[i] * wind[i];
			y_cf[i * 2 + 1] = 0;
		}

		dsps_fft2r_fc32(y_cf, N);
		dsps_bit_rev_fc32(y_cf, N);
		dsps_cplx2reC_fc32(y_cf, N);

		for (int i = 0; i < N / 2; i++) {
			y1_cf[i] = 10 * log10f((y1_cf[i * 2 + 0] * y1_cf[i * 2 + 0] + y1_cf[i * 2 + 1] * y1_cf[i * 2 + 1]) / N);
		}

		float min = y1_cf[0];
		float max = y1_cf[0];
		for (int i = 0; i < N / 2; i++) {
			if (min > y1_cf[i]) {
				min = y1_cf[i];
			}

			if (max < y1_cf[i]) {
				max = y1_cf[i];
			}
		}

		max -= min;
		for (int i = 0; i < N / 2; i++) {
			y1_cf[i] -= min;
			y1_cf[i] /= max;
			y1_cf[i] = y1_cf[i] * y1_cf[i];
			y1_cf[i] = y1_cf[i] * y1_cf[i];
			y1_cf[i] *= 1000;
		}
		y1_cf[0] = 0;
		y1_cf[1] = 0;

		// dsps_view(const float *data, int32_t len, int width, int height, float min, float max, char view_char)
		// dsps_view(y1_cf, N / 2, fmin(N / 2, 64), 30, -10, 1100, '.');

		// vTaskDelete(NULL);
		// vTaskSuspend(NULL);
		// delay(200);
	// }
}

// Main ---------------------------------------------------------------------------------------------------------------
static void powerOnResetRoutine() {
	if (!isUsrBtnPressed()) {
		return;
	}

	ESP_LOGI(APP_NAME, "User button pressed...");
	vTaskDelay(10000 / portTICK_PERIOD_MS);
	if (!isUsrBtnPressed()) {
		return;
	}

	resetStorage();
}

void task1(void *arg) {
	ESP_LOGI(APP_NAME, "Task 1");
	vTaskDelete(NULL);
}

void task2(void *arg) {
	for (int i = 0; i < 5; i++) {
		ESP_LOGI(APP_NAME, "Task 2");
		delay(1000);
	}
	vTaskDelete(NULL);
}

extern "C" void app_main() {
	ESP_LOGI(APP_NAME, "Init ----------------------------");

	powerOnResetRoutine();

	initStorage();
	increaseRestartCounterStorage();
	setLED();
	setBtn();
	delay(1000);

	// xTaskCreate(task1, "task1", 4096, NULL, 10, &task1Handler);
	// xTaskCreate(task2, "task2", 4096, NULL, 10, &task2Handler);
	xTaskCreate(adcTask, "adcTask", 4096, NULL, 10, &taskADCHandler);
	// xTaskCreate(audioFFT, "audioFFT", 4096, NULL, 10, &taskFFTHandler);
	// ESP_LOGI(APP_NAME, "Ending...");
}
