#include "application.hpp"

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
