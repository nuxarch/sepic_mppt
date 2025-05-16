#include "sepic.h"

void setup()
{
    Serial.begin(100000);

    xTaskCreate(vPIDTask, "vPIDTask", configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vMainTask, "vMainTask", configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vStairSetpointTask, "vStairSetpointTask", configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1, NULL);

    // xTaskCreate(vPlottingTask, "vPlottingTask", configMINIMAL_STACK_SIZE + 100, NULL, tskIDLE_PRIORITY + 2, NULL);
    // xTaskCreate(vPrintTask, "vPrintTask", configMINIMAL_STACK_SIZE + 100, NULL, tskIDLE_PRIORITY + 2, NULL);
    vTaskStartScheduler();
}

void loop()
{
    // delay(1000);
}
