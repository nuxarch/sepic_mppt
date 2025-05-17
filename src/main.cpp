#include "sepic.h"

void setup()
{
    Serial.begin(100000);

    // Create a queue to hold 10 integers
    QueueVout = xQueueCreate(500, sizeof(double));
    if (QueueVout == NULL)
    {
        Serial.println("Queue creation failed!");
        while (1)
            ;
    }

    xTaskCreate(vPIDTask, "vPIDTask", configMINIMAL_STACK_SIZE + 1048, NULL, tskIDLE_PRIORITY, &vPIDTaskHandle);
    xTaskCreate(vMainTask, "vMainTask", configMINIMAL_STACK_SIZE + 1048, NULL, tskIDLE_PRIORITY + 1, &vMainTaskHandle);
    xTaskCreate(vStairSetpointTask, "vStairSetpointTask", configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1, &vStairSetpointTaskHandle);

    // xTaskCreate(vPlottingTask, "vPlottingTask", configMINIMAL_STACK_SIZE + 100, NULL, tskIDLE_PRIORITY + 2, NULL);
    // xTaskCreate(vPrintTask, "vPrintTask", configMINIMAL_STACK_SIZE + 100, NULL, tskIDLE_PRIORITY + 2, NULL);
    vTaskStartScheduler();
}

void loop()
{
    // printTaskStatus();
    // delay(1000);
}
