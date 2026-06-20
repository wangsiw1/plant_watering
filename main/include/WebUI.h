#pragma once
#include <Arduino.h>
#include <freertos/task.h>

void webBegin();
void webHandleClientLoop();
void webSetDiagnosticsTaskHandles(TaskHandle_t loopTask,
                                  TaskHandle_t webTask,
                                  TaskHandle_t sensorTask,
                                  TaskHandle_t wateringTask);
