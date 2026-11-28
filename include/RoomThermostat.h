#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

void thermostatSetup();
void thermostatLoop();

void thermostatStateJson(JsonVariant &root);
bool thermostatQueueCommand(JsonVariantConst json);
void thermostatRequestSend();
