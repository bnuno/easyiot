#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

void boilerSetup();
void boilerLoop();

bool boilerHandleMqtt(const char *topic, const char *payload);
void boilerSubscribe();
void boilerDiscovery();

int boilerCreateDefault();
int boilerCreate(JsonVariantConst json);
int boilerUpdateHardware(JsonVariantConst json);
void boilerRemove();
bool boilerIsCreated();
bool boilerUsesPin(unsigned int pin);

void boilerConfigJson(JsonVariant root);
void boilerStateJson(JsonVariant &root);
bool boilerQueueCommand(JsonVariantConst json);
void boilerRequestScan();
