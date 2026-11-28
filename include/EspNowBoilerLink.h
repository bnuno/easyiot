#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

constexpr uint32_t kEspNowPacketMagic = 0x45494F54UL; // 'EIOT'
constexpr uint8_t kEspNowPacketVersion = 1;
constexpr uint8_t kEspNowMsgRoomTelemetry = 1;
constexpr uint8_t kEspNowMsgBoilerAck = 2;

struct __attribute__((packed)) EspNowRoomPacket
{
    uint32_t magic;
    uint8_t version;
    uint8_t msgType;
    float roomTemp;
    float humidity;
    float spRoom;
    uint8_t flags;
    int8_t rssiHint;
    uint16_t seq;
};

bool espNowEnsureInit(uint8_t preferredChannel);
bool espNowSendPacket(const uint8_t *destMac, uint8_t channel, const EspNowRoomPacket &pkt);
void espNowFormatMac(const uint8_t *mac, char *out, size_t outLen);
bool espNowParseMac(const char *str, uint8_t *outMac);
bool espNowIsBroadcast(const uint8_t *mac);

// Implemented by Boiler.cpp and RoomThermostat.cpp
void boilerOnEspNowPacket(const uint8_t *mac, const EspNowRoomPacket &pkt);
void thermostatOnEspNowAck(const uint8_t *mac, const EspNowRoomPacket &pkt);
