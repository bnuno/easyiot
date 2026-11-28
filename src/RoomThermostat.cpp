#include "RoomThermostat.h"
#include "ConfigOnofre.h"
#include "Constants.h"
#include "DeviceLog.h"
#include "EspNowBoilerLink.h"
#include "LittleFS.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifdef ESP32
#include <WiFi.h>
#include <esp_idf_version.h>
#include <esp_now.h>
#include <esp_wifi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

extern ConfigOnofre config;

// ---------------- Shared ESP-NOW link helpers ----------------

namespace
{
bool espNowReady = false;
uint8_t espNowAppliedChannel = 0;

#ifdef ESP32
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onEspNowDispatchRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    const uint8_t *mac = (info != nullptr) ? info->src_addr : nullptr;
#else
void onEspNowDispatchRecv(const uint8_t *mac, const uint8_t *data, int len)
{
#endif
    if (mac == nullptr || data == nullptr || len != (int)sizeof(EspNowRoomPacket))
        return;
    EspNowRoomPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.magic != kEspNowPacketMagic || pkt.version != kEspNowPacketVersion)
        return;
    if (pkt.msgType == kEspNowMsgRoomTelemetry)
        boilerOnEspNowPacket(mac, pkt);
    else if (pkt.msgType == kEspNowMsgBoilerAck)
        thermostatOnEspNowAck(mac, pkt);
}
#endif
} // namespace

void espNowFormatMac(const uint8_t *mac, char *out, size_t outLen)
{
    if (!out || outLen < 18)
        return;
    if (!mac)
    {
        strlcpy(out, "00:00:00:00:00:00", outLen);
        return;
    }
    snprintf(out, outLen, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool espNowParseMac(const char *str, uint8_t *outMac)
{
    if (!str || !outMac)
        return false;
    unsigned int b[6] = {0};
    if (sscanf(str, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6)
    {
        if (sscanf(str, "%x-%x-%x-%x-%x-%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6)
            return false;
    }
    for (int i = 0; i < 6; i++)
    {
        if (b[i] > 255u)
            return false;
        outMac[i] = (uint8_t)b[i];
    }
    return true;
}

bool espNowIsBroadcast(const uint8_t *mac)
{
    if (!mac)
        return false;
    for (int i = 0; i < 6; i++)
        if (mac[i] != 0xFF)
            return false;
    return true;
}

bool espNowEnsureInit(uint8_t preferredChannel)
{
#ifdef ESP32
    if (WiFi.getMode() == WIFI_OFF)
        WiFi.mode(WIFI_STA);

    if (preferredChannel >= 1 && preferredChannel <= 13 &&
        WiFi.status() != WL_CONNECTED &&
        espNowAppliedChannel != preferredChannel)
    {
        esp_wifi_set_channel(preferredChannel, WIFI_SECOND_CHAN_NONE);
        espNowAppliedChannel = preferredChannel;
    }

    if (!espNowReady)
    {
        if (esp_now_init() != ESP_OK)
            return false;
        esp_now_register_recv_cb(onEspNowDispatchRecv);
        espNowReady = true;
    }
    return true;
#else
    (void)preferredChannel;
    return false;
#endif
}

bool espNowSendPacket(const uint8_t *destMac, uint8_t channel, const EspNowRoomPacket &pkt)
{
#ifdef ESP32
    if (!destMac || !espNowEnsureInit(channel))
        return false;

    const wifi_mode_t mode = WiFi.getMode();
    const wifi_interface_t ifidx = (mode == WIFI_AP) ? WIFI_IF_AP : WIFI_IF_STA;
    const uint8_t peerChan = (WiFi.status() == WL_CONNECTED) ? 0 : (channel <= 13 ? channel : 0);

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, destMac, 6);
    peer.channel = peerChan;
    peer.ifidx = ifidx;
    peer.encrypt = false;

    if (!esp_now_is_peer_exist(destMac))
    {
        if (esp_now_add_peer(&peer) != ESP_OK)
            return false;
    }
    else
    {
        esp_now_mod_peer(&peer);
    }

    return esp_now_send(destMac, reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt)) == ESP_OK;
#else
    (void)destMac;
    (void)channel;
    (void)pkt;
    return false;
#endif
}

// ---------------- ESP-NOW Bridge for Auto-Detected SHT4X ----------------

namespace
{
const char *kThermostatFile = "/thermostat.json";
const char *kThermostatTmp = "/thermostat.tmp";
constexpr unsigned long kSensorSampleMs = 1500;
constexpr unsigned long kSaveDelayMs = 2500;
constexpr unsigned long kMinJitterTxMs = 1500;

struct ThermostatLinkConfig
{
    uint8_t targetMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t channel = 0;
    uint16_t sendIntervalSec = 10;
    float tempOffset = 0.0f;
    bool syncSetpoint = false;
    float spRoom = 21.0f;
} tcfg;

bool hasSht4xSensor = false;
char shtSensorName[32] = "Termostato ambiente";
float shtTemp = NAN;
float shtHum = NAN;
bool shtOk = false;

unsigned long lastSampleMs = 0;
unsigned long lastTxMs = 0;
float lastTxTemp = NAN;
uint16_t txSeq = 0;
uint32_t txCount = 0;
uint32_t ackCount = 0;
bool lastTxOk = false;
bool sendRequested = false;
bool espNowLinkInit = false;

unsigned long lastChangeMs = 0;
bool needsSave = false;

// Async ESP-NOW ACK staging
volatile bool ackPending = false;
EspNowRoomPacket ackPktStaging;
uint8_t ackMacStaging[6] = {0};

bool boilerOnline = false;
bool boilerOn = false;
bool boilerPump = false;
bool boilerHeater = false;
float boilerWaterTemp = NAN;
float boilerSpRoom = NAN;
unsigned long lastAckMs = 0;
uint8_t lastBoilerMac[6] = {0};

char thPendingBuf[256];
volatile bool thPendingFlag = false;

void saveThermostat()
{
    JsonDocument doc;
    char macStr[20];
    espNowFormatMac(tcfg.targetMac, macStr, sizeof(macStr));
    doc["mac"] = macStr;
    doc["ch"] = tcfg.channel;
    doc["iv"] = tcfg.sendIntervalSec;
    doc["off"] = tcfg.tempOffset;
    doc["syncSp"] = tcfg.syncSetpoint;
    doc["spR"] = tcfg.spRoom;

    File f = LittleFS.open(kThermostatTmp, "w");
    if (!f)
        return;
    serializeJson(doc, f);
    f.close();
    LittleFS.remove(kThermostatFile);
    LittleFS.rename(kThermostatTmp, kThermostatFile);
}

void loadThermostat()
{
    tcfg = ThermostatLinkConfig();
    if (!LittleFS.exists(kThermostatFile))
        return;
    File f = LittleFS.open(kThermostatFile, "r");
    if (!f)
        return;
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err)
        return;

    if (doc["mac"].is<const char *>())
    {
        uint8_t parsed[6];
        if (espNowParseMac(doc["mac"].as<const char *>(), parsed))
            memcpy(tcfg.targetMac, parsed, 6);
    }
    tcfg.channel = (uint8_t)constrain((int)(doc["ch"] | 0), 0, 13);
    tcfg.sendIntervalSec = (uint16_t)constrain((int)(doc["iv"] | 10), 2, 300);
    tcfg.tempOffset = constrain((float)(doc["off"] | 0.0f), -5.0f, 5.0f);
    tcfg.syncSetpoint = doc["syncSp"] | false;
    tcfg.spRoom = constrain((float)(doc["spR"] | 21.0f), 16.0f, 30.0f);
}

void refreshAutoDetectedSht4x()
{
    String stateCopy;
    char nameCopy[32] = "Termostato ambiente";
    bool found = false;

    if (!config.tryBeginFeatureAccess("espnowSht4x"))
        return;
    for (const auto &s : config.sensors)
    {
        if (s.driver == SensorDriver::SHT4X)
        {
            found = true;
            strlcpy(nameCopy, s.name, sizeof(nameCopy));
            stateCopy = s.state;
            break;
        }
    }
    config.endFeatureAccess();

    hasSht4xSensor = found;
    strlcpy(shtSensorName, nameCopy, sizeof(shtSensorName));
    if (!found || stateCopy.length() == 0)
    {
        if (!found)
        {
            shtTemp = NAN;
            shtHum = NAN;
            shtOk = false;
        }
        return;
    }

    JsonDocument doc;
    if (deserializeJson(doc, stateCopy))
        return;

    if (doc["temperature"].is<float>() || doc["temperature"].is<int>())
    {
        const float rawT = doc["temperature"].as<float>();
        if (!isnan(rawT) && rawT > -40.0f && rawT < 85.0f)
        {
            shtTemp = roundf((rawT + tcfg.tempOffset) * 10.0f) / 10.0f;
            shtOk = true;
        }
    }
    if (doc["humidity"].is<float>() || doc["humidity"].is<int>())
    {
        const float rawH = doc["humidity"].as<float>();
        if (!isnan(rawH) && rawH >= 0.0f && rawH <= 100.0f)
        {
            shtHum = roundf(rawH * 10.0f) / 10.0f;
        }
    }
}

void sendTelemetryPacket(unsigned long now)
{
    if (!shtOk || isnan(shtTemp))
        return;
    EspNowRoomPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = kEspNowPacketMagic;
    pkt.version = kEspNowPacketVersion;
    pkt.msgType = kEspNowMsgRoomTelemetry;
    pkt.roomTemp = shtTemp;
    pkt.humidity = isnan(shtHum) ? 0.0f : shtHum;
    pkt.spRoom = tcfg.spRoom;
    pkt.flags = tcfg.syncSetpoint ? 0x01 : 0x00;
#ifdef ESP32
    pkt.rssiHint = (WiFi.status() == WL_CONNECTED) ? (int8_t)constrain((int)WiFi.RSSI(), -127, 0) : 0;
#else
    pkt.rssiHint = 0;
#endif
    pkt.seq = ++txSeq;

    lastTxOk = espNowSendPacket(tcfg.targetMac, tcfg.channel, pkt);
    lastTxMs = now;
    if (lastTxOk)
    {
        lastTxTemp = shtTemp;
        txCount++;
    }
}

void applyJsonCommand(JsonVariantConst doc)
{
    bool changed = false;
    bool txNow = false;

    if (doc["targetMac"].is<const char *>())
    {
        uint8_t parsed[6];
        if (espNowParseMac(doc["targetMac"].as<const char *>(), parsed))
        {
            memcpy(tcfg.targetMac, parsed, 6);
            changed = true;
            txNow = true;
        }
    }
    if (doc["channel"].is<int>())
    {
        tcfg.channel = (uint8_t)constrain(doc["channel"].as<int>(), 0, 13);
        espNowAppliedChannel = 0;
        espNowEnsureInit(tcfg.channel);
        changed = true;
        txNow = true;
    }
    if (doc["sendIntervalSec"].is<int>())
    {
        tcfg.sendIntervalSec = (uint16_t)constrain(doc["sendIntervalSec"].as<int>(), 2, 300);
        changed = true;
    }
    if (doc["tempOffset"].is<float>() || doc["tempOffset"].is<int>())
    {
        tcfg.tempOffset = constrain(roundf(doc["tempOffset"].as<float>() * 10.0f) / 10.0f, -5.0f, 5.0f);
        changed = true;
        txNow = true;
    }
    if (doc["syncSetpoint"].is<bool>())
    {
        tcfg.syncSetpoint = doc["syncSetpoint"].as<bool>();
        changed = true;
        txNow = true;
    }
    if (doc["spRoom"].is<float>() || doc["spRoom"].is<int>())
    {
        tcfg.spRoom = constrain(roundf(doc["spRoom"].as<float>() * 10.0f) / 10.0f, 16.0f, 30.0f);
        changed = true;
        if (tcfg.syncSetpoint)
            txNow = true;
    }
    if (doc["sendNow"] | false)
        txNow = true;

    if (changed)
    {
        needsSave = true;
        lastChangeMs = millis();
    }
    if (txNow)
        sendRequested = true;
}
} // namespace

bool isThermostatTemplateActive()
{
    return config.templateId == Template::THERMOSTAT;
}

void thermostatOnEspNowAck(const uint8_t *mac, const EspNowRoomPacket &pkt)
{
    if (!mac || ackPending)
        return;
    memcpy(ackMacStaging, mac, 6);
    ackPktStaging = pkt;
    __sync_synchronize();
    ackPending = true;
}

void thermostatSetup()
{
    loadThermostat();
    refreshAutoDetectedSht4x();
#ifdef ESP32
    if (isThermostatTemplateActive() && hasSht4xSensor)
    {
        WiFi.setSleep(false);
        espNowEnsureInit(tcfg.channel);
        espNowLinkInit = true;
        sendRequested = true;
    }
#endif
}

void thermostatLoop()
{
    const unsigned long now = millis();

    if (thPendingFlag)
    {
        char local[sizeof(thPendingBuf)];
        memcpy(local, thPendingBuf, sizeof(local));
        thPendingFlag = false;
        JsonDocument doc;
        if (!deserializeJson(doc, local))
            applyJsonCommand(doc.as<JsonVariantConst>());
    }

    if (ackPending)
    {
        EspNowRoomPacket ack = ackPktStaging;
        uint8_t srcMac[6];
        memcpy(srcMac, ackMacStaging, 6);
        __sync_synchronize();
        ackPending = false;

        if (isThermostatTemplateActive())
        {
            memcpy(lastBoilerMac, srcMac, 6);
            boilerOnline = true;
            boilerOn = (ack.flags & 0x04) != 0;
            boilerPump = (ack.flags & 0x08) != 0;
            boilerHeater = (ack.flags & 0x10) != 0;
            boilerWaterTemp = ack.roomTemp;
            boilerSpRoom = ack.spRoom;
            lastAckMs = now;
            ackCount++;
        }
    }

    if (lastSampleMs == 0 || (now - lastSampleMs) >= kSensorSampleMs)
    {
        lastSampleMs = now;
        refreshAutoDetectedSht4x();
    }

    if (!isThermostatTemplateActive() || !hasSht4xSensor)
    {
        if (needsSave && (now - lastChangeMs) >= kSaveDelayMs)
        {
            needsSave = false;
            saveThermostat();
        }
        return;
    }

#ifdef ESP32
    if (!espNowLinkInit)
    {
        WiFi.setSleep(false);
        espNowEnsureInit(tcfg.channel);
        espNowLinkInit = true;
        sendRequested = true;
    }
#endif

    const unsigned long intervalMs = (unsigned long)tcfg.sendIntervalSec * 1000UL;
    const bool tempChanged =
        shtOk && !isnan(shtTemp) &&
        (isnan(lastTxTemp) || fabsf(shtTemp - lastTxTemp) >= 0.1f) &&
        (lastTxMs == 0 || (now - lastTxMs) >= kMinJitterTxMs);
    const bool intervalDue = shtOk && (lastTxMs == 0 || (now - lastTxMs) >= intervalMs);

    if (sendRequested || tempChanged || intervalDue)
    {
        sendRequested = false;
        sendTelemetryPacket(now);
    }

    if (needsSave && (now - lastChangeMs) >= kSaveDelayMs)
    {
        needsSave = false;
        saveThermostat();
    }
}

void thermostatStateJson(JsonVariant &root)
{
    const unsigned long now = millis();
    const bool active = isThermostatTemplateActive();
    root["templateActive"] = active;
    root["enabled"] = active && hasSht4xSensor;
    root["name"] = shtSensorName;
    root["shtOk"] = shtOk;
    if (!isnan(shtTemp))
        root["temperature"] = shtTemp;
    else
        root["temperature"] = nullptr;
    if (!isnan(shtHum))
        root["humidity"] = shtHum;
    else
        root["humidity"] = nullptr;
    root["spRoom"] = tcfg.spRoom;
    root["syncSetpoint"] = tcfg.syncSetpoint;
    root["tempOffset"] = tcfg.tempOffset;
    root["sendIntervalSec"] = tcfg.sendIntervalSec;
    root["channel"] = tcfg.channel;

    char macStr[20];
    espNowFormatMac(tcfg.targetMac, macStr, sizeof(macStr));
    root["targetMac"] = macStr;
    root["broadcast"] = espNowIsBroadcast(tcfg.targetMac);

#ifdef ESP32
    root["ownMac"] = WiFi.macAddress();
    root["wifiChannel"] = (int)WiFi.channel();
    root["espNowReady"] = espNowReady;
#else
    root["ownMac"] = "ESP8266";
    root["wifiChannel"] = 0;
    root["espNowReady"] = false;
#endif

    root["txCount"] = txCount;
    root["ackCount"] = ackCount;
    root["lastTxOk"] = lastTxOk;
    root["lastTxAgoSec"] = (lastTxMs > 0) ? (long)((now - lastTxMs) / 1000UL) : -1;
    root["lastAckAgoSec"] = (lastAckMs > 0) ? (long)((now - lastAckMs) / 1000UL) : -1;
    root["boilerOnline"] = boilerOnline && lastAckMs > 0 &&
                           ((now - lastAckMs) < (unsigned long)tcfg.sendIntervalSec * 4000UL + 15000UL);
    root["boilerOn"] = boilerOn;
    root["boilerPump"] = boilerPump;
    root["boilerHeater"] = boilerHeater;
    if (!isnan(boilerWaterTemp))
        root["boilerWaterTemp"] = roundf(boilerWaterTemp * 10.0f) / 10.0f;
    else
        root["boilerWaterTemp"] = nullptr;
    if (!isnan(boilerSpRoom))
        root["boilerSpRoom"] = boilerSpRoom;
    else
        root["boilerSpRoom"] = nullptr;

    if (lastAckMs > 0)
    {
        char bMac[20];
        espNowFormatMac(lastBoilerMac, bMac, sizeof(bMac));
        root["lastBoilerMac"] = bMac;
    }
    else
    {
        root["lastBoilerMac"] = "";
    }
}

bool thermostatQueueCommand(JsonVariantConst json)
{
    if (thPendingFlag || measureJson(json) >= sizeof(thPendingBuf))
        return false;
    serializeJson(json, thPendingBuf, sizeof(thPendingBuf));
    __sync_synchronize();
    thPendingFlag = true;
    return true;
}

void thermostatRequestSend()
{
    sendRequested = true;
}
