#include "Boiler.h"
#include "ConfigOnofre.h"
#include "Constants.h"
#include "DeviceClock.h"
#include "DeviceLog.h"
#include "EspNowBoilerLink.h"
#include "LittleFS.h"
#include "Mqtt.h"
#ifdef ESP32
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

extern ConfigOnofre config;

namespace
{
    const char *kBoilerFile = "/boiler.json";
    const char *kBoilerTmp = "/boiler.tmp";
    constexpr unsigned long kSaveDelayMs = 2500;
    constexpr unsigned long kReadIntervalMs = 10000;
    constexpr unsigned long kConvWaitMs = 800;
    constexpr unsigned long kLcdIntervalMs = 500;
    constexpr unsigned long kMenuTimeoutMs = 20000;
    constexpr unsigned long kDebounceMs = 10;
    constexpr unsigned long kMqttPublishMinMs = 5000;
    constexpr int kMaxProbes = 8;

    volatile bool created = false;
    bool pinCheckBypass = false;

    struct BoilerConfig
    {
        char name[24] = "Caldeira";
#if defined(ESP32) && !defined(ESP32C3) && !defined(ESP32C6)
        int sdaPin = 32;
        int sclPin = 33;
        int dallasPin = 25;
        int pumpPin = 4;
        int heaterPin = 5;
        int btnMenuPin = 14;
        int btnPlusPin = 26;
        int btnMinusPin = 27;
        int btnPowerPin = 13;
#else
        int sdaPin = DefaultPins::SDA;
        int sclPin = DefaultPins::SCL;
        int dallasPin = DefaultPins::OUTPUT_ONE;
        int pumpPin = DefaultPins::OUTPUT_ONE;
        int heaterPin = DefaultPins::OUTPUT_TWO;
        int btnMenuPin = DefaultPins::INPUT_ONE;
        int btnPlusPin = -1;
        int btnMinusPin = -1;
        int btnPowerPin = DefaultPins::INPUT_TWO;
#endif
        uint8_t lcdAddr = 0x27;
        uint8_t romWater[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        uint8_t romReturn[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        uint16_t heaterWatts = 6500;
        uint16_t pumpWatts = 70;
        float spHeater = 50.0f;
        float spPump = 30.0f;
        float spRoom = 21.0f;
        float diffHeater = 3.0f;
        float diffPump = 3.0f;
        float diffRoom = 0.5f;
        bool masterOn = false;
        float roomTemp = 20.0f;
        uint32_t secHeaterToday = 0;
        uint32_t secPumpToday = 0;
    } cfg;

    // Runtime state
    bool pumpOn = false;
    bool heaterOn = false;
    float tempWater = NAN;
    float tempReturn = NAN;
    int menuIndex = 0;
    unsigned long menuActivityMs = 0;

    struct ProbeInfo
    {
        uint8_t addr[8];
        float temp;
    };
    ProbeInfo probes[kMaxProbes];
    int probeCount = 0;

    OneWire *oneWire = nullptr;
    DallasTemperature *dallas = nullptr;
    bool convPending = false;
    bool scanRequested = false;
    unsigned long convStartMs = 0;
    unsigned long lastReadMs = 0;
    unsigned long lastSecMs = 0;
    unsigned long lastLcdMs = 0;
    unsigned long lastPublishMs = 0;
    unsigned long lastChangeMs = 0;
    int lastMinOfDay = -1;

    bool needsSave = false;
    bool needsPublish = false;
    bool needsDiscovery = false;
    bool needsUndiscovery = false;
    bool hwInitDone = false;
    bool lcdAvailable = false;
    bool lcdForceRedraw = true;
    uint8_t lcdActiveAddr = 0;
    int lcdActiveSda = -1;
    int lcdActiveScl = -1;
    char lcdPrev[4][21];

    char pendingBuf[512];
    volatile bool pendingFlag = false;

    // ESP-NOW telemetry staging from room SHT4x sensor
    volatile bool espNowRxPending = false;
    EspNowRoomPacket espNowRxPktStaging;
    uint8_t espNowRxMacStaging[6] = {0};
    unsigned long lastEspNowRxMs = 0;
    uint32_t espNowRxCount = 0;
    uint8_t lastEspNowSenderMac[6] = {0};
    float lastEspNowRoomHum = NAN;

    char tSet[64], tNumSet[64], tResetSet[64], tState[64];

    void initTopics()
    {
        snprintf(tSet, sizeof(tSet), "onofre/%s/boiler/set", config.chipId);
        snprintf(tNumSet, sizeof(tNumSet), "onofre/%s/boiler/num/+/set", config.chipId);
        snprintf(tResetSet, sizeof(tResetSet), "onofre/%s/boiler/reset/set", config.chipId);
        snprintf(tState, sizeof(tState), "onofre/%s/boiler/state", config.chipId);
    }

    bool romIsEmpty(const uint8_t *rom)
    {
        for (int i = 0; i < 8; i++)
            if (rom[i] != 0)
                return false;
        return true;
    }

    bool romEqual(const uint8_t *a, const uint8_t *b)
    {
        return memcmp(a, b, 8) == 0;
    }

    // Standard uint64_t hex string matching ESPHome format (e.g. "be0b255482876428", rom[7]..rom[0])
    void romToHex(const uint8_t *rom, char *out, size_t outLen)
    {
        if (outLen < 17)
            return;
        for (int i = 0; i < 8; i++)
            snprintf(out + i * 2, 3, "%02x", rom[7 - i]);
        out[16] = '\0';
    }

    bool hexToRom(const char *hex, uint8_t *rom)
    {
        if (!hex)
            return false;
        if (hex[0] == '0' && (hex[1] == 'x' || hex[1] == 'X'))
            hex += 2;
        if (strlen(hex) != 16)
            return false;
        for (int i = 0; i < 8; i++)
        {
            char buf[3] = {hex[i * 2], hex[i * 2 + 1], '\0'};
            char *endp = nullptr;
            unsigned long v = strtoul(buf, &endp, 16);
            if (!endp || *endp != '\0')
                return false;
            rom[7 - i] = (uint8_t)v;
        }
        return true;
    }

    bool isBtnPinActive(int pin)
    {
        return pin >= 0 && pin != (int)DefaultPins::noGPIO;
    }

    bool validBusOrOutPin(int pin)
    {
        if (pin < 0)
            return false;
        if (pin == (int)DefaultPins::SDA || pin == (int)DefaultPins::SCL)
            return true;
#if defined(ESP32) && !defined(ESP32C3) && !defined(ESP32C6)
        static const int safePins[] = {4, 5, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
        for (size_t i = 0; i < sizeof(safePins) / sizeof(safePins[0]); i++)
        {
            if (safePins[i] == pin)
                return true;
        }
#endif
        return config.validOutputPin((unsigned int)pin);
    }

    bool validBtnPin(int pin)
    {
        if (pin == -1 || pin == (int)DefaultPins::noGPIO)
            return true;
        if (validBusOrOutPin(pin))
            return true;
#if defined(ESP32) && !defined(ESP32C3) && !defined(ESP32C6)
        if (pin == 34 || pin == 35 || pin == 36 || pin == 39)
            return true;
#endif
        return pin >= 0 && config.validInputPin((unsigned int)pin);
    }

    // ---------------- Lightweight PCF8574 20x4 I2C LCD ----------------
    constexpr uint8_t LCD_BACKLIGHT = 0x08;
    constexpr uint8_t LCD_EN = 0x04;
    constexpr uint8_t LCD_RS = 0x01;

    void lcdWriteByte(uint8_t val)
    {
        Wire.beginTransmission(cfg.lcdAddr);
        Wire.write(val | LCD_BACKLIGHT);
        Wire.endTransmission();
    }

    void lcdPulseEnable(uint8_t val)
    {
        lcdWriteByte(val | LCD_EN);
        delayMicroseconds(1);
        lcdWriteByte(val & ~LCD_EN);
        delayMicroseconds(45);
    }

    void lcdWrite4Bits(uint8_t val)
    {
        lcdWriteByte(val);
        lcdPulseEnable(val);
    }

    void lcdSend(uint8_t value, uint8_t mode)
    {
        lcdWrite4Bits((value & 0xF0) | mode);
        lcdWrite4Bits(((value << 4) & 0xF0) | mode);
    }

    void lcdCommand(uint8_t cmd)
    {
        lcdSend(cmd, 0);
    }

    void lcdSetCursor(uint8_t col, uint8_t row)
    {
        static const uint8_t rowOffsets[4] = {0x00, 0x40, 0x14, 0x54};
        if (row > 3)
            row = 3;
        lcdCommand(0x80 | (col + rowOffsets[row]));
    }

    void lcdInit()
    {
        lcdAvailable = false;
        if (cfg.sdaPin < 0 || cfg.sclPin < 0 || cfg.lcdAddr < 0x08 || cfg.lcdAddr > 0x77)
            return;
        if (lcdActiveSda != cfg.sdaPin || lcdActiveScl != cfg.sclPin)
        {
            Wire.begin(cfg.sdaPin, cfg.sclPin);
            lcdActiveSda = cfg.sdaPin;
            lcdActiveScl = cfg.sclPin;
        }
        Wire.beginTransmission(cfg.lcdAddr);
        if (Wire.endTransmission() != 0)
            return;

        delay(50);
        lcdWriteByte(0x00);
        delayMicroseconds(500);
        lcdWrite4Bits(0x30);
        delayMicroseconds(4500);
        lcdWrite4Bits(0x30);
        delayMicroseconds(4500);
        lcdWrite4Bits(0x30);
        delayMicroseconds(150);
        lcdWrite4Bits(0x20); // 4-bit interface
        lcdCommand(0x28);    // 4-bit, 2+ lines, 5x8 font
        lcdCommand(0x08);    // display off
        lcdCommand(0x01);    // clear
        delay(2);
        lcdCommand(0x06);    // entry mode: increment, no shift
        lcdCommand(0x0C);    // display on, cursor off, blink off

        lcdAvailable = true;
        lcdActiveAddr = cfg.lcdAddr;
        lcdForceRedraw = true;
        for (int r = 0; r < 4; r++)
            lcdPrev[r][0] = '\0';
    }

    void formatTemp5(float t, char *out, size_t len)
    {
        if (isnan(t))
            snprintf(out, len, "--.-C");
        else
            snprintf(out, len, "%.1fC", t);
    }

    void lcdRender()
    {
        if (!lcdAvailable)
            return;
        if (cfg.lcdAddr != lcdActiveAddr || cfg.sdaPin != lcdActiveSda || cfg.sclPin != lcdActiveScl)
        {
            lcdInit();
            if (!lcdAvailable)
                return;
        }

        char lines[4][21];
        for (int r = 0; r < 4; r++)
            memset(lines[r], ' ', 20), lines[r][20] = '\0';

        char raw[4][48];
        for (int r = 0; r < 4; r++)
            raw[r][0] = '\0';

        if (menuIndex == 0)
        {
            char tw[10], tr[10], ts[10];
            formatTemp5(tempWater, tw, sizeof(tw));
            formatTemp5(tempReturn, tr, sizeof(tr));
            formatTemp5(cfg.roomTemp, ts, sizeof(ts));
            snprintf(raw[0], sizeof(raw[0]), "   MENU PRINCIPAL");
            snprintf(raw[1], sizeof(raw[1]), "Agua:%s Ret:%s", tw, tr);
            snprintf(raw[2], sizeof(raw[2]), "Sala:%s Set:%.1fC", ts, cfg.spRoom);
            if (cfg.masterOn)
                snprintf(raw[3], sizeof(raw[3]), "Bomba:%-3s Resist:%-3s", pumpOn ? "ON" : "OFF", heaterOn ? "ON" : "OFF");
            else
                snprintf(raw[3], sizeof(raw[3]), "SISTEMA: STANDBY");
        }
        else
        {
            snprintf(raw[0], sizeof(raw[0]), "    CONFIGURACAO");
            if (menuIndex == 1)
            {
                snprintf(raw[1], sizeof(raw[1]), "Setpoint Resistencia");
                snprintf(raw[2], sizeof(raw[2]), "Valor: %.0f C", cfg.spHeater);
            }
            else if (menuIndex == 2)
            {
                snprintf(raw[1], sizeof(raw[1]), "Setpoint Bomba");
                snprintf(raw[2], sizeof(raw[2]), "Valor: %.0f C", cfg.spPump);
            }
            else if (menuIndex == 3)
            {
                snprintf(raw[1], sizeof(raw[1]), "Setpoint Sala");
                snprintf(raw[2], sizeof(raw[2]), "Valor: %.1f C", cfg.spRoom);
            }
            else if (menuIndex == 4)
            {
                snprintf(raw[1], sizeof(raw[1]), "Diferencial Resist.");
                snprintf(raw[2], sizeof(raw[2]), "Valor: %.0f C", cfg.diffHeater);
            }
            else if (menuIndex == 5)
            {
                snprintf(raw[1], sizeof(raw[1]), "Diferencial Bomba");
                snprintf(raw[2], sizeof(raw[2]), "Valor: %.0f C", cfg.diffPump);
            }
            else
            {
                snprintf(raw[1], sizeof(raw[1]), "Diferencial Sala");
                snprintf(raw[2], sizeof(raw[2]), "Valor: %.1f C", cfg.diffRoom);
            }
            snprintf(raw[3], sizeof(raw[3]), "Botoes: [-]  [+]");
        }

        for (int r = 0; r < 4; r++)
        {
            size_t n = strlen(raw[r]);
            if (n > 20)
                n = 20;
            memcpy(lines[r], raw[r], n);
            lines[r][20] = '\0';
            if (lcdForceRedraw || strcmp(lines[r], lcdPrev[r]) != 0)
            {
                lcdSetCursor(0, (uint8_t)r);
                for (int c = 0; c < 20; c++)
                    lcdSend((uint8_t)lines[r][c], LCD_RS);
                memcpy(lcdPrev[r], lines[r], 21);
            }
        }
        lcdForceRedraw = false;
    }

    // ---------------- Relay output + Control rules ----------------
    void applyOutputs()
    {
        if (cfg.pumpPin >= 0)
        {
            pinMode(cfg.pumpPin, OUTPUT);
            digitalWrite(cfg.pumpPin, pumpOn ? HIGH : LOW);
        }
        if (cfg.heaterPin >= 0)
        {
            pinMode(cfg.heaterPin, OUTPUT);
            digitalWrite(cfg.heaterPin, heaterOn ? HIGH : LOW);
        }
    }

    void evaluateBoiler()
    {
        const bool prevPump = pumpOn;
        const bool prevHeater = heaterOn;

        if (!cfg.masterOn || isnan(tempWater))
        {
            pumpOn = false;
            heaterOn = false;
        }
        else
        {
            // Resistance control (water hysteresis)
            if (tempWater < (cfg.spHeater - cfg.diffHeater))
                heaterOn = true;
            else if (tempWater >= cfg.spHeater)
                heaterOn = false;

            // Pump control (strict Rule 1 OFF / Rule 2 ON from ESPHome YAML)
            const bool pumpShouldOff =
                (tempWater < (cfg.spPump - cfg.diffPump)) ||
                (!isnan(cfg.roomTemp) && cfg.roomTemp >= cfg.spRoom);
            if (pumpShouldOff)
            {
                pumpOn = false;
            }
            else if (tempWater > cfg.spPump && !isnan(cfg.roomTemp) &&
                     cfg.roomTemp < (cfg.spRoom - cfg.diffRoom))
            {
                pumpOn = true;
            }
        }

        applyOutputs();
        if (prevPump != pumpOn || prevHeater != heaterOn)
        {
            lcdForceRedraw = true;
            needsPublish = true;
        }
    }

    // ---------------- 4-Button Debounce ----------------
    struct BtnState
    {
        int pin = -1;
        bool lastRaw = true;
        bool stable = true;
        unsigned long lastEdgeMs = 0;
    };
    BtnState btns[4];

    void initButtons()
    {
        const int pins[4] = {cfg.btnMenuPin, cfg.btnPlusPin, cfg.btnMinusPin, cfg.btnPowerPin};
        for (int i = 0; i < 4; i++)
        {
            btns[i].pin = isBtnPinActive(pins[i]) ? pins[i] : -1;
            btns[i].lastRaw = true;
            btns[i].stable = true;
            btns[i].lastEdgeMs = 0;
            if (btns[i].pin >= 0)
                pinMode(btns[i].pin, INPUT_PULLUP);
        }
    }

    void touchConfig()
    {
        needsSave = true;
        lastChangeMs = millis();
        needsPublish = true;
        lcdForceRedraw = true;
    }

    void handleButtonPress(int idx, unsigned long now)
    {
        menuActivityMs = now;
        if (idx == 0) // Menu
        {
            menuIndex = (menuIndex + 1) % 7;
            lcdForceRedraw = true;
            return;
        }
        if (idx == 3) // ON/OFF
        {
            cfg.masterOn = !cfg.masterOn;
            evaluateBoiler();
            touchConfig();
            return;
        }

        const bool plus = (idx == 1);
        if (menuIndex == 0)
        {
            cfg.masterOn = plus;
            evaluateBoiler();
            touchConfig();
            return;
        }

        if (menuIndex == 1)
            cfg.spHeater = constrain(cfg.spHeater + (plus ? 1.0f : -1.0f), 40.0f, 60.0f);
        else if (menuIndex == 2)
            cfg.spPump = constrain(cfg.spPump + (plus ? 1.0f : -1.0f), 20.0f, 60.0f);
        else if (menuIndex == 3)
            cfg.spRoom = constrain(roundf((cfg.spRoom + (plus ? 0.5f : -0.5f)) * 10.0f) / 10.0f, 16.0f, 30.0f);
        else if (menuIndex == 4)
            cfg.diffHeater = constrain(cfg.diffHeater + (plus ? 1.0f : -1.0f), 1.0f, 10.0f);
        else if (menuIndex == 5)
            cfg.diffPump = constrain(cfg.diffPump + (plus ? 1.0f : -1.0f), 1.0f, 10.0f);
        else if (menuIndex == 6)
            cfg.diffRoom = constrain(roundf((cfg.diffRoom + (plus ? 0.1f : -0.1f)) * 10.0f) / 10.0f, 0.2f, 3.0f);

        evaluateBoiler();
        touchConfig();
    }

    void pollButtons(unsigned long now)
    {
        for (int i = 0; i < 4; i++)
        {
            if (btns[i].pin < 0)
                continue;
            const bool raw = (digitalRead(btns[i].pin) != LOW);
            if (raw != btns[i].lastRaw)
            {
                btns[i].lastRaw = raw;
                btns[i].lastEdgeMs = now;
            }
            if ((now - btns[i].lastEdgeMs) >= kDebounceMs && raw != btns[i].stable)
            {
                btns[i].stable = raw;
                if (!raw) // pressed (LOW)
                    handleButtonPress(i, now);
            }
        }
    }

    // ---------------- 1-Wire DS18B20 ----------------
    void teardownDallas()
    {
        if (dallas)
        {
            delete dallas;
            dallas = nullptr;
        }
        if (oneWire)
        {
            delete oneWire;
            oneWire = nullptr;
        }
        convPending = false;
        probeCount = 0;
    }

    void initDallas()
    {
        teardownDallas();
        if (cfg.dallasPin < 0)
            return;
        pinMode(cfg.dallasPin, INPUT_PULLUP);
        oneWire = new OneWire((uint8_t)cfg.dallasPin);
        dallas = new DallasTemperature(oneWire);
        dallas->begin();
        dallas->setResolution(12);
        dallas->setWaitForConversion(false);
        lastReadMs = 0;
        scanRequested = true;
    }

    float sanitizeTemp(float t)
    {
        if (isnan(t) || t <= -55.0f || t >= 125.0f)
            return NAN;
        return roundf(t * 10.0f) / 10.0f;
    }

    void startConversion(unsigned long now)
    {
        if (!dallas)
            return;
        dallas->begin();
        dallas->setResolution(12);
        dallas->setWaitForConversion(false);
        dallas->requestTemperatures();
        convPending = true;
        convStartMs = now;
        lastReadMs = now;
    }

    void finishConversion()
    {
        convPending = false;
        if (!dallas)
            return;

        const int cnt = ((int)dallas->getDeviceCount() > kMaxProbes ? kMaxProbes : (int)dallas->getDeviceCount());
        probeCount = 0;
        for (int i = 0; i < cnt; i++)
        {
            uint8_t addr[8];
            if (dallas->getAddress(addr, i))
            {
                memcpy(probes[probeCount].addr, addr, 8);
                probes[probeCount].temp = sanitizeTemp(dallas->getTempC(addr));
                probeCount++;
            }
        }

        // Auto-assign ROMs only if unassigned
        if (romIsEmpty(cfg.romWater) && probeCount >= 1)
        {
            memcpy(cfg.romWater, probes[0].addr, 8);
            needsSave = true;
        }
        if (romIsEmpty(cfg.romReturn) && probeCount >= 2)
        {
            for (int i = 0; i < probeCount; i++)
            {
                if (!romEqual(probes[i].addr, cfg.romWater))
                {
                    memcpy(cfg.romReturn, probes[i].addr, 8);
                    needsSave = true;
                    break;
                }
            }
        }

        float newW = NAN;
        float newR = NAN;
        if (!romIsEmpty(cfg.romWater))
            newW = sanitizeTemp(dallas->getTempC(cfg.romWater));
        if (!romIsEmpty(cfg.romReturn))
            newR = sanitizeTemp(dallas->getTempC(cfg.romReturn));

        const bool changed =
            (isnan(newW) != isnan(tempWater)) || (!isnan(newW) && fabsf(newW - tempWater) >= 0.1f) ||
            (isnan(newR) != isnan(tempReturn)) || (!isnan(newR) && fabsf(newR - tempReturn) >= 0.1f);

        tempWater = newW;
        tempReturn = newR;

        if (isnan(tempWater) && (pumpOn || heaterOn))
        {
            deviceLog("caldeira: sonda agua em falha (NaN), reles desligados");
        }
        evaluateBoiler();
        if (changed)
            needsPublish = true;
    }

    // ---------------- Persistence (/boiler.json) ----------------
    void saveBoiler()
    {
        if (!created)
            return;
        JsonDocument doc;
        doc["name"] = cfg.name;
        doc["sda"] = cfg.sdaPin;
        doc["scl"] = cfg.sclPin;
        doc["dallas"] = cfg.dallasPin;
        doc["pump"] = cfg.pumpPin;
        doc["heater"] = cfg.heaterPin;
        doc["btnMenu"] = cfg.btnMenuPin;
        doc["btnPlus"] = cfg.btnPlusPin;
        doc["btnMinus"] = cfg.btnMinusPin;
        doc["btnPower"] = cfg.btnPowerPin;
        doc["lcdAddr"] = cfg.lcdAddr;
        char hw[17], hr[17];
        romToHex(cfg.romWater, hw, sizeof(hw));
        romToHex(cfg.romReturn, hr, sizeof(hr));
        doc["romW"] = hw;
        doc["romR"] = hr;
        doc["hw"] = cfg.heaterWatts;
        doc["pw"] = cfg.pumpWatts;
        doc["spH"] = cfg.spHeater;
        doc["spP"] = cfg.spPump;
        doc["spR"] = cfg.spRoom;
        doc["dH"] = cfg.diffHeater;
        doc["dP"] = cfg.diffPump;
        doc["dR"] = cfg.diffRoom;
        doc["on"] = cfg.masterOn;
        doc["rt"] = cfg.roomTemp;
        doc["secH"] = cfg.secHeaterToday;
        doc["secP"] = cfg.secPumpToday;

        File f = LittleFS.open(kBoilerTmp, "w");
        if (!f)
            return;
        serializeJson(doc, f);
        f.close();
        LittleFS.remove(kBoilerFile);
        LittleFS.rename(kBoilerTmp, kBoilerFile);
    }

    void loadBoiler()
    {
        if (!LittleFS.exists(kBoilerFile))
        {
            created = false;
            return;
        }
        File f = LittleFS.open(kBoilerFile, "r");
        if (!f)
        {
            created = false;
            return;
        }
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, f);
        f.close();
        if (err)
        {
            created = false;
            return;
        }

        cfg = BoilerConfig();
        strlcpy(cfg.name, doc["name"] | "Caldeira", sizeof(cfg.name));
        cfg.sdaPin = doc["sda"] | DefaultPins::SDA;
        cfg.sclPin = doc["scl"] | DefaultPins::SCL;
        cfg.dallasPin = doc["dallas"] | cfg.dallasPin;
        cfg.pumpPin = doc["pump"] | cfg.pumpPin;
        cfg.heaterPin = doc["heater"] | cfg.heaterPin;
        cfg.btnMenuPin = doc["btnMenu"] | cfg.btnMenuPin;
        cfg.btnPlusPin = doc["btnPlus"] | cfg.btnPlusPin;
        cfg.btnMinusPin = doc["btnMinus"] | cfg.btnMinusPin;
        cfg.btnPowerPin = doc["btnPower"] | cfg.btnPowerPin;
        cfg.lcdAddr = (uint8_t)constrain((int)(doc["lcdAddr"] | 0x27), 0x08, 0x77);
        if (doc["romW"].is<const char *>())
            hexToRom(doc["romW"].as<const char *>(), cfg.romWater);
        if (doc["romR"].is<const char *>())
            hexToRom(doc["romR"].as<const char *>(), cfg.romReturn);
        cfg.heaterWatts = (uint16_t)constrain((int)(doc["hw"] | 6500), 0, 30000);
        cfg.pumpWatts = (uint16_t)constrain((int)(doc["pw"] | 70), 0, 5000);
        cfg.spHeater = constrain((float)(doc["spH"] | 50.0f), 40.0f, 60.0f);
        cfg.spPump = constrain((float)(doc["spP"] | 30.0f), 20.0f, 60.0f);
        cfg.spRoom = constrain((float)(doc["spR"] | 21.0f), 16.0f, 30.0f);
        cfg.diffHeater = constrain((float)(doc["dH"] | 3.0f), 1.0f, 10.0f);
        cfg.diffPump = constrain((float)(doc["dP"] | 3.0f), 1.0f, 10.0f);
        cfg.diffRoom = constrain((float)(doc["dR"] | 0.5f), 0.2f, 3.0f);
        cfg.masterOn = doc["on"] | false;
        cfg.roomTemp = constrain((float)(doc["rt"] | 20.0f), 0.0f, 50.0f);
        cfg.secHeaterToday = doc["secH"] | 0u;
        cfg.secPumpToday = doc["secP"] | 0u;

        created = true;
    }

    void initHardware()
    {
        pumpOn = false;
        heaterOn = false;
        applyOutputs();
        initButtons();
        initDallas();
        lcdInit();
#ifdef ESP32
        WiFi.setSleep(false);
        espNowEnsureInit(0);
#endif
        hwInitDone = true;
        needsDiscovery = true;
        needsPublish = true;
    }

    void deinitHardware()
    {
        pumpOn = false;
        heaterOn = false;
        applyOutputs();
        if (cfg.pumpPin >= 0)
            pinMode(cfg.pumpPin, INPUT);
        if (cfg.heaterPin >= 0)
            pinMode(cfg.heaterPin, INPUT);
        teardownDallas();
        if (lcdAvailable)
        {
            lcdCommand(0x01);
            delay(2);
            lcdCommand(0x08);
            lcdAvailable = false;
        }
        hwInitDone = false;
        needsSave = false;
        needsUndiscovery = true;
    }

    // ---------------- MQTT State & HA Discovery ----------------
    float heaterKwh() { return (float)(((double)cfg.secHeaterToday * (double)cfg.heaterWatts) / 3600000.0); }
    float pumpKwh() { return (float)(((double)cfg.secPumpToday * (double)cfg.pumpWatts) / 3600000.0); }
    float totalWattsNow() { return (heaterOn ? (float)cfg.heaterWatts : 0.0f) + (pumpOn ? (float)cfg.pumpWatts : 0.0f); }

    void publishBoilerState()
    {
        if (!created || !mqttConnected())
            return;
        JsonDocument doc;
        doc["on"] = cfg.masterOn;
        doc["pump"] = pumpOn;
        doc["heater"] = heaterOn;
        if (!isnan(tempWater))
            doc["tw"] = roundf(tempWater * 10.0f) / 10.0f;
        if (!isnan(tempReturn))
            doc["tr"] = roundf(tempReturn * 10.0f) / 10.0f;
        if (!isnan(cfg.roomTemp))
            doc["ts"] = roundf(cfg.roomTemp * 10.0f) / 10.0f;
        doc["ph"] = heaterOn ? cfg.heaterWatts : 0;
        doc["pp"] = pumpOn ? cfg.pumpWatts : 0;
        doc["pwr"] = (int)totalWattsNow();
        doc["kw"] = roundf((totalWattsNow() / 1000.0f) * 100.0f) / 100.0f;
        const float er = heaterKwh();
        const float eb = pumpKwh();
        doc["er"] = roundf(er * 1000.0f) / 1000.0f;
        doc["eb"] = roundf(eb * 10000.0f) / 10000.0f;
        doc["et"] = roundf((er + eb) * 1000.0f) / 1000.0f;
        doc["sr"] = roundf(cfg.spHeater);
        doc["sb"] = roundf(cfg.spPump);
        doc["ss"] = roundf(cfg.spRoom * 10.0f) / 10.0f;
        doc["dr"] = roundf(cfg.diffHeater);
        doc["db"] = roundf(cfg.diffPump);
        doc["ds"] = roundf(cfg.diffRoom * 10.0f) / 10.0f;

        String out;
        serializeJson(doc, out);
        publishOnMqtt(tState, out.c_str(), true);
        lastPublishMs = millis();
        needsPublish = false;
    }

    void haDevice(JsonObject &o)
    {
        JsonObject d = o["dev"].to<JsonObject>();
        d["ids"] = "OnOfre-" + String(config.chipId);
        d["name"] = config.nodeId;
        d["mf"] = "OnOfre Portugal";
        d["sw"] = String(VERSION);
    }

    void publishCfg(const char *domain, const String &uid, JsonDocument &doc)
    {
        String out;
        serializeJson(doc, out);
        publishOnMqtt((String(constantsMqtt::homeAssistantAutoDiscoveryPrefix) + "/" + domain + "/" + uid + "/config").c_str(),
                      out.c_str(), true);
        yield();
        delay(5);
    }

    void publishHaSensor(const String &base, const String &chip, const char *suffix, const char *name,
                         const char *valTpl, const char *unit, const char *devCla, const char *statCla, const char *icon)
    {
        JsonDocument doc;
        JsonObject o = doc.to<JsonObject>();
        o["~"] = base;
        o["name"] = name;
        o["uniq_id"] = chip + suffix;
        o["has_entity_name"] = true;
        o["stat_t"] = "~/state";
        o["avty_t"] = config.healthTopic;
        o["val_tpl"] = valTpl;
        if (unit)
            o["unit_of_meas"] = unit;
        if (devCla)
            o["dev_cla"] = devCla;
        if (statCla)
            o["stat_cla"] = statCla;
        if (icon)
            o["ic"] = icon;
        haDevice(o);
        publishCfg("sensor", chip + suffix, doc);
    }

    void publishHaBinary(const String &base, const String &chip, const char *suffix, const char *name,
                         const char *valTpl, const char *icon)
    {
        JsonDocument doc;
        JsonObject o = doc.to<JsonObject>();
        o["~"] = base;
        o["name"] = name;
        o["uniq_id"] = chip + suffix;
        o["has_entity_name"] = true;
        o["stat_t"] = "~/state";
        o["avty_t"] = config.healthTopic;
        o["val_tpl"] = valTpl;
        o["pl_on"] = "ON";
        o["pl_off"] = "OFF";
        o["ic"] = icon;
        haDevice(o);
        publishCfg("binary_sensor", chip + suffix, doc);
    }

    void publishHaNumber(const String &base, const String &chip, const char *suffix, const char *name,
                         const char *cmdKey, const char *jsonKey, float minV, float maxV, float stepV, const char *icon)
    {
        JsonDocument doc;
        JsonObject o = doc.to<JsonObject>();
        o["~"] = base;
        o["name"] = name;
        o["uniq_id"] = chip + suffix;
        o["has_entity_name"] = true;
        o["cmd_t"] = String("~/num/") + cmdKey + "/set";
        o["stat_t"] = "~/state";
        o["avty_t"] = config.healthTopic;
        o["val_tpl"] = String("{{ value_json.") + jsonKey + " }}";
        o["min"] = minV;
        o["max"] = maxV;
        o["step"] = stepV;
        o["unit_of_meas"] = "°C";
        o["ic"] = icon;
        haDevice(o);
        publishCfg("number", chip + suffix, doc);
    }

    void undiscoverBoiler()
    {
        const String chip = String(config.chipId);
        const char *doms[] = {
            "switch", "binary_sensor", "binary_sensor",
            "sensor", "sensor", "sensor", "sensor", "sensor", "sensor", "sensor", "sensor", "sensor",
            "number", "number", "number", "number", "number", "number", "number", "button"};
        const char *ids[] = {
            "_boiler_master", "_boiler_pump", "_boiler_heater",
            "_boiler_tw", "_boiler_tr", "_boiler_ts", "_boiler_ph", "_boiler_pp", "_boiler_pwr",
            "_boiler_er", "_boiler_eb", "_boiler_et",
            "_boiler_sr", "_boiler_sb", "_boiler_ss", "_boiler_dr", "_boiler_db", "_boiler_ds",
            "_boiler_room_in", "_boiler_reset"};
        for (size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); i++)
        {
            publishOnMqtt((String(constantsMqtt::homeAssistantAutoDiscoveryPrefix) + "/" + doms[i] + "/" + chip + ids[i] + "/config").c_str(), "", true);
            yield();
            delay(3);
        }
        publishOnMqtt(tState, "", true);
    }

    void applyJsonCommand(JsonVariantConst doc)
    {
        bool changed = false;
        if (doc["state"].is<const char *>())
        {
            const char *s = doc["state"];
            cfg.masterOn = (strcasecmp(s, "ON") == 0 || strcmp(s, "1") == 0 || strcasecmp(s, "true") == 0);
            changed = true;
        }
        if (doc["on"].is<bool>())
        {
            cfg.masterOn = doc["on"].as<bool>();
            changed = true;
        }
        if (doc["spHeater"].is<float>() || doc["spHeater"].is<int>())
        {
            cfg.spHeater = constrain(roundf(doc["spHeater"].as<float>()), 40.0f, 60.0f);
            changed = true;
        }
        if (doc["spPump"].is<float>() || doc["spPump"].is<int>())
        {
            cfg.spPump = constrain(roundf(doc["spPump"].as<float>()), 20.0f, 60.0f);
            changed = true;
        }
        if (doc["spRoom"].is<float>() || doc["spRoom"].is<int>())
        {
            cfg.spRoom = constrain(roundf(doc["spRoom"].as<float>() * 10.0f) / 10.0f, 16.0f, 30.0f);
            changed = true;
        }
        if (doc["diffHeater"].is<float>() || doc["diffHeater"].is<int>())
        {
            cfg.diffHeater = constrain(roundf(doc["diffHeater"].as<float>()), 1.0f, 10.0f);
            changed = true;
        }
        if (doc["diffPump"].is<float>() || doc["diffPump"].is<int>())
        {
            cfg.diffPump = constrain(roundf(doc["diffPump"].as<float>()), 1.0f, 10.0f);
            changed = true;
        }
        if (doc["diffRoom"].is<float>() || doc["diffRoom"].is<int>())
        {
            cfg.diffRoom = constrain(roundf(doc["diffRoom"].as<float>() * 10.0f) / 10.0f, 0.2f, 3.0f);
            changed = true;
        }
        if (doc["roomTemp"].is<float>() || doc["roomTemp"].is<int>())
        {
            cfg.roomTemp = constrain(roundf(doc["roomTemp"].as<float>() * 10.0f) / 10.0f, 0.0f, 50.0f);
            changed = true;
        }
        if (doc["resetEnergy"] | false)
        {
            cfg.secHeaterToday = 0;
            cfg.secPumpToday = 0;
            changed = true;
        }
        if (doc["scan"] | false)
            scanRequested = true;

        if (changed)
        {
            evaluateBoiler();
            touchConfig();
        }
    }
} // namespace

void boilerOnEspNowPacket(const uint8_t *mac, const EspNowRoomPacket &pkt)
{
    if (!created || !mac || espNowRxPending)
        return;
    memcpy(espNowRxMacStaging, mac, 6);
    espNowRxPktStaging = pkt;
    __sync_synchronize();
    espNowRxPending = true;
}

void boilerSetup()
{
    initTopics();
    loadBoiler();
    if (created)
        initHardware();
}

void boilerLoop()
{
    const unsigned long now = millis();
    if (created && !hwInitDone)
        initHardware();
    if (!created && hwInitDone)
    {
        deinitHardware();
        return;
    }
    if (!created)
    {
        if (needsUndiscovery && mqttConnected() && config.tryBeginFeatureAccess("boilerUndisc"))
        {
            undiscoverBoiler();
            config.endFeatureAccess();
            needsUndiscovery = false;
        }
        return;
    }

    if (pendingFlag)
    {
        char local[sizeof(pendingBuf)];
        memcpy(local, pendingBuf, sizeof(local));
        pendingFlag = false;
        JsonDocument doc;
        if (!deserializeJson(doc, local))
            applyJsonCommand(doc.as<JsonVariantConst>());
    }

    pollButtons(now);

    if (espNowRxPending)
    {
        EspNowRoomPacket pkt = espNowRxPktStaging;
        uint8_t srcMac[6];
        memcpy(srcMac, espNowRxMacStaging, 6);
        __sync_synchronize();
        espNowRxPending = false;

        if (!isnan(pkt.roomTemp) && pkt.roomTemp >= 0.0f && pkt.roomTemp <= 50.0f)
        {
            const float newRoom = roundf(pkt.roomTemp * 10.0f) / 10.0f;
            const bool roomChanged = isnan(cfg.roomTemp) || fabsf(newRoom - cfg.roomTemp) >= 0.1f;
            cfg.roomTemp = newRoom;
            if ((pkt.flags & 0x01) != 0 && !isnan(pkt.spRoom) && pkt.spRoom >= 16.0f && pkt.spRoom <= 30.0f)
            {
                const float newSp = constrain(roundf(pkt.spRoom * 10.0f) / 10.0f, 16.0f, 30.0f);
                if (fabsf(newSp - cfg.spRoom) >= 0.05f)
                {
                    cfg.spRoom = newSp;
                    needsSave = true;
                    lastChangeMs = now;
                }
            }
            if (!isnan(pkt.humidity) && pkt.humidity >= 0.0f && pkt.humidity <= 100.0f)
                lastEspNowRoomHum = roundf(pkt.humidity * 10.0f) / 10.0f;
            memcpy(lastEspNowSenderMac, srcMac, 6);
            lastEspNowRxMs = now;
            espNowRxCount++;
            evaluateBoiler();
            lcdForceRedraw = true;
            if (roomChanged)
            {
                needsPublish = true;
                needsSave = true;
                lastChangeMs = now;
            }
            char macStr[20];
            espNowFormatMac(srcMac, macStr, sizeof(macStr));
            deviceLog("caldeira: temp sala ESP-NOW %.1fC (%s)", cfg.roomTemp, macStr);

            EspNowRoomPacket ack;
            memset(&ack, 0, sizeof(ack));
            ack.magic = kEspNowPacketMagic;
            ack.version = kEspNowPacketVersion;
            ack.msgType = kEspNowMsgBoilerAck;
            ack.roomTemp = tempWater;
            ack.humidity = lastEspNowRoomHum;
            ack.spRoom = cfg.spRoom;
            ack.flags = (cfg.masterOn ? 0x04 : 0) | (pumpOn ? 0x08 : 0) | (heaterOn ? 0x10 : 0);
            ack.seq = pkt.seq;
            espNowSendPacket(srcMac, 0, ack);
        }
    }

    if (menuIndex != 0 && (now - menuActivityMs) >= kMenuTimeoutMs)
    {
        menuIndex = 0;
        lcdForceRedraw = true;
    }

    // 1-second energy counter + midnight reset
    if (lastSecMs == 0)
        lastSecMs = now;
    if ((now - lastSecMs) >= 1000)
    {
        const unsigned long ticks = (now - lastSecMs) / 1000;
        lastSecMs += ticks * 1000;
        if (heaterOn)
            cfg.secHeaterToday += (uint32_t)ticks;
        if (pumpOn)
            cfg.secPumpToday += (uint32_t)ticks;

        if (clockSynced())
        {
            const int m = clockMinuteOfDay();
            if (lastMinOfDay >= 0 && m >= 0 && m < lastMinOfDay)
            {
                cfg.secHeaterToday = 0;
                cfg.secPumpToday = 0;
                needsSave = true;
                needsPublish = true;
                deviceLog("caldeira: contadores diarios reiniciados a meia-noite");
            }
            lastMinOfDay = m;
        }
    }

    // Non-blocking DS18B20 conversion cycle
    if (!convPending && (scanRequested || lastReadMs == 0 || (now - lastReadMs) >= kReadIntervalMs))
    {
        scanRequested = false;
        startConversion(now);
    }
    if (convPending && (now - convStartMs) >= kConvWaitMs)
        finishConversion();

    if ((now - lastLcdMs) >= kLcdIntervalMs || lcdForceRedraw)
    {
        lastLcdMs = now;
        lcdRender();
    }

    if ((needsPublish || needsDiscovery || (now - lastPublishMs) >= 30000) &&
        mqttConnected() && config.tryBeginFeatureAccess("boilerMqtt"))
    {
        if (needsDiscovery)
        {
            boilerSubscribe();
            boilerDiscovery();
            needsDiscovery = false;
        }
        else if (needsPublish || (now - lastPublishMs) >= kMqttPublishMinMs)
        {
            publishBoilerState();
        }
        config.endFeatureAccess();
    }

    if (needsSave && (now - lastChangeMs) >= kSaveDelayMs)
    {
        needsSave = false;
        saveBoiler();
    }
}

bool boilerHandleMqtt(const char *topic, const char *payload)
{
    if (!created || !topic || !payload)
        return false;

    if (strcmp(topic, tSet) == 0)
    {
        if (payload[0] == '{')
        {
            JsonDocument doc;
            if (!deserializeJson(doc, payload))
                applyJsonCommand(doc.as<JsonVariantConst>());
        }
        else
        {
            cfg.masterOn = (strcasecmp(payload, "ON") == 0 || strcmp(payload, "1") == 0);
            evaluateBoiler();
            touchConfig();
        }
        publishBoilerState();
        return true;
    }

    if (strcmp(topic, tResetSet) == 0)
    {
        cfg.secHeaterToday = 0;
        cfg.secPumpToday = 0;
        touchConfig();
        publishBoilerState();
        return true;
    }

    char prefix[64];
    snprintf(prefix, sizeof(prefix), "onofre/%s/boiler/num/", config.chipId);
    const size_t plen = strlen(prefix);
    if (strncmp(topic, prefix, plen) == 0)
    {
        const char *key = topic + plen;
        const float v = atof(payload);
        if (strncmp(key, "sp_res/", 7) == 0)
            cfg.spHeater = constrain(roundf(v), 40.0f, 60.0f);
        else if (strncmp(key, "sp_bomba/", 9) == 0)
            cfg.spPump = constrain(roundf(v), 20.0f, 60.0f);
        else if (strncmp(key, "sp_sala/", 8) == 0)
            cfg.spRoom = constrain(roundf(v * 10.0f) / 10.0f, 16.0f, 30.0f);
        else if (strncmp(key, "dif_res/", 8) == 0)
            cfg.diffHeater = constrain(roundf(v), 1.0f, 10.0f);
        else if (strncmp(key, "dif_bomba/", 10) == 0)
            cfg.diffPump = constrain(roundf(v), 1.0f, 10.0f);
        else if (strncmp(key, "dif_sala/", 9) == 0)
            cfg.diffRoom = constrain(roundf(v * 10.0f) / 10.0f, 0.2f, 3.0f);
        else if (strncmp(key, "temp_sala/", 10) == 0)
            cfg.roomTemp = constrain(roundf(v * 10.0f) / 10.0f, 0.0f, 50.0f);
        else
            return false;

        evaluateBoiler();
        touchConfig();
        publishBoilerState();
        return true;
    }
    return false;
}

void boilerSubscribe()
{
    if (!created || !mqttConnected())
        return;
    subscribeOnMqtt(tSet);
    subscribeOnMqtt(tNumSet);
    subscribeOnMqtt(tResetSet);
}

void boilerDiscovery()
{
    if (!created || !mqttConnected())
        return;
    const String chip = String(config.chipId);
    const String base = String("onofre/") + chip + "/boiler";

    {
        JsonDocument doc;
        JsonObject o = doc.to<JsonObject>();
        o["~"] = base;
        o["name"] = String(cfg.name) + " Geral";
        o["uniq_id"] = chip + "_boiler_master";
        o["has_entity_name"] = true;
        o["cmd_t"] = "~/set";
        o["stat_t"] = "~/state";
        o["avty_t"] = config.healthTopic;
        o["pl_on"] = "ON";
        o["pl_off"] = "OFF";
        o["val_tpl"] = "{{ 'ON' if value_json.on else 'OFF' }}";
        o["ic"] = "mdi:water-boiler";
        haDevice(o);
        publishCfg("switch", chip + "_boiler_master", doc);
    }
    publishHaBinary(base, chip, "_boiler_pump", "Bomba", "{{ 'ON' if value_json.pump else 'OFF' }}", "mdi:pump");
    publishHaBinary(base, chip, "_boiler_heater", "Resistência", "{{ 'ON' if value_json.heater else 'OFF' }}", "mdi:heating-coil");

    publishHaSensor(base, chip, "_boiler_tw", "Temperatura Água", "{{ value_json.tw | default('unavailable') }}", "°C", "temperature", "measurement", "mdi:thermometer-water");
    publishHaSensor(base, chip, "_boiler_tr", "Temperatura Água Retorno", "{{ value_json.tr | default('unavailable') }}", "°C", "temperature", "measurement", "mdi:thermometer");
    publishHaSensor(base, chip, "_boiler_ts", "Temperatura Sala", "{{ value_json.ts | default(20) }}", "°C", "temperature", "measurement", "mdi:home-thermometer-outline");
    publishHaSensor(base, chip, "_boiler_ph", "Potência Resistência", "{{ value_json.ph }}", "W", "power", "measurement", "mdi:flash");
    publishHaSensor(base, chip, "_boiler_pp", "Potência Bomba", "{{ value_json.pp }}", "W", "power", "measurement", "mdi:flash");
    publishHaSensor(base, chip, "_boiler_pwr", "Potência Total Caldeira", "{{ value_json.pwr }}", "W", "power", "measurement", "mdi:lightning-bolt");
    publishHaSensor(base, chip, "_boiler_er", "Energia Diária Resistência", "{{ value_json.er }}", "kWh", "energy", "total_increasing", "mdi:heating-coil");
    publishHaSensor(base, chip, "_boiler_eb", "Energia Diária Bomba", "{{ value_json.eb }}", "kWh", "energy", "total_increasing", "mdi:pump");
    publishHaSensor(base, chip, "_boiler_et", "Energia Diária Total", "{{ value_json.et }}", "kWh", "energy", "total_increasing", "mdi:counter");

    publishHaNumber(base, chip, "_boiler_sr", "Setpoint Resistência", "sp_res", "sr", 40, 60, 1, "mdi:thermometer-high");
    publishHaNumber(base, chip, "_boiler_sb", "Setpoint Bomba", "sp_bomba", "sb", 20, 60, 1, "mdi:pump");
    publishHaNumber(base, chip, "_boiler_ss", "Setpoint Sala", "sp_sala", "ss", 16, 30, 0.5f, "mdi:thermostat");
    publishHaNumber(base, chip, "_boiler_dr", "Diferencial Resistência", "dif_res", "dr", 1, 10, 1, "mdi:plus-minus");
    publishHaNumber(base, chip, "_boiler_db", "Diferencial Bomba", "dif_bomba", "db", 1, 10, 1, "mdi:plus-minus");
    publishHaNumber(base, chip, "_boiler_ds", "Diferencial Sala", "dif_sala", "ds", 0.2f, 3.0f, 0.1f, "mdi:plus-minus");
    publishHaNumber(base, chip, "_boiler_room_in", "Temperatura Interior Sala", "temp_sala", "ts", 0, 50, 0.1f, "mdi:home-thermometer");

    {
        JsonDocument doc;
        JsonObject o = doc.to<JsonObject>();
        o["~"] = base;
        o["name"] = "Reset Contadores Energia";
        o["uniq_id"] = chip + "_boiler_reset";
        o["has_entity_name"] = true;
        o["cmd_t"] = "~/reset/set";
        o["pl_prs"] = "PRESS";
        o["avty_t"] = config.healthTopic;
        o["ic"] = "mdi:restore";
        haDevice(o);
        publishCfg("button", chip + "_boiler_reset", doc);
    }
    publishBoilerState();
}

int boilerCreateDefault()
{
    cfg = BoilerConfig();
    // Pre-seed the 2 DS18B20 ROM addresses from the ESPHome central heating config
    hexToRom("be0b255482876428", cfg.romWater);
    hexToRom("0a01204f92110628", cfg.romReturn);
    created = true;
    saveBoiler();
    return 0;
}

static int validateAndApplyPins(const BoilerConfig &cand)
{
    if (!cand.name[0])
        return 1;
    if (!validBusOrOutPin(cand.sdaPin) || !validBusOrOutPin(cand.sclPin) ||
        cand.sdaPin == cand.sclPin)
        return 2;
    if (!validBusOrOutPin(cand.dallasPin))
        return 2;
    if (!validBusOrOutPin(cand.pumpPin))
        return 2;
    if (!validBusOrOutPin(cand.heaterPin))
        return 2;
    if (!validBtnPin(cand.btnMenuPin) || !validBtnPin(cand.btnPlusPin) ||
        !validBtnPin(cand.btnMinusPin) || !validBtnPin(cand.btnPowerPin))
        return 2;

    // Internal duplicate check across Boiler's own pins
    const int pins[9] = {
        cand.sdaPin, cand.sclPin, cand.dallasPin, cand.pumpPin, cand.heaterPin,
        cand.btnMenuPin, cand.btnPlusPin, cand.btnMinusPin, cand.btnPowerPin};
    for (int i = 0; i < 9; i++)
    {
        if (pins[i] < 0 || pins[i] == (int)DefaultPins::noGPIO)
            continue;
        for (int j = i + 1; j < 9; j++)
        {
            if (pins[i] == pins[j])
                return 5;
        }
    }

    // External clash check against Actuators / Sensors / LedStrip
    pinCheckBypass = true;
    bool externalClash = false;
    for (int i = 0; i < 9; i++)
    {
        if (pins[i] < 0 || pins[i] == (int)DefaultPins::noGPIO)
            continue;
        if (i < 2 && (pins[i] == (int)DefaultPins::SDA || pins[i] == (int)DefaultPins::SCL))
            continue;
        if (config.pinInUse((unsigned int)pins[i]))
        {
            externalClash = true;
            break;
        }
    }
    pinCheckBypass = false;
    return externalClash ? 5 : 0;
}

int boilerCreate(JsonVariantConst json)
{
    if (created)
        return 7;
    BoilerConfig cand;
    if (json["name"].is<const char *>())
        strlcpy(cand.name, json["name"], sizeof(cand.name));
    if (json["sda"].is<int>()) cand.sdaPin = json["sda"].as<int>();
    if (json["scl"].is<int>()) cand.sclPin = json["scl"].as<int>();
    if (json["input1"].is<int>()) cand.dallasPin = json["input1"].as<int>();
    if (json["input2"].is<int>()) cand.pumpPin = json["input2"].as<int>();
    if (json["dallas"].is<int>()) cand.dallasPin = json["dallas"].as<int>();
    if (json["pump"].is<int>()) cand.pumpPin = json["pump"].as<int>();
    if (json["heater"].is<int>()) cand.heaterPin = json["heater"].as<int>();
    if (json["btnMenu"].is<int>()) cand.btnMenuPin = json["btnMenu"].as<int>();
    if (json["btnPlus"].is<int>()) cand.btnPlusPin = json["btnPlus"].as<int>();
    if (json["btnMinus"].is<int>()) cand.btnMinusPin = json["btnMinus"].as<int>();
    if (json["btnPower"].is<int>()) cand.btnPowerPin = json["btnPower"].as<int>();
    if (json["lcdAddr"].is<int>()) cand.lcdAddr = (uint8_t)constrain(json["lcdAddr"].as<int>(), 0x08, 0x77);
    if (json["romW"].is<const char *>()) hexToRom(json["romW"], cand.romWater);
    if (json["romR"].is<const char *>()) hexToRom(json["romR"], cand.romReturn);

    const int err = validateAndApplyPins(cand);
    if (err != 0)
        return err;

    cfg = cand;
    created = true;
    saveBoiler();
    initHardware();
    return 0;
}

int boilerUpdateHardware(JsonVariantConst json)
{
    if (!created)
        return 1;
    BoilerConfig cand = cfg;
    if (json["name"].is<const char *>())
        strlcpy(cand.name, json["name"], sizeof(cand.name));
    if (json["sda"].is<int>()) cand.sdaPin = json["sda"].as<int>();
    if (json["scl"].is<int>()) cand.sclPin = json["scl"].as<int>();
    if (json["dallas"].is<int>()) cand.dallasPin = json["dallas"].as<int>();
    if (json["pump"].is<int>()) cand.pumpPin = json["pump"].as<int>();
    if (json["heater"].is<int>()) cand.heaterPin = json["heater"].as<int>();
    if (json["btnMenu"].is<int>()) cand.btnMenuPin = json["btnMenu"].as<int>();
    if (json["btnPlus"].is<int>()) cand.btnPlusPin = json["btnPlus"].as<int>();
    if (json["btnMinus"].is<int>()) cand.btnMinusPin = json["btnMinus"].as<int>();
    if (json["btnPower"].is<int>()) cand.btnPowerPin = json["btnPower"].as<int>();
    if (json["lcdAddr"].is<int>()) cand.lcdAddr = (uint8_t)constrain(json["lcdAddr"].as<int>(), 0x08, 0x77);
    if (json["heaterWatts"].is<int>()) cand.heaterWatts = (uint16_t)constrain(json["heaterWatts"].as<int>(), 0, 30000);
    if (json["pumpWatts"].is<int>()) cand.pumpWatts = (uint16_t)constrain(json["pumpWatts"].as<int>(), 0, 5000);
    if (json["romW"].is<const char *>())
    {
        if (strlen(json["romW"]) == 0)
            memset(cand.romWater, 0, 8);
        else
            hexToRom(json["romW"], cand.romWater);
    }
    if (json["romR"].is<const char *>())
    {
        if (strlen(json["romR"]) == 0)
            memset(cand.romReturn, 0, 8);
        else
            hexToRom(json["romR"], cand.romReturn);
    }

    const int err = validateAndApplyPins(cand);
    if (err != 0)
        return err;

    // Release previous relay GPIOs cleanly if moved
    if (cfg.pumpPin >= 0 && cfg.pumpPin != cand.pumpPin)
    {
        digitalWrite(cfg.pumpPin, LOW);
        pinMode(cfg.pumpPin, INPUT);
    }
    if (cfg.heaterPin >= 0 && cfg.heaterPin != cand.heaterPin)
    {
        digitalWrite(cfg.heaterPin, LOW);
        pinMode(cfg.heaterPin, INPUT);
    }

    cfg = cand;
    saveBoiler();
    initHardware();
    evaluateBoiler();
    return 0;
}

void boilerRemove()
{
    created = false;
    pendingFlag = false;
    needsSave = false;
    LittleFS.remove(kBoilerFile);
}

bool boilerIsCreated()
{
    return (bool)created;
}

bool boilerUsesPin(unsigned int pin)
{
    if (!created || pinCheckBypass)
        return false;
    const int p = (int)pin;
    if (p == cfg.dallasPin || p == cfg.pumpPin || p == cfg.heaterPin ||
        p == cfg.btnMenuPin || p == cfg.btnPlusPin || p == cfg.btnMinusPin || p == cfg.btnPowerPin)
        return true;
    if (p == cfg.sdaPin && p != (int)DefaultPins::SDA)
        return true;
    if (p == cfg.sclPin && p != (int)DefaultPins::SCL)
        return true;
    return false;
}

void boilerConfigJson(JsonVariant root)
{
    if (!created)
        return;
    JsonObject o = root["boiler"].to<JsonObject>();
    o["name"] = cfg.name;
    o["sda"] = cfg.sdaPin;
    o["scl"] = cfg.sclPin;
    o["dallas"] = cfg.dallasPin;
    o["pump"] = cfg.pumpPin;
    o["heater"] = cfg.heaterPin;
    o["btnMenu"] = cfg.btnMenuPin;
    o["btnPlus"] = cfg.btnPlusPin;
    o["btnMinus"] = cfg.btnMinusPin;
    o["btnPower"] = cfg.btnPowerPin;
    o["lcdAddr"] = cfg.lcdAddr;
    o["heaterWatts"] = cfg.heaterWatts;
    o["pumpWatts"] = cfg.pumpWatts;
    char hw[17], hr[17];
    romToHex(cfg.romWater, hw, sizeof(hw));
    romToHex(cfg.romReturn, hr, sizeof(hr));
    o["romW"] = romIsEmpty(cfg.romWater) ? "" : hw;
    o["romR"] = romIsEmpty(cfg.romReturn) ? "" : hr;
}

void boilerStateJson(JsonVariant &root)
{
    root["enabled"] = (bool)created;
    root["name"] = cfg.name;
    root["on"] = cfg.masterOn;
    root["pump"] = pumpOn;
    root["heater"] = heaterOn;
    if (!isnan(tempWater))
        root["tempWater"] = roundf(tempWater * 10.0f) / 10.0f;
    else
        root["tempWater"] = nullptr;
    if (!isnan(tempReturn))
        root["tempReturn"] = roundf(tempReturn * 10.0f) / 10.0f;
    else
        root["tempReturn"] = nullptr;
    root["roomTemp"] = roundf(cfg.roomTemp * 10.0f) / 10.0f;
    root["spHeater"] = cfg.spHeater;
    root["spPump"] = cfg.spPump;
    root["spRoom"] = cfg.spRoom;
    root["diffHeater"] = cfg.diffHeater;
    root["diffPump"] = cfg.diffPump;
    root["diffRoom"] = cfg.diffRoom;
    root["heaterWatts"] = heaterOn ? cfg.heaterWatts : 0;
    root["pumpWatts"] = pumpOn ? cfg.pumpWatts : 0;
    root["totalKw"] = roundf((totalWattsNow() / 1000.0f) * 100.0f) / 100.0f;
    const float er = heaterKwh();
    const float eb = pumpKwh();
    root["kwhHeater"] = roundf(er * 1000.0f) / 1000.0f;
    root["kwhPump"] = roundf(eb * 10000.0f) / 10000.0f;
    root["kwhTotal"] = roundf((er + eb) * 1000.0f) / 1000.0f;
    root["lcdOk"] = lcdAvailable;
    root["menu"] = menuIndex;
#ifdef ESP32
    root["ownMac"] = WiFi.macAddress();
    root["wifiChannel"] = (int)WiFi.channel();
#else
    root["ownMac"] = "ESP8266";
    root["wifiChannel"] = 0;
#endif
    root["espNowRxCount"] = espNowRxCount;
    root["espNowRxAgoSec"] = (lastEspNowRxMs > 0) ? (long)((millis() - lastEspNowRxMs) / 1000UL) : -1;
    if (lastEspNowRxMs > 0)
    {
        char senderStr[20];
        espNowFormatMac(lastEspNowSenderMac, senderStr, sizeof(senderStr));
        root["espNowSenderMac"] = senderStr;
    }
    else
    {
        root["espNowSenderMac"] = "";
    }
    if (!isnan(lastEspNowRoomHum))
        root["espNowRoomHum"] = lastEspNowRoomHum;
    else
        root["espNowRoomHum"] = nullptr;

    char hw[17], hr[17];
    romToHex(cfg.romWater, hw, sizeof(hw));
    romToHex(cfg.romReturn, hr, sizeof(hr));
    root["romW"] = romIsEmpty(cfg.romWater) ? "" : hw;
    root["romR"] = romIsEmpty(cfg.romReturn) ? "" : hr;

    JsonArray arr = root["probes"].to<JsonArray>();
    for (int i = 0; i < probeCount; i++)
    {
        JsonObject p = arr.add<JsonObject>();
        char hx[17];
        romToHex(probes[i].addr, hx, sizeof(hx));
        p["rom"] = hx;
        if (!isnan(probes[i].temp))
            p["temp"] = probes[i].temp;
        else
            p["temp"] = nullptr;
    }
}

bool boilerQueueCommand(JsonVariantConst json)
{
    if (!created || pendingFlag || measureJson(json) >= sizeof(pendingBuf))
        return false;
    serializeJson(json, pendingBuf, sizeof(pendingBuf));
    __sync_synchronize();
    pendingFlag = true;
    return true;
}

void boilerRequestScan()
{
    scanRequested = true;
}
