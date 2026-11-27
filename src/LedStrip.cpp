#ifdef LED_STRIP

#include "LedStrip.h"
#include "LedEffects.h"
#include "ConfigOnofre.h"
#include "Mqtt.h"
#include "Persistence.h"
#include "LittleFS.h"
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <esp-knx-ip.h>
#include <esp_system.h>
#include "CoreWiFi.h"
#include "DeviceLog.h"
#include "WebServer.h"
#include "CloudIO.h"

#ifndef LED_STRIP_PIN
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
#define LED_STRIP_PIN 4
#else
#define LED_STRIP_PIN 18
#endif
#endif
#ifndef LED_STRIP_COUNT
#define LED_STRIP_COUNT 60
#endif
#ifndef LED_STRIP_MAX
#define LED_STRIP_MAX 600
#endif

extern ConfigOnofre config;

namespace
{
    const char *kFile = "/led.json";
    const char *kTmp = "/led.tmp";
    const char *kPresetFile = "/led_presets.json";
    const char *kPresetTmp = "/led_presets.tmp";
    constexpr unsigned long kSaveDelayMs = 3000;
    constexpr unsigned long kInitDelayMs = 8000;
    constexpr unsigned long kCreateDelayMs = 500;

    volatile bool created = false;
    char fname[24] = "Fita LED";
    int fpin = LED_STRIP_PIN;

    uint8_t knxAddr[3] = {0, 0, 0};
    volatile int knxPending = -1;
    int lastKnxSent = 0;

    bool knxOn() { return knxAddr[0] > 0; }
    void knxAssign()
    {
        if (!knxOn()) return;
        knx.callback_assign(config.knxIdRegister, knx.GA_to_address(knxAddr[0], knxAddr[1], knxAddr[2]));
        knx.callback_assign(config.knxIdRegister, knx.GA_to_address(knxAddr[0], knxAddr[1], 0));
        knx.callback_assign(config.knxIdRegister, knx.GA_to_address(knxAddr[0], 0, 0));
    }

    struct State
    {
        bool on = false;
        uint8_t brightness = 255;
        uint16_t count = LED_STRIP_COUNT;
        uint16_t transition = 400;
        uint16_t limitMA = 0;
        uint16_t mAperLED = 60;
        uint8_t colorOrder = 0;
        uint8_t gamma = 10;
        uint8_t cal[3] = {100, 100, 100};
        uint8_t bootPreset = 0;
    } st;

    const neoPixelType kOrders[6] = {NEO_GRB, NEO_RGB, NEO_BRG, NEO_RBG, NEO_GBR, NEO_BGR};

    uint8_t gammaLut[256];
    uint8_t gammaBuilt = 0;
    void buildGamma()
    {
        if (gammaBuilt == st.gamma) return;
        for (int i = 0; i < 256; i++)
            gammaLut[i] = st.gamma <= 10 ? (uint8_t)i : (uint8_t)(powf(i / 255.f, st.gamma / 10.f) * 255.f + 0.5f);
        gammaBuilt = st.gamma;
    }

    float mLvl = 0.f;
    float mBri = 255.f;
    float segLvl[8] = {1, 1, 1, 1, 1, 1, 1, 1};
    unsigned long lastRenderMs = 0;
    uint32_t estMA = 0;
    bool limiting = false;

    constexpr int kMaxSeg = 8;
    struct Seg
    {
        uint16_t start = 0, stop = 0;
        bool on = true;
        uint8_t bri = 255;
        led::RGB color{255, 160, 60};
        led::RGB color2{0, 0, 0};
        led::RGB color3{255, 255, 255};
        uint8_t effect = led::FX_STATIC;
        uint8_t speed = 128;
        uint8_t intensity = 128;
        uint8_t palette = 0;
        bool reverse = false;
        bool mirror = false;
        uint8_t grouping = 1;
        uint8_t spacing = 0;
        int16_t offset = 0;
    };
    Seg segs[kMaxSeg];
    int segCount = 1;
    led::Engine engines[kMaxSeg];
    std::vector<led::RGB> frame;
    bool frameDirty = true;

    struct PatternStep
    {
        uint8_t effect;
        uint8_t palette;
        led::RGB color;
        uint8_t speed;
        uint8_t intensity;
        uint8_t brightness;
        uint32_t durationMs;
    };

    struct DynamicPattern
    {
        char name[28];
        bool loop;
        uint8_t count;
        PatternStep steps[8];
    };

    DynamicPattern kPatterns[6] = {
        {"Entardecer Dinâmico", true, 3, {{22, 8, {250, 163, 7}, 75, 160, 230, 8000u}, {23, 5, {230, 36, 0}, 95, 140, 190, 8000u}, {2, 8, {144, 31, 91}, 65, 120, 150, 6000u}}},
        {"Modo Cinema & Relax", true, 2, {{15, 3, {0, 180, 216}, 55, 110, 130, 10000u}, {22, 9, {56, 239, 125}, 60, 145, 140, 10000u}}},
        {"Festa WLED", true, 3, {{16, 2, {255, 85, 0}, 170, 190, 255, 6000u}, {21, 1, {255, 0, 128}, 165, 210, 255, 6000u}, {13, 10, {0, 245, 212}, 180, 185, 255, 6000u}}},
        {"Lareira Aconchegante", true, 2, {{8, 7, {255, 136, 0}, 120, 165, 200, 12000u}, {18, 0, {255, 140, 40}, 110, 140, 175, 8000u}}}
    };
    int kPatternCount = 4;
    int activePatternIdx = -1;
    uint8_t activeStepIdx = 0;
    unsigned long stepStartMs = 0;

    constexpr int kPresetCount = 8;
    struct WledPreset
    {
        uint8_t slot;
        char name[24];
        bool occupied;
        uint8_t brightness;
        uint8_t effect;
        uint8_t palette;
        led::RGB color;
        led::RGB color2;
        led::RGB color3;
        uint8_t speed;
        uint8_t intensity;
        bool reverse;
        bool mirror;
        uint8_t grouping;
        uint8_t spacing;
        int16_t offset;
    };

    WledPreset kPresets[kPresetCount] = {
        {1, "Pacifica Relax", true, 210, 27, 3, {0, 180, 216}, {0, 18, 51}, {202, 240, 248}, 90, 140, false, false, 1, 0, 0},
        {2, "Sunset Lounge", true, 225, 28, 8, {250, 163, 7}, {43, 11, 63}, {255, 223, 122}, 105, 155, false, true, 1, 0, 0},
        {3, "Festa Fogos", true, 255, 26, 2, {255, 85, 0}, {0, 0, 0}, {255, 255, 255}, 150, 185, false, false, 1, 0, 0},
        {4, "Branco Quente", true, 190, 0, 0, {255, 175, 95}, {0, 0, 0}, {255, 255, 255}, 128, 128, false, false, 1, 0, 0},
        {5, "Preset 5", false, 220, 0, 0, {255, 160, 60}, {0, 0, 0}, {255, 255, 255}, 128, 128, false, false, 1, 0, 0},
        {6, "Preset 6", false, 220, 0, 0, {255, 160, 60}, {0, 0, 0}, {255, 255, 255}, 128, 128, false, false, 1, 0, 0},
        {7, "Preset 7", false, 220, 0, 0, {255, 160, 60}, {0, 0, 0}, {255, 255, 255}, 128, 128, false, false, 1, 0, 0},
        {8, "Preset 8", false, 220, 0, 0, {255, 160, 60}, {0, 0, 0}, {255, 255, 255}, 128, 128, false, false, 1, 0, 0}
    };
    uint8_t activePresetSlot = 0;

    bool nlActive = false;
    uint16_t nlDurationMin = 15;
    uint8_t nlStartBri = 255;
    unsigned long nlStartMs = 0;

    void rebuildEngines()
    {
        for (int i = 0; i < segCount; i++)
        {
            const Seg &g = segs[i];
            engines[i].resize(g.stop - g.start + 1);
            engines[i].setTransition(st.transition);
            engines[i].set((led::Effect)g.effect, g.color, g.palette, g.speed, g.intensity,
                           g.reverse, g.mirror, g.color2, g.color3, g.grouping, g.spacing, g.offset);
        }
        frame.assign(st.count, led::RGB());
        frameDirty = true;
    }

    void syncEngines()
    {
        for (int i = 0; i < segCount; i++)
        {
            engines[i].setTransition(st.transition);
            engines[i].set((led::Effect)segs[i].effect, segs[i].color, segs[i].palette,
                           segs[i].speed, segs[i].intensity, segs[i].reverse, segs[i].mirror,
                           segs[i].color2, segs[i].color3, segs[i].grouping, segs[i].spacing, segs[i].offset);
        }
        frameDirty = true;
    }

    void fitSegments()
    {
        const int n = st.count;
        int keep = 0;
        for (int i = 0; i < segCount; i++)
        {
            if (segs[i].start >= n) continue;
            if (segs[i].stop >= n) segs[i].stop = n - 1;
            if (keep != i) segs[keep] = segs[i];
            keep++;
        }
        if (keep == 0)
        {
            segs[0] = Seg();
            segs[0].stop = n - 1;
            keep = 1;
        }
        segCount = keep;
    }

    Adafruit_NeoPixel *strip = nullptr;
    unsigned long initAfter = 0;
    bool needsSave = false, wasOn = false, needsPublish = false;
    bool saveNow = false;
    bool needsDiscovery = false, needsUndiscovery = false;
    bool testRequested = false;
    bool cmdAppliedBeforeInit = false;
    unsigned long lastChange = 0;
    char tCmd[64], tState[64], tSpeed[64], tSpeedSet[64], tInt[64], tIntSet[64];
    char tPalette[64], tPaletteSet[64], tPattern[64], tPatternSet[64];
    char tPreset[64], tPresetSet[64], tNl[64], tNlSet[64];
    char tCloudCmd[160] = {0}, tCloudState[160] = {0};

    void topics()
    {
        sprintf(tCmd,        "onofre/%s/led/set",            config.chipId);
        sprintf(tState,      "onofre/%s/led/state",          config.chipId);
        sprintf(tSpeed,      "onofre/%s/led/speed",          config.chipId);
        sprintf(tSpeedSet,   "onofre/%s/led/speed/set",      config.chipId);
        sprintf(tInt,        "onofre/%s/led/intensity",      config.chipId);
        sprintf(tIntSet,     "onofre/%s/led/intensity/set",  config.chipId);
        sprintf(tPalette,    "onofre/%s/led/palette",        config.chipId);
        sprintf(tPaletteSet, "onofre/%s/led/palette/set",    config.chipId);
        sprintf(tPattern,    "onofre/%s/led/pattern",        config.chipId);
        sprintf(tPatternSet, "onofre/%s/led/pattern/set",    config.chipId);
        sprintf(tPreset,     "onofre/%s/led/preset",         config.chipId);
        sprintf(tPresetSet,  "onofre/%s/led/preset/set",     config.chipId);
        sprintf(tNl,         "onofre/%s/led/nightlight",     config.chipId);
        sprintf(tNlSet,      "onofre/%s/led/nightlight/set", config.chipId);
    }

    void touch()
    {
        needsSave = true;
        lastChange = millis();
        syncEngines();
    }

    void loadPresets()
    {
        File f = LittleFS.open(kPresetFile, "r");
        if (!f) return;
        JsonDocument doc;
        if (deserializeJson(doc, f) == DeserializationError::Ok && doc.is<JsonArray>())
        {
            int idx = 0;
            for (JsonVariant v : doc.as<JsonArray>())
            {
                if (idx >= kPresetCount) break;
                JsonObject o = v.as<JsonObject>();
                WledPreset &p = kPresets[idx++];
                p.occupied = o["occ"] | p.occupied;
                if (o["nm"].is<const char *>()) strlcpy(p.name, o["nm"], sizeof(p.name));
                p.brightness = o["bri"] | p.brightness;
                uint8_t fx = o["fx"] | p.effect;
                p.effect = fx < led::FX_COUNT ? fx : 0;
                uint8_t pal = o["pal"] | p.palette;
                p.palette = pal < led::PAL_COUNT ? pal : 0;
                p.color = led::RGB(o["r"] | 255, o["g"] | 160, o["b"] | 60);
                p.color2 = led::RGB(o["r2"] | 0, o["g2"] | 0, o["b2"] | 0);
                p.color3 = led::RGB(o["r3"] | 255, o["g3"] | 255, o["b3"] | 255);
                p.speed = o["sp"] | 128;
                p.intensity = o["it"] | 128;
                p.reverse = o["rev"] | false;
                p.mirror = o["mir"] | false;
                p.grouping = constrain((int)(o["grp"] | 1), 1, 16);
                p.spacing = constrain((int)(o["spc"] | 0), 0, 15);
                p.offset = (int16_t)(o["off"] | 0);
            }
        }
        f.close();
    }

    void savePresets()
    {
        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();
        for (int i = 0; i < kPresetCount; i++)
        {
            const WledPreset &p = kPresets[i];
            JsonObject o = arr.add<JsonObject>();
            o["occ"] = p.occupied;
            o["nm"] = p.name;
            o["bri"] = p.brightness;
            o["fx"] = p.effect;
            o["pal"] = p.palette;
            o["r"] = p.color.r; o["g"] = p.color.g; o["b"] = p.color.b;
            o["r2"] = p.color2.r; o["g2"] = p.color2.g; o["b2"] = p.color2.b;
            o["r3"] = p.color3.r; o["g3"] = p.color3.g; o["b3"] = p.color3.b;
            o["sp"] = p.speed;
            o["it"] = p.intensity;
            o["rev"] = p.reverse;
            o["mir"] = p.mirror;
            o["grp"] = p.grouping;
            o["spc"] = p.spacing;
            o["off"] = p.offset;
        }
        persistJsonAtomically(kPresetFile, kPresetTmp, doc);
    }

    void applyPresetSlot(int slot)
    {
        if (slot < 1 || slot > kPresetCount)
        {
            activePresetSlot = 0;
            return;
        }
        const WledPreset &p = kPresets[slot - 1];
        if (!p.occupied) return;
        st.on = true;
        st.brightness = p.brightness;
        activePatternIdx = -1;
        activePresetSlot = (uint8_t)slot;
        for (int i = 0; i < segCount; i++)
        {
            segs[i].effect = p.effect;
            segs[i].palette = p.palette;
            segs[i].color = p.color;
            segs[i].color2 = p.color2;
            segs[i].color3 = p.color3;
            segs[i].speed = p.speed;
            segs[i].intensity = p.intensity;
            segs[i].reverse = p.reverse;
            segs[i].mirror = p.mirror;
            segs[i].grouping = p.grouping;
            segs[i].spacing = p.spacing;
            segs[i].offset = p.offset;
        }
        touch();
    }

    void saveCurrentToPreset(int slot, const char *customName)
    {
        if (slot < 1 || slot > kPresetCount) return;
        WledPreset &p = kPresets[slot - 1];
        p.occupied = true;
        if (customName && *customName)
            strlcpy(p.name, customName, sizeof(p.name));
        else if (strncmp(p.name, "Preset ", 7) == 0 || !*p.name)
            snprintf(p.name, sizeof(p.name), "%s", led::kEffectNames[segs[0].effect]);
        p.brightness = st.brightness;
        p.effect = segs[0].effect;
        p.palette = segs[0].palette;
        p.color = segs[0].color;
        p.color2 = segs[0].color2;
        p.color3 = segs[0].color3;
        p.speed = segs[0].speed;
        p.intensity = segs[0].intensity;
        p.reverse = segs[0].reverse;
        p.mirror = segs[0].mirror;
        p.grouping = segs[0].grouping;
        p.spacing = segs[0].spacing;
        p.offset = segs[0].offset;
        activePresetSlot = (uint8_t)slot;
        savePresets();
        needsDiscovery = true;
        needsPublish = true;
    }

    int findPreset(const char *s)
    {
        if (!s || strcasecmp(s, "Nenhum") == 0 || strcasecmp(s, "0") == 0) return 0;
        const int num = atoi(s);
        if (num >= 1 && num <= kPresetCount) return num;
        for (int i = 0; i < kPresetCount; i++)
        {
            char lbl[32];
            snprintf(lbl, sizeof(lbl), "%d: %s", i + 1, kPresets[i].name);
            if (strcasecmp(s, kPresets[i].name) == 0 || strcasecmp(s, lbl) == 0)
                return i + 1;
        }
        return -1;
    }

    void startNightlight(uint16_t minutes)
    {
        if (minutes == 0)
        {
            nlActive = false;
            return;
        }
        nlDurationMin = constrain((int)minutes, 1, 120);
        if (!st.on) st.on = true;
        if (st.brightness < 10) st.brightness = 180;
        nlStartBri = st.brightness;
        nlStartMs = millis();
        nlActive = true;
        touch();
    }

    void tickNightlight(unsigned long now)
    {
        if (!nlActive) return;
        const unsigned long totalMs = (unsigned long)nlDurationMin * 60000UL;
        const unsigned long elapsed = now - nlStartMs;
        if (elapsed >= totalMs)
        {
            nlActive = false;
            st.on = false;
            st.brightness = nlStartBri;
            touch();
            needsPublish = true;
            return;
        }
        const float fracLeft = 1.f - (float)elapsed / (float)totalMs;
        const uint8_t targetBri = (uint8_t)max(1, (int)(nlStartBri * fracLeft + 0.5f));
        if (targetBri != st.brightness)
        {
            st.brightness = targetBri;
            frameDirty = true;
        }
    }

    void applyPatternStep(const PatternStep &step)
    {
        st.on = true;
        st.brightness = step.brightness;
        activePresetSlot = 0;
        for (int i = 0; i < segCount; i++)
        {
            segs[i].effect = step.effect < led::FX_COUNT ? step.effect : 0;
            segs[i].palette = step.palette < led::PAL_COUNT ? step.palette : 0;
            segs[i].color = step.color;
            segs[i].speed = step.speed;
            segs[i].intensity = step.intensity;
        }
        syncEngines();
    }

    void activatePatternByName(const char *name)
    {
        if (!name || strcasecmp(name, "Nenhum") == 0 || strcasecmp(name, "None") == 0)
        {
            activePatternIdx = -1;
            return;
        }
        for (int i = 0; i < kPatternCount; i++)
        {
            if (strcasecmp(name, kPatterns[i].name) == 0)
            {
                activePatternIdx = i;
                activeStepIdx = 0;
                stepStartMs = millis();
                applyPatternStep(kPatterns[i].steps[0]);
                return;
            }
        }
    }

    void tickDynamicPattern(unsigned long now)
    {
        if (activePatternIdx < 0 || activePatternIdx >= kPatternCount) return;
        const DynamicPattern &pat = kPatterns[activePatternIdx];
        if (pat.count == 0) return;
        const PatternStep &cur = pat.steps[activeStepIdx];
        if (now - stepStartMs >= cur.durationMs)
        {
            if (activeStepIdx + 1 < pat.count)
                activeStepIdx++;
            else if (pat.loop)
                activeStepIdx = 0;
            else
            {
                activePatternIdx = -1;
                needsPublish = true;
                return;
            }
            stepStartMs = now;
            applyPatternStep(pat.steps[activeStepIdx]);
            needsPublish = true;
        }
    }

    void load()
    {
        File f = LittleFS.open(kFile, "r");
        if (!f) return;
        JsonDocument doc;
        if (deserializeJson(doc, f) == DeserializationError::Ok)
        {
            st.on = doc["on"] | false;
            st.brightness = doc["bri"] | 255;
            uint16_t n = doc["n"] | LED_STRIP_COUNT;
            st.count = (n >= 1 && n <= LED_STRIP_MAX) ? n : LED_STRIP_COUNT;
            st.transition = constrain((int)(doc["tr"] | 400), 0, 3000);
            st.limitMA = constrain((int)(doc["lm"] | 0), 0, 20000);
            st.mAperLED = constrain((int)(doc["lp"] | 60), 10, 100);
            st.colorOrder = constrain((int)(doc["co"] | 0), 0, 5);
            st.gamma = constrain((int)(doc["gm"] | 10), 10, 40);
            st.cal[0] = constrain((int)(doc["cr"] | 100), 10, 100);
            st.cal[1] = constrain((int)(doc["cg"] | 100), 10, 100);
            st.cal[2] = constrain((int)(doc["cb"] | 100), 10, 100);
            st.bootPreset = constrain((int)(doc["bp"] | 0), 0, kPresetCount);

            if (doc["cp"].is<JsonArray>())
            {
                JsonArray cps = doc["cp"].as<JsonArray>();
                for (int slot = 0; slot < led::kCustomCount && slot < (int)cps.size(); slot++)
                {
                    JsonArray stops = cps[slot].as<JsonArray>();
                    uint8_t cnt = 0;
                    for (JsonVariant sv : stops)
                    {
                        if (cnt >= led::kMaxCustomStops) break;
                        JsonArray sa = sv.as<JsonArray>();
                        if (sa.size() >= 4)
                        {
                            led::customPalettes()[slot].stops[cnt++] = {
                                (uint8_t)(sa[0] | 0), (uint8_t)(sa[1] | 0),
                                (uint8_t)(sa[2] | 0), (uint8_t)(sa[3] | 0)
                            };
                        }
                    }
                    if (cnt >= 2) led::customPalettes()[slot].count = cnt;
                }
            }

            if (doc["sg"].is<JsonArray>() && doc["sg"].size() > 0)
            {
                segCount = 0;
                for (JsonVariant v : doc["sg"].as<JsonArray>())
                {
                    if (segCount >= kMaxSeg) break;
                    JsonArray a = v.as<JsonArray>();
                    Seg &g = segs[segCount++];
                    g.start = a[0] | 0;
                    g.stop = a[1] | 0;
                    g.on = a[2] | true;
                    g.bri = a[3] | 255;
                    g.color = led::RGB(a[4] | 255, a[5] | 160, a[6] | 60);
                    uint8_t fx = a[7] | 0;
                    g.effect = fx < led::FX_COUNT ? fx : 0;
                    g.speed = a[8] | 128;
                    g.intensity = a[9] | 128;
                    uint8_t pal = a.size() > 10 ? (uint8_t)(a[10] | 0) : 0;
                    g.palette = pal < led::PAL_COUNT ? pal : 0;
                    g.reverse = a.size() > 11 ? (bool)(a[11] | 0) : false;
                    g.mirror = a.size() > 12 ? (bool)(a[12] | 0) : false;
                    g.color2 = a.size() > 15 ? led::RGB(a[13] | 0, a[14] | 0, a[15] | 0) : led::RGB(0, 0, 0);
                    g.color3 = a.size() > 18 ? led::RGB(a[16] | 255, a[17] | 255, a[18] | 255) : led::RGB(255, 255, 255);
                    g.grouping = a.size() > 19 ? constrain((int)(a[19] | 1), 1, 16) : 1;
                    g.spacing = a.size() > 20 ? constrain((int)(a[20] | 0), 0, 15) : 0;
                    g.offset = a.size() > 21 ? (int16_t)(a[21] | 0) : 0;
                }
            }
            else
            {
                segCount = 1;
                segs[0] = Seg();
                segs[0].stop = st.count - 1;
                segs[0].color = led::RGB(doc["r"] | 255, doc["g"] | 160, doc["b"] | 60);
                uint8_t fx = doc["fx"] | 0;
                segs[0].effect = fx < led::FX_COUNT ? fx : 0;
                segs[0].speed = doc["sx"] | 128;
                segs[0].intensity = doc["ix"] | 128;
            }
            fitSegments();
            fpin = doc["pin"] | LED_STRIP_PIN;
            strlcpy(fname, doc["name"] | "Fita LED", sizeof(fname));
            created = doc["created"] | false;
            knxAddr[0] = doc["ka"] | 0;
            knxAddr[1] = doc["kl"] | 0;
            knxAddr[2] = doc["km"] | 0;
        }
        f.close();
    }

    void save()
    {
        JsonDocument doc;
        doc["created"] = (bool)created;
        doc["name"] = fname;
        doc["pin"] = fpin;
        doc["on"] = st.on;
        doc["bri"] = st.brightness;
        doc["n"] = st.count;
        doc["tr"] = st.transition;
        doc["lm"] = st.limitMA;
        doc["lp"] = st.mAperLED;
        doc["co"] = st.colorOrder;
        doc["gm"] = st.gamma;
        doc["cr"] = st.cal[0];
        doc["cg"] = st.cal[1];
        doc["cb"] = st.cal[2];
        doc["bp"] = st.bootPreset;

        JsonArray cps = doc["cp"].to<JsonArray>();
        for (int slot = 0; slot < led::kCustomCount; slot++)
        {
            JsonArray stops = cps.add<JsonArray>();
            for (uint8_t k = 0; k < led::customPalettes()[slot].count; k++)
            {
                JsonArray sa = stops.add<JsonArray>();
                sa.add(led::customPalettes()[slot].stops[k].pos);
                sa.add(led::customPalettes()[slot].stops[k].r);
                sa.add(led::customPalettes()[slot].stops[k].g);
                sa.add(led::customPalettes()[slot].stops[k].b);
            }
        }

        JsonArray sg = doc["sg"].to<JsonArray>();
        for (int i = 0; i < segCount; i++)
        {
            const Seg &g = segs[i];
            JsonArray a = sg.add<JsonArray>();
            a.add(g.start);
            a.add(g.stop);
            a.add(g.on);
            a.add(g.bri);
            a.add(g.color.r); a.add(g.color.g); a.add(g.color.b);
            a.add(g.effect);
            a.add(g.speed);
            a.add(g.intensity);
            a.add(g.palette);
            a.add(g.reverse ? 1 : 0);
            a.add(g.mirror ? 1 : 0);
            a.add(g.color2.r); a.add(g.color2.g); a.add(g.color2.b);
            a.add(g.color3.r); a.add(g.color3.g); a.add(g.color3.b);
            a.add(g.grouping);
            a.add(g.spacing);
            a.add(g.offset);
        }
        doc["ka"] = knxAddr[0];
        doc["kl"] = knxAddr[1];
        doc["km"] = knxAddr[2];
        persistJsonAtomically(kFile, kTmp, doc);
    }

    void presetLabel(int slot, char *buf, size_t len)
    {
        if (slot < 1 || slot > kPresetCount) { strlcpy(buf, "Nenhum", len); return; }
        snprintf(buf, len, "%d: %s", slot, kPresets[slot - 1].name);
    }

    void publishState()
    {
        JsonDocument doc;
        doc["state"] = st.on ? "ON" : "OFF";
        doc["brightness"] = st.brightness;
        doc["color_mode"] = "rgb";
        JsonObject c = doc["color"].to<JsonObject>();
        c["r"] = segs[0].color.r;
        c["g"] = segs[0].color.g;
        c["b"] = segs[0].color.b;
        doc["effect"] = led::kEffectNames[segs[0].effect];
        doc["palette"] = led::kPaletteNames[segs[0].palette];
        doc["pattern"] = activePatternIdx >= 0 ? kPatterns[activePatternIdx].name : "Nenhum";
        char pLbl[32];
        presetLabel(activePresetSlot, pLbl, sizeof(pLbl));
        doc["preset"] = pLbl;
        doc["speed"] = segs[0].speed;
        doc["intensity"] = segs[0].intensity;
        doc["leds"] = st.count;

        JsonArray sg = doc["segs"].to<JsonArray>();
        for (int i = 0; i < segCount; i++)
        {
            const Seg &g = segs[i];
            JsonObject o = sg.add<JsonObject>();
            o["start"] = g.start;
            o["stop"] = g.stop;
            o["on"] = g.on;
            o["brightness"] = g.bri;
            JsonArray gc = o["color"].to<JsonArray>();
            gc.add(g.color.r); gc.add(g.color.g); gc.add(g.color.b);
            o["effect"] = g.effect;
            o["p"] = g.palette;
        }
        if (measureJson(doc) + strlen(tState) + 16 > MQTT_MAX_PACKET_SIZE)
            doc.remove("segs");
        if (measureJson(doc) + strlen(tState) + 16 > MQTT_MAX_PACKET_SIZE)
        {
            doc.remove("pattern");
            doc.remove("preset");
        }
        String out;
        serializeJson(doc, out);
        publishOnMqtt(tState, out.c_str(), true);
        if (tCloudState[0] != '\0')
            notifyStateToCloudIO(tCloudState, out.c_str());
        yield();
        publishOnMqtt(tSpeed, String(segs[0].speed).c_str(), true);
        publishOnMqtt(tInt, String(segs[0].intensity).c_str(), true);
        publishOnMqtt(tPalette, led::kPaletteNames[segs[0].palette], true);
        publishOnMqtt(tPattern, activePatternIdx >= 0 ? kPatterns[activePatternIdx].name : "Nenhum", true);
        publishOnMqtt(tPreset, pLbl, true);
        publishOnMqtt(tNl, nlActive ? String(nlDurationMin).c_str() : "0", true);
    }

    int findEffect(const char *name)
    {
        if (!name) return -1;
        while (*name == ' ') name++;
        for (int i = 0; i < led::FX_COUNT; i++)
            if (strcasecmp(name, led::kEffectNames[i]) == 0) return i;
        return -1;
    }

    int findPalette(const char *name)
    {
        if (!name) return -1;
        while (*name == ' ') name++;
        if (strcasecmp(name, "Default (Cor Única)") == 0 || strcasecmp(name, "Default (Cor Unica)") == 0)
            return 0;
        for (int i = 0; i < led::PAL_COUNT; i++)
            if (strcasecmp(name, led::kPaletteNames[i]) == 0) return i;
        return -1;
    }

    void applyCount(uint16_t n)
    {
        if (n < 1 || n > LED_STRIP_MAX || n == st.count) return;
        st.count = n;
        if (strip)
        {
            strip->clear();
            strip->show();
            strip->updateLength(n);
        }
        if (segCount == 1)
        {
            segs[0].start = 0;
            segs[0].stop = n - 1;
        }
        else
            fitSegments();
        rebuildEngines();
    }

    void applySegments(JsonArray arr)
    {
        Seg tmp[kMaxSeg];
        int n = 0;
        for (JsonVariant v : arr)
        {
            if (n >= kMaxSeg) break;
            int a = constrain((int)(v["start"] | 0), 0, st.count - 1);
            int b = constrain((int)(v["stop"] | 0), 0, st.count - 1);
            if (b < a) { const int x = a; a = b; b = x; }
            Seg g = (n < segCount) ? segs[n] : segs[0];
            g.start = a;
            g.stop = b;
            tmp[n++] = g;
        }
        if (n == 0) return;
        for (int i = 0; i < n; i++) segs[i] = tmp[i];
        segCount = n;
        for (int i = 0; i < kMaxSeg; i++)
            segLvl[i] = (i < segCount && segs[i].on) ? 1.f : 0.f;
        rebuildEngines();
    }

    bool effectIgnoresPrimaryOnDefault(uint8_t fx)
    {
        switch (fx)
        {
        case led::FX_RAINBOW:
        case led::FX_FIRE:
        case led::FX_COLOR_CYCLE:
        case led::FX_THEATER_RAINBOW:
        case led::FX_PLASMA:
        case led::FX_CONFETTI:
        case led::FX_POLICE:
        case led::FX_JUGGLE:
        case led::FX_BOUNCING_BALLS:
        case led::FX_FIREWORKS:
        case led::FX_PACIFICA:
        case led::FX_COLORWAVES:
            return true;
        default:
            return false;
        }
    }

    void applyCommand(const char *payload)
    {
        if (!strip) cmdAppliedBeforeInit = true;
        if (strcasecmp(payload, "ON") == 0 || strcasecmp(payload, "OFF") == 0)
        {
            st.on = strcasecmp(payload, "ON") == 0;
            if (st.on)
            {
                if (st.brightness == 0) st.brightness = 255;
                bool anyOn = false;
                for (int i = 0; i < segCount; i++) if (segs[i].on) anyOn = true;
                if (!anyOn) for (int i = 0; i < segCount; i++) segs[i].on = true;
            }
            else
            {
                activePatternIdx = -1;
                nlActive = false;
            }
            touch();
            return;
        }
        JsonDocument doc;
        if (deserializeJson(doc, payload) != DeserializationError::Ok) return;

        if (doc["leds"].is<int>() || doc["segments"].is<JsonArray>() ||
            doc["colorOrder"].is<int>() || doc["gamma"].is<int>() ||
            (doc["transition"].is<int>() && !doc["state"].is<const char *>()) ||
            doc["limit"].is<int>() || doc["maPerLed"].is<int>() || doc["knx"].is<JsonObject>() ||
            doc["customPalette"].is<JsonObject>() || doc["bootPreset"].is<int>())
            saveNow = true;

        if (doc["bootPreset"].is<int>())
            st.bootPreset = constrain((int)doc["bootPreset"], 0, kPresetCount);

        if (doc["savePreset"].is<int>())
            saveCurrentToPreset((int)doc["savePreset"], doc["presetName"] | "");

        if (doc["preset"].is<int>())
        {
            applyPresetSlot((int)doc["preset"]);
            return;
        }
        if (doc["preset"].is<const char *>())
        {
            const int ps = findPreset(doc["preset"]);
            if (ps >= 0) applyPresetSlot(ps);
            return;
        }

        if (doc["nightlight"].is<JsonObject>())
        {
            const bool nlOn = doc["nightlight"]["on"] | true;
            const int mins = doc["nightlight"]["min"] | (int)nlDurationMin;
            startNightlight(nlOn ? (uint16_t)max(1, mins) : 0);
        }
        else if (doc["nightlight"].is<int>())
        {
            startNightlight((uint16_t)max(0, (int)doc["nightlight"]));
        }

        if (doc["customPalette"].is<JsonObject>())
        {
            int slot = constrain((int)(doc["customPalette"]["slot"] | 0), 0, (int)led::kCustomCount - 1);
            if (doc["customPalette"]["stops"].is<JsonArray>())
            {
                uint8_t cnt = 0;
                for (JsonObject s : doc["customPalette"]["stops"].as<JsonArray>())
                {
                    if (cnt >= led::kMaxCustomStops) break;
                    led::customPalettes()[slot].stops[cnt++] = {
                        (uint8_t)constrain((int)(s["pos"] | 0), 0, 255),
                        (uint8_t)constrain((int)(s["r"] | 0), 0, 255),
                        (uint8_t)constrain((int)(s["g"] | 0), 0, 255),
                        (uint8_t)constrain((int)(s["b"] | 0), 0, 255)
                    };
                }
                if (cnt >= 2) led::customPalettes()[slot].count = cnt;
            }
        }

        if (doc["pattern"].is<const char *>())
        {
            activatePatternByName(doc["pattern"]);
            touch();
            return;
        }

        int t = doc["seg"] | -1;
        if (t < 0 || t >= segCount) t = -1;
        const int first = (t < 0) ? 0 : t, last = (t < 0) ? segCount - 1 : t;

        if (doc["state"].is<const char *>())
        {
            const bool v = strcasecmp(doc["state"] | "ON", "ON") == 0;
            if (t < 0)
            {
                st.on = v;
                if (v)
                {
                    if (st.brightness == 0) st.brightness = 255;
                    bool anyOn = false;
                    for (int i = 0; i < segCount; i++) if (segs[i].on) anyOn = true;
                    if (!anyOn) for (int i = 0; i < segCount; i++) segs[i].on = true;
                }
                else
                {
                    activePatternIdx = -1;
                    nlActive = false;
                }
            }
            else { segs[t].on = v; if (v) st.on = true; }
        }
        if (doc["brightness"].is<int>())
        {
            const uint8_t v = constrain((int)doc["brightness"], 0, 255);
            if (t < 0)
            {
                st.brightness = v;
                if (segCount == 1 && v > 0) segs[0].bri = 255;
            }
            else segs[t].bri = v;
        }
        const bool bareColorCmd = doc["color"].is<JsonObject>() &&
                                  !doc["palette"].is<const char *>() && !doc["palette"].is<int>() &&
                                  !doc["effect"].is<const char *>() &&
                                  !doc["color2"].is<JsonObject>() && !doc["color3"].is<JsonObject>();
        for (int i = first; i <= last; i++)
        {
            if (doc["color"].is<JsonObject>())
            {
                segs[i].color = led::RGB(doc["color"]["r"] | 0, doc["color"]["g"] | 0, doc["color"]["b"] | 0);
                activePatternIdx = -1;
                activePresetSlot = 0;
                if (bareColorCmd)
                {
                    if (segs[i].palette > 0 && segs[i].palette < led::kDynamicStartId)
                        segs[i].palette = 0;
                    if (segs[i].palette == 0 && effectIgnoresPrimaryOnDefault(segs[i].effect))
                        segs[i].effect = led::FX_STATIC;
                }
            }
            if (doc["color2"].is<JsonObject>())
                segs[i].color2 = led::RGB(doc["color2"]["r"] | 0, doc["color2"]["g"] | 0, doc["color2"]["b"] | 0);
            if (doc["color3"].is<JsonObject>())
                segs[i].color3 = led::RGB(doc["color3"]["r"] | 255, doc["color3"]["g"] | 255, doc["color3"]["b"] | 255);
            if (doc["effect"].is<const char *>())
            {
                const int fx = findEffect(doc["effect"]);
                if (fx >= 0) { segs[i].effect = fx; activePatternIdx = -1; activePresetSlot = 0; }
            }
            if (doc["palette"].is<const char *>())
            {
                const int pal = findPalette(doc["palette"]);
                if (pal >= 0) { segs[i].palette = pal; activePatternIdx = -1; activePresetSlot = 0; }
            }
            else if (doc["palette"].is<int>())
            {
                segs[i].palette = constrain((int)doc["palette"], 0, led::PAL_COUNT - 1);
                activePatternIdx = -1;
                activePresetSlot = 0;
            }
            if (doc["reverse"].is<bool>()) segs[i].reverse = doc["reverse"];
            if (doc["mirror"].is<bool>()) segs[i].mirror = doc["mirror"];
            if (doc["grouping"].is<int>()) segs[i].grouping = constrain((int)doc["grouping"], 1, 16);
            if (doc["spacing"].is<int>()) segs[i].spacing = constrain((int)doc["spacing"], 0, 15);
            if (doc["offset"].is<int>()) segs[i].offset = (int16_t)constrain((int)doc["offset"], -300, 300);
            if (doc["speed"].is<int>()) segs[i].speed = constrain((int)doc["speed"], 0, 255);
            if (doc["intensity"].is<int>()) segs[i].intensity = constrain((int)doc["intensity"], 0, 255);
        }
        if (doc["leds"].is<int>()) applyCount((uint16_t)constrain((int)doc["leds"], 1, LED_STRIP_MAX));
        if (doc["segments"].is<JsonArray>()) applySegments(doc["segments"].as<JsonArray>());
        if (doc["transition"].is<int>() && !doc["state"].is<const char *>())
            st.transition = constrain((int)doc["transition"], 0, 3000);
        if (doc["limit"].is<int>()) st.limitMA = constrain((int)doc["limit"], 0, 20000);
        if (doc["maPerLed"].is<int>()) st.mAperLED = constrain((int)doc["maPerLed"], 10, 100);
        if (doc["colorOrder"].is<int>())
        {
            st.colorOrder = constrain((int)doc["colorOrder"], 0, 5);
            if (strip) strip->updateType(kOrders[st.colorOrder] + NEO_KHZ800);
            frameDirty = true;
        }
        if (doc["gamma"].is<int>()) st.gamma = constrain((int)doc["gamma"], 10, 40);
        if (doc["cal"].is<JsonArray>() && doc["cal"].size() == 3)
            for (int i = 0; i < 3; i++) st.cal[i] = constrain((int)doc["cal"][i], 10, 100);
        if (doc["test"] | false) testRequested = true;
        if (doc["knx"].is<JsonObject>())
        {
            knxAddr[0] = constrain((int)(doc["knx"]["area"] | 0), 0, 31);
            knxAddr[1] = constrain((int)(doc["knx"]["line"] | 0), 0, 7);
            knxAddr[2] = constrain((int)(doc["knx"]["member"] | 0), 0, 255);
            knxAssign();
            lastKnxSent = st.on ? 100 : 0;
        }
        touch();
    }

    char pendingBuf[1024];
    volatile bool pendingFlag = false;

    void haDevice(JsonObject &o)
    {
        JsonObject d = o["dev"].to<JsonObject>();
        d["ids"] = "OnOfre-" + String(config.chipId);
        d["name"] = config.nodeId;
        d["mf"] = "OnOfre Portugal";
        d["sw"] = String(VERSION);
    }

    void publishConfig(const char *domain, const String &uid, JsonDocument &doc)
    {
        String out;
        serializeJson(doc, out);
        publishOnMqtt((String("homeassistant/") + domain + "/" + uid + "/config").c_str(), out.c_str(), true);
        yield();
        delay(5);
    }

    void selfTest();

    void initStrip()
    {
        strip = new Adafruit_NeoPixel(st.count, fpin, kOrders[st.colorOrder] + NEO_KHZ800);
        strip->begin();
        strip->clear();
        strip->show();
        if (!cmdAppliedBeforeInit && st.bootPreset > 0 && st.bootPreset <= kPresetCount && kPresets[st.bootPreset - 1].occupied)
            applyPresetSlot(st.bootPreset);
        rebuildEngines();
        mLvl = st.on ? 1.f : 0.f;
        mBri = st.brightness;
        for (int i = 0; i < kMaxSeg; i++)
            segLvl[i] = (i < segCount && segs[i].on) ? 1.f : 0.f;
        selfTest();
        needsDiscovery = true;
        needsPublish = true;
        lastKnxSent = st.on ? 100 : 0;
    }

    void selfTest()
    {
        if (!strip) return;
        const uint32_t cores[3] = {strip->Color(40, 0, 0), strip->Color(0, 40, 0), strip->Color(0, 0, 40)};
        for (int i = 0; i < 3; i++)
        {
            strip->fill(cores[i], 0, st.count);
            strip->show();
            delay(350);
        }
        strip->clear();
        strip->show();
        syncEngines();
    }

    void deinitStrip()
    {
        strip->clear();
        strip->show();
        delete strip;
        strip = nullptr;
        wasOn = false;
        needsSave = false;
        needsUndiscovery = true;
    }

    void undiscover()
    {
        String chip = String(config.chipId);
        publishOnMqtt((String("homeassistant/light/") + chip + "_led/config").c_str(), "", true);
        publishOnMqtt((String("homeassistant/number/") + chip + "_led_speed/config").c_str(), "", true);
        publishOnMqtt((String("homeassistant/number/") + chip + "_led_intensity/config").c_str(), "", true);
        publishOnMqtt((String("homeassistant/number/") + chip + "_led_nightlight/config").c_str(), "", true);
        publishOnMqtt((String("homeassistant/select/") + chip + "_led_palette/config").c_str(), "", true);
        publishOnMqtt((String("homeassistant/select/") + chip + "_led_pattern/config").c_str(), "", true);
        publishOnMqtt((String("homeassistant/select/") + chip + "_led_preset/config").c_str(), "", true);
        publishOnMqtt(tState, "", true);
        publishOnMqtt(tSpeed, "", true);
        publishOnMqtt(tInt, "", true);
        publishOnMqtt(tPalette, "", true);
        publishOnMqtt(tPattern, "", true);
        publishOnMqtt(tPreset, "", true);
        publishOnMqtt(tNl, "", true);
    }
} // namespace

void ledStripSetup()
{
    topics();
    loadPresets();
    load();
    if (created) knxAssign();
    initAfter = millis() + kInitDelayMs;
}

void ledStripLoop()
{
    unsigned long now = millis();
    if (created && !strip)
    {
        if ((long)(now - initAfter) >= 0) initStrip();
        return;
    }
    if (!created && strip) { deinitStrip(); return; }
    if (!strip)
    {
        if (needsUndiscovery && mqttConnected() && config.tryBeginFeatureAccess())
        {
            undiscover();
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
        applyCommand(local);
        needsPublish = true;
    }

    tickDynamicPattern(now);
    tickNightlight(now);

    if (testRequested) { testRequested = false; selfTest(); }
    if (knxOn() && wifiConnected()) knx.loop();

    if (knxPending >= 0)
    {
        const int v = knxPending;
        knxPending = -1;
        st.on = (v == 102) ? !st.on : (v != 0);
        touch();
        needsPublish = true;
        lastKnxSent = st.on ? 100 : 0;
    }
    if (knxOn() && (st.on ? 100 : 0) != lastKnxSent)
    {
        lastKnxSent = st.on ? 100 : 0;
        knx.write_1byte_int(knx.GA_to_address(knxAddr[0], knxAddr[1], knxAddr[2]), lastKnxSent);
    }

    if ((needsPublish || needsDiscovery) && mqttConnected() && config.tryBeginFeatureAccess())
    {
        if (needsDiscovery)
        {
            ledStripSubscribe();
            ledStripDiscovery();
            needsDiscovery = false;
        }
        else
        {
            publishState();
        }
        config.endFeatureAccess();
        needsPublish = false;
    }

    if (needsSave && (saveNow || now - lastChange > kSaveDelayMs))
    {
        needsSave = false;
        saveNow = false;
        if (created) save();
    }

    if (!st.on && mLvl <= 0.f)
    {
        if (wasOn)
        {
            strip->clear();
            strip->show();
            wasOn = false;
            estMA = 0;
            limiting = false;
        }
        return;
    }

    if (!wasOn) { wasOn = true; frameDirty = true; lastRenderMs = now; }
    if (now - lastRenderMs < 16) return;
    unsigned long dtMs = now - lastRenderMs;
    if (dtMs > 100) dtMs = 100;
    lastRenderMs = now;

    const float stepLvl = st.transition == 0 ? 1.f : dtMs / (float)st.transition;
    bool moving = false;
    const float mTarget = st.on ? 1.f : 0.f;
    if (mLvl != mTarget)
    {
        mLvl += (mTarget > mLvl) ? stepLvl : -stepLvl;
        mLvl = constrain(mLvl, 0.f, 1.f);
        moving = true;
    }
    if (fabsf(mBri - st.brightness) > 0.5f && st.transition > 0)
    {
        mBri += (st.brightness - mBri) * (1.f - expf(-3.f * dtMs / (float)st.transition));
        moving = true;
    }
    else
        mBri = st.brightness;

    for (int i = 0; i < segCount; i++)
    {
        const float sTarget = segs[i].on ? 1.f : 0.f;
        if (segLvl[i] != sTarget)
        {
            segLvl[i] += (sTarget > segLvl[i]) ? stepLvl : -stepLvl;
            segLvl[i] = constrain(segLvl[i], 0.f, 1.f);
            moving = true;
        }
    }

    bool changed = frameDirty || moving;
    for (int i = 0; i < segCount; i++)
        if ((segs[i].on || segLvl[i] > 0.f) && engines[i].update(now))
            changed = true;
    if (!changed) return;

    frameDirty = false;
    if (frame.size() != st.count) frame.assign(st.count, led::RGB());
    for (auto &p : frame) p = led::RGB();

    for (int i = 0; i < segCount; i++)
    {
        if (segLvl[i] <= 0.f) continue;
        const int sb = (int)(segs[i].bri * segLvl[i]);
        const std::vector<led::RGB> &px = engines[i].pixels();
        for (size_t k = 0; k < px.size(); k++)
        {
            const size_t idx = segs[i].start + k;
            if (idx >= frame.size()) break;
            frame[idx] = led::RGB((uint8_t)((px[k].r * sb) / 255), (uint8_t)((px[k].g * sb) / 255),
                                  (uint8_t)((px[k].b * sb) / 255));
        }
    }

    const int B = (int)(mBri * mLvl + 0.5f);
    buildGamma();
    uint32_t sum = 0;
    for (uint16_t i = 0; i < st.count; i++)
    {
        led::RGB &p = frame[i];
        p.r = (uint8_t)(gammaLut[(p.r * B) / 255] * st.cal[0] / 100);
        p.g = (uint8_t)(gammaLut[(p.g * B) / 255] * st.cal[1] / 100);
        p.b = (uint8_t)(gammaLut[(p.b * B) / 255] * st.cal[2] / 100);
        sum += p.r + p.g + p.b;
    }

    estMA = led::estimateMA(sum, st.count, st.mAperLED);
    const uint16_t fq = led::limitFactor256(estMA, st.count, st.limitMA);
    limiting = fq < 256;
    if (limiting) estMA = led::estimateMA((uint64_t)sum * fq / 256, st.count, st.mAperLED);

    for (uint16_t i = 0; i < st.count; i++)
    {
        const led::RGB &p = frame[i];
        strip->setPixelColor(i, (uint8_t)(((uint32_t)p.r * fq) >> 8),
                             (uint8_t)(((uint32_t)p.g * fq) >> 8),
                             (uint8_t)(((uint32_t)p.b * fq) >> 8));
    }
    strip->show();
}

bool ledStripHandleMqtt(const char *topic, const char *payload)
{
    if (!created) return false;
    if (!strip) cmdAppliedBeforeInit = true;
    if (strcmp(topic, tSpeedSet) == 0)
    {
        for (int i = 0; i < segCount; i++) segs[i].speed = constrain(atoi(payload), 0, 255);
        touch(); publishState(); return true;
    }
    if (strcmp(topic, tIntSet) == 0)
    {
        for (int i = 0; i < segCount; i++) segs[i].intensity = constrain(atoi(payload), 0, 255);
        touch(); publishState(); return true;
    }
    if (strcmp(topic, tPaletteSet) == 0)
    {
        int p = findPalette(payload);
        if (p >= 0)
        {
            for (int i = 0; i < segCount; i++) segs[i].palette = p;
            activePatternIdx = -1;
            activePresetSlot = 0;
            touch(); publishState();
        }
        return true;
    }
    if (strcmp(topic, tPatternSet) == 0)
    {
        activatePatternByName(payload);
        publishState();
        return true;
    }
    if (strcmp(topic, tPresetSet) == 0)
    {
        int ps = findPreset(payload);
        if (ps >= 0)
        {
            applyPresetSlot(ps);
            publishState();
        }
        return true;
    }
    if (strcmp(topic, tNlSet) == 0)
    {
        startNightlight((uint16_t)max(0, atoi(payload)));
        publishState();
        return true;
    }
    const bool fromCloud = tCloudCmd[0] != '\0' && strcmp(topic, tCloudCmd) == 0;
    if (!fromCloud && strcmp(topic, tCmd) != 0) return false;
    applyCommand(payload);
    publishState();
    return true;
}

void ledStripSubscribe()
{
    if (!created) return;
    subscribeOnMqtt(tCmd);
    subscribeOnMqtt(tSpeedSet);
    subscribeOnMqtt(tIntSet);
    subscribeOnMqtt(tPaletteSet);
    subscribeOnMqtt(tPatternSet);
    subscribeOnMqtt(tPresetSet);
    subscribeOnMqtt(tNlSet);
}

void ledStripDiscovery()
{
    if (!created || !mqttConnected()) return;
    String chip = String(config.chipId);
    String base = String("onofre/") + chip;
    {
        JsonDocument doc;
        JsonObject o = doc.to<JsonObject>();
        o["~"] = base;
        o["name"] = fname;
        o["uniq_id"] = chip + "_led";
        o["has_entity_name"] = true;
        o["schema"] = "json";
        o["cmd_t"] = "~/led/set";
        o["stat_t"] = "~/led/state";
        o["avty_t"] = "~/available";
        o["brightness"] = true;
        o["effect"] = true;
        o["ic"] = "mdi:led-strip-variant";
        JsonArray modes = o["supported_color_modes"].to<JsonArray>();
        modes.add("rgb");
        JsonArray fx = o["effect_list"].to<JsonArray>();
        for (int i = 0; i < led::FX_COUNT; i++) fx.add(led::kEffectNames[i]);
        haDevice(o);
        publishConfig("light", chip + "_led", doc);
    }
    const char *names[2] = {"velocidade", "intensidade"};
    const char *ids[2] = {"_led_speed", "_led_intensity"};
    const char *cmd[2] = {"~/led/speed/set", "~/led/intensity/set"};
    const char *stat[2] = {"~/led/speed", "~/led/intensity"};
    for (int i = 0; i < 2; i++)
    {
        JsonDocument doc;
        JsonObject o = doc.to<JsonObject>();
        o["~"] = base;
        o["name"] = String(fname) + " " + names[i];
        o["uniq_id"] = chip + ids[i];
        o["has_entity_name"] = true;
        o["cmd_t"] = cmd[i];
        o["stat_t"] = stat[i];
        o["avty_t"] = "~/available";
        o["min"] = 0; o["max"] = 255; o["step"] = 1;
        o["mode"] = "slider";
        o["ent_cat"] = "config";
        haDevice(o);
        publishConfig("number", chip + ids[i], doc);
    }
    {
        JsonDocument doc;
        JsonObject o = doc.to<JsonObject>();
        o["~"] = base;
        o["name"] = String(fname) + " Temporizador";
        o["uniq_id"] = chip + "_led_nightlight";
        o["has_entity_name"] = true;
        o["cmd_t"] = "~/led/nightlight/set";
        o["stat_t"] = "~/led/nightlight";
        o["avty_t"] = "~/available";
        o["min"] = 0; o["max"] = 120; o["step"] = 1;
        o["unit_of_meas"] = "min";
        o["ic"] = "mdi:weather-night";
        o["ent_cat"] = "config";
        haDevice(o);
        publishConfig("number", chip + "_led_nightlight", doc);
    }
    {
        JsonDocument doc;
        JsonObject o = doc.to<JsonObject>();
        o["~"] = base;
        o["name"] = String(fname) + " Palete";
        o["uniq_id"] = chip + "_led_palette";
        o["has_entity_name"] = true;
        o["cmd_t"] = "~/led/palette/set";
        o["stat_t"] = "~/led/palette";
        o["avty_t"] = "~/available";
        o["ic"] = "mdi:palette";
        JsonArray opts = o["options"].to<JsonArray>();
        for (int i = 0; i < led::PAL_COUNT; i++) opts.add(led::kPaletteNames[i]);
        haDevice(o);
        publishConfig("select", chip + "_led_palette", doc);
    }
    {
        JsonDocument doc;
        JsonObject o = doc.to<JsonObject>();
        o["~"] = base;
        o["name"] = String(fname) + " Padrão";
        o["uniq_id"] = chip + "_led_pattern";
        o["has_entity_name"] = true;
        o["cmd_t"] = "~/led/pattern/set";
        o["stat_t"] = "~/led/pattern";
        o["avty_t"] = "~/available";
        o["ic"] = "mdi:playlist-play";
        JsonArray opts = o["options"].to<JsonArray>();
        opts.add("Nenhum");
        for (int i = 0; i < kPatternCount; i++) opts.add(kPatterns[i].name);
        haDevice(o);
        publishConfig("select", chip + "_led_pattern", doc);
    }
    {
        JsonDocument doc;
        JsonObject o = doc.to<JsonObject>();
        o["~"] = base;
        o["name"] = String(fname) + " Preset";
        o["uniq_id"] = chip + "_led_preset";
        o["has_entity_name"] = true;
        o["cmd_t"] = "~/led/preset/set";
        o["stat_t"] = "~/led/preset";
        o["avty_t"] = "~/available";
        o["ic"] = "mdi:star";
        JsonArray opts = o["options"].to<JsonArray>();
        opts.add("Nenhum");
        for (int i = 1; i <= kPresetCount; i++)
        {
            if (!kPresets[i - 1].occupied) continue;
            char lbl[32];
            presetLabel(i, lbl, sizeof(lbl));
            opts.add(lbl);
        }
        haDevice(o);
        publishConfig("select", chip + "_led_preset", doc);
    }
    publishState();
}

int ledStripCreate(const char *name, int pin, int count)
{
    if (!name || !*name || strlen(name) >= sizeof(fname)) return 1;
    if (created) return 7;
    if (pin < 0 || !config.validOutputPin((unsigned int)pin)) return 2;
    if (config.pinInUse((unsigned int)pin)) return 5;
    strlcpy(fname, name, sizeof(fname));
    fpin = pin;
    st.count = (count >= 1 && count <= LED_STRIP_MAX) ? count : LED_STRIP_COUNT;
    st.on = false;
    st.brightness = 255;
    segCount = 1;
    segs[0] = Seg();
    segs[0].stop = st.count - 1;
    initAfter = millis() + kCreateDelayMs;
    __sync_synchronize();
    created = true;
    save();
    return 0;
}

void ledStripRemove()
{
    created = false;
    pendingFlag = false;
    needsSave = false;
    LittleFS.remove(kFile);
    LittleFS.remove(kPresetFile);
}

bool ledStripUsesPin(unsigned int pin) { return created && (int)pin == fpin; }

void ledStripConfigJson(JsonVariant root)
{
    if (!created) return;
    JsonObject o = root["ledStrip"].to<JsonObject>();
    o["id"] = "led";
    o["name"] = fname;
    o["pin"] = fpin;
    o["leds"] = st.count;
    o["area"] = knxAddr[0];
    o["line"] = knxAddr[1];
    o["member"] = knxAddr[2];
}

static const char *cloudStripId()
{
    static char id[32];
    snprintf(id, sizeof(id), "%sled", config.chipId);
    return id;
}

void ledStripCloudTopics(const char *username, const char *chipId)
{
    if (!created || username == nullptr || username[0] == '\0')
    {
        tCloudCmd[0] = '\0';
        tCloudState[0] = '\0';
        return;
    }
    snprintf(tCloudCmd, sizeof(tCloudCmd), "%s/%s/light/%s/set", username, chipId, cloudStripId());
    snprintf(tCloudState, sizeof(tCloudState), "%s/%s/light/%s/status", username, chipId, cloudStripId());
}

const char *ledStripCloudWriteTopic() { return tCloudCmd; }

void ledStripFeatureJson(JsonArray features)
{
    if (!created) return;
    JsonObject a = features.add<JsonObject>();
    a["id"] = cloudStripId();
    a["name"] = fname;
    a["family"] = "LIGHT";
    a["driver"] = "LED_STRIP";
    a["state"] = st.on ? "ON" : "OFF";
    a["area"] = knxAddr[0];
    a["line"] = knxAddr[1];
    a["member"] = knxAddr[2];
}

static bool knxMatches(uint8_t a, uint8_t l, uint8_t m)
{
    return knxOn() && knxAddr[0] == a && (knxAddr[1] == l || (l == 0 && m == 0)) && (knxAddr[2] == m || m == 0);
}

bool ledStripKnxWrite(uint8_t a, uint8_t l, uint8_t m, int value)
{
    if (!created || !knxMatches(a, l, m)) return false;
    knxPending = value;
    return true;
}

bool ledStripKnxRead(uint8_t a, uint8_t l, uint8_t m, int &state)
{
    if (!created || !knxOn() || knxAddr[0] != a || knxAddr[1] != l || knxAddr[2] != m) return false;
    state = st.on ? 100 : 0;
    return true;
}

static const char kPage[] PROGMEM = R"rawliteral(<!doctype html><html lang="pt"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Fita LED</title><style>:root{--bg:#f4f5f7;--card:#fff;--tx:#1b1d21;--mut:#6b7280;--ac:#97d700;--bd:#e2e5ea}@media(prefers-color-scheme:dark){:root{--bg:#14161a;--card:#1d2026;--tx:#eceef1;--mut:#9aa1ad;--bd:#2c313a}}*{box-sizing:border-box}body{margin:0;padding:16px;background:var(--bg);color:var(--tx);font:16px system-ui,sans-serif}.w{max-width:460px;margin:0 auto}.c{background:var(--card);border:1px solid var(--bd);border-radius:14px;padding:16px;margin-bottom:12px}h1{font-size:20px;margin:0 0 12px;display:flex;justify-content:space-between;align-items:center;gap:8px}label{display:block;font-size:13px;color:var(--mut);margin:14px 0 6px}label:first-child{margin-top:0}input[type=range]{width:100%;accent-color:var(--ac)}select,input[type=number]{width:100%;padding:10px;border-radius:10px;border:1px solid var(--bd);background:var(--bg);color:var(--tx);font-size:16px}input[type=color]{width:100%;height:46px;border:1px solid var(--bd);border-radius:10px;background:var(--bg);padding:4px}button{padding:10px 18px;border-radius:999px;border:0;font-size:15px;font-weight:600;cursor:pointer;background:var(--bd);color:var(--tx)}button.on{background:var(--ac);color:#10200a}.g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}.v{float:right;color:var(--mut)}#st,#info{font-size:12px;color:var(--mut);text-align:center}a{color:var(--mut)}</style></head><body><div class="w"><div class="c" id="off" style="display:none">Ainda não há nenhuma fita LED. No painel principal, em <b>Funções</b> &rarr; <b>Nova função</b>, escolhe <b>Fita LED</b>, o GPIO e o número de LEDs.</div><div class="c" id="ctl"><h1><span id="nm">Fita LED</span> <button id="pw">Desligada</button></h1><div class="g3"><div><label>Cor 1</label><input type="color" id="co" value="#ffa03c"></div><div><label>Cor 2</label><input type="color" id="c2" value="#000000"></div><div><label>Cor 3</label><input type="color" id="c3" value="#ffffff"></div></div><label>Brilho <span class="v" id="bv"></span></label><input type="range" id="br" min="1" max="255"><label>Efeito</label><select id="fx"></select><label>Palete de Cores (WLED)</label><select id="pl"></select><label>Preset WLED</label><select id="pr"></select><label>Padrão Dinâmico</label><select id="pt"></select><label>Velocidade <span class="v" id="sv"></span></label><input type="range" id="sp" min="0" max="255"><label>Intensidade <span class="v" id="iv"></span></label><input type="range" id="it" min="0" max="255"><label>Número de LEDs</label><input type="number" id="nl" min="1" max="600"></div><div id="info"></div><div id="st"></div></div><script>const $=i=>document.getElementById(i);let on=false,busy=0,t,st2;const hex=c=>'#'+[c.r,c.g,c.b].map(x=>x.toString(16).padStart(2,'0')).join('');const rgb=h=>({r:parseInt(h.substr(1,2),16),g:parseInt(h.substr(3,2),16),b:parseInt(h.substr(5,2),16)});function show(s){$('off').style.display=s.enabled?'none':'block';$('ctl').style.display=s.enabled?'block':'none';$('nm').textContent=s.name||'Fita LED';$('info').textContent=s.enabled?'GPIO'+s.pin+' · '+s.leds+' LEDs':'';if(busy)return;on=s.on;$('pw').textContent=on?'Ligada':'Desligada';$('pw').className=on?'on':'';$('co').value=hex(s.color);if(s.color2)$('c2').value=hex(s.color2);if(s.color3)$('c3').value=hex(s.color3);$('br').value=s.brightness;$('sp').value=s.speed;$('it').value=s.intensity;if(!$('fx').options.length)s.effects.forEach(e=>$('fx').add(new Option(e,e)));$('fx').value=s.effect;if(!$('pl').options.length&&s.palettes)s.palettes.forEach(p=>$('pl').add(new Option(p,p)));if(s.palette)$('pl').value=s.palette;if(!$('pr').options.length&&s.presets){$('pr').add(new Option('Nenhum',0));s.presets.forEach(p=>$('pr').add(new Option(p.slot+': '+p.name,p.slot)))}if(s.preset!==undefined)$('pr').value=s.preset;if(!$('pt').options.length&&s.patterns){$('pt').add(new Option('Nenhum','Nenhum'));s.patterns.forEach(p=>$('pt').add(new Option(p,p)))}if(s.pattern)$('pt').value=s.pattern;if(document.activeElement!==$('nl'))$('nl').value=s.leds;$('bv').textContent=s.brightness;$('sv').textContent=s.speed;$('iv').textContent=s.intensity}async function get(){const reqAt=Date.now();try{const s=await(await fetch('/ledstate')).json();if(busy>reqAt)return;show(s);$('st').textContent=''}catch(e){$('st').textContent='sem ligação'}}function send(o,now){clearTimeout(t);clearTimeout(st2);busy=Date.now();if(o.state){on=o.state==='ON';$('pw').textContent=on?'Ligada':'Desligada';$('pw').className=on?'on':''}const go=async()=>{busy=Date.now();try{await fetch('/led/set',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(o)});st2=setTimeout(()=>{busy=0;get()},250)}catch(e){busy=0;$('st').textContent='erro ao enviar';get()}};now?go():t=setTimeout(go,90)}$('pw').onclick=()=>send({state:on?'OFF':'ON'},1);$('co').oninput=e=>send({state:'ON',color:rgb(e.target.value)});$('c2').oninput=e=>send({color2:rgb(e.target.value)});$('c3').oninput=e=>send({color3:rgb(e.target.value)});$('br').oninput=e=>{$('bv').textContent=e.target.value;send({brightness:+e.target.value})};$('sp').oninput=e=>{$('sv').textContent=e.target.value;send({speed:+e.target.value})};$('it').oninput=e=>{$('iv').textContent=e.target.value;send({intensity:+e.target.value})};$('fx').onchange=e=>send({state:'ON',effect:e.target.value},1);$('pl').onchange=e=>send({state:'ON',palette:e.target.value},1);$('pr').onchange=e=>send({preset:+e.target.value},1);$('pt').onchange=e=>send({state:'ON',pattern:e.target.value},1);$('nl').onchange=e=>send({leds:+e.target.value},1);get();setInterval(()=>{if(!busy)get()},3000);</script></body></html>)rawliteral";

const char *ledStripPageHtml() { return kPage; }
size_t ledStripPageSize() { return sizeof(kPage) - 1; }

bool ledStripQueueCommand(JsonVariantConst json)
{
    if (!created || !strip || pendingFlag || measureJson(json) >= sizeof(pendingBuf))
        return false;
    serializeJson(json, pendingBuf, sizeof(pendingBuf));
    __sync_synchronize();
    pendingFlag = true;
    return true;
}

void ledStripStateJson(JsonVariant &root)
{
    root["enabled"] = (bool)created;
    root["name"] = fname;
    root["pin"] = fpin;
    root["on"] = st.on;
    root["brightness"] = st.brightness;
    JsonObject c = root["color"].to<JsonObject>();
    c["r"] = segs[0].color.r; c["g"] = segs[0].color.g; c["b"] = segs[0].color.b;
    JsonObject c2 = root["color2"].to<JsonObject>();
    c2["r"] = segs[0].color2.r; c2["g"] = segs[0].color2.g; c2["b"] = segs[0].color2.b;
    JsonObject c3 = root["color3"].to<JsonObject>();
    c3["r"] = segs[0].color3.r; c3["g"] = segs[0].color3.g; c3["b"] = segs[0].color3.b;
    root["effect"] = led::kEffectNames[segs[0].effect];
    root["palette"] = led::kPaletteNames[segs[0].palette];
    root["paletteId"] = segs[0].palette;
    root["pattern"] = activePatternIdx >= 0 ? kPatterns[activePatternIdx].name : "Nenhum";
    root["preset"] = activePresetSlot;
    root["bootPreset"] = st.bootPreset;
    root["reverse"] = segs[0].reverse;
    root["mirror"] = segs[0].mirror;
    root["grouping"] = segs[0].grouping;
    root["spacing"] = segs[0].spacing;
    root["offset"] = segs[0].offset;
    root["speed"] = segs[0].speed;
    root["intensity"] = segs[0].intensity;

    JsonObject nl = root["nightlight"].to<JsonObject>();
    nl["on"] = nlActive;
    nl["min"] = nlDurationMin;
    if (nlActive)
    {
        unsigned long totalMs = (unsigned long)nlDurationMin * 60000UL;
        unsigned long el = millis() - nlStartMs;
        nl["remSec"] = el < totalMs ? (uint32_t)((totalMs - el) / 1000UL) : 0;
    }
    else
        nl["remSec"] = 0;

    JsonArray sg = root["segs"].to<JsonArray>();
    for (int i = 0; i < segCount; i++)
    {
        const Seg &g = segs[i];
        JsonObject o = sg.add<JsonObject>();
        o["start"] = g.start;
        o["stop"] = g.stop;
        o["on"] = g.on;
        o["brightness"] = g.bri;
        JsonObject gc = o["color"].to<JsonObject>();
        gc["r"] = g.color.r; gc["g"] = g.color.g; gc["b"] = g.color.b;
        JsonObject gc2 = o["color2"].to<JsonObject>();
        gc2["r"] = g.color2.r; gc2["g"] = g.color2.g; gc2["b"] = g.color2.b;
        JsonObject gc3 = o["color3"].to<JsonObject>();
        gc3["r"] = g.color3.r; gc3["g"] = g.color3.g; gc3["b"] = g.color3.b;
        o["effect"] = led::kEffectNames[g.effect];
        o["palette"] = led::kPaletteNames[g.palette];
        o["paletteId"] = g.palette;
        o["reverse"] = g.reverse;
        o["mirror"] = g.mirror;
        o["grouping"] = g.grouping;
        o["spacing"] = g.spacing;
        o["offset"] = g.offset;
        o["speed"] = g.speed;
        o["intensity"] = g.intensity;
    }

    root["leds"] = st.count;
    root["knx"] = knxAddr[0] > 0;
    root["transition"] = st.transition;
    root["limit"] = st.limitMA;
    root["maPerLed"] = st.mAperLED;
    root["ma"] = estMA;
    root["limited"] = limiting;
    root["colorOrder"] = st.colorOrder;
    root["gamma"] = st.gamma;
    JsonArray cal = root["cal"].to<JsonArray>();
    for (int i = 0; i < 3; i++) cal.add(st.cal[i]);

    JsonArray fx = root["effects"].to<JsonArray>();
    for (int i = 0; i < led::FX_COUNT; i++) fx.add(led::kEffectNames[i]);

    JsonArray pl = root["palettes"].to<JsonArray>();
    for (int i = 0; i < led::PAL_COUNT; i++) pl.add(led::kPaletteNames[i]);

    JsonArray pt = root["patterns"].to<JsonArray>();
    for (int i = 0; i < kPatternCount; i++) pt.add(kPatterns[i].name);

    JsonArray prs = root["presets"].to<JsonArray>();
    for (int i = 0; i < kPresetCount; i++)
    {
        JsonObject po = prs.add<JsonObject>();
        po["slot"] = kPresets[i].slot;
        po["name"] = kPresets[i].name;
        po["occ"] = kPresets[i].occupied;
    }

    JsonArray cps = root["customPalettes"].to<JsonArray>();
    for (int slot = 0; slot < led::kCustomCount; slot++)
    {
        JsonArray stops = cps.add<JsonArray>();
        for (uint8_t k = 0; k < led::customPalettes()[slot].count; k++)
        {
            JsonObject so = stops.add<JsonObject>();
            so["pos"] = led::customPalettes()[slot].stops[k].pos;
            so["r"] = led::customPalettes()[slot].stops[k].r;
            so["g"] = led::customPalettes()[slot].stops[k].g;
            so["b"] = led::customPalettes()[slot].stops[k].b;
        }
    }
}

#endif // LED_STRIP
