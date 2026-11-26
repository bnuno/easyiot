#pragma once

// Effect engine for addressable LED strips, in the WLED spirit.
// Features: 31 WLED Effects, Color Palettes, Primary/Secondary/Tertiary colors,
// Reverse, Mirror, Grouping, Spacing & Offset segment geometry.
// Plain C++11: no Arduino, no libraries, PC-testable.
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include "LedPalettes.h"

namespace led
{
    struct RGB
    {
        uint8_t r, g, b;
        RGB() : r(0), g(0), b(0) {}
        RGB(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_) {}
    };

    enum Effect : uint8_t
    {
        FX_STATIC = 0, FX_BLINK, FX_BREATHE, FX_RAINBOW, FX_COLOR_WIPE,
        FX_CHASE, FX_SCAN, FX_TWINKLE, FX_FIRE, FX_METEOR,
        FX_STROBE, FX_COLOR_CYCLE, FX_THEATER_RAINBOW, FX_RUNNING, FX_SPARKLE,
        FX_PLASMA, FX_CONFETTI, FX_LIGHTNING, FX_CANDLE, FX_POLICE,
        FX_HEARTBEAT, FX_JUGGLE, FX_GRADIENT, FX_NOISE,
        FX_SINELON, FX_BOUNCING_BALLS, FX_FIREWORKS, FX_PACIFICA, FX_COLORWAVES,
        FX_TWINKLEFOX, FX_TWINKLECAT, FX_COUNT
    };

    static const char *const kEffectNames[FX_COUNT] = {
        "Static", "Blink", "Breathe", "Rainbow", "Color Wipe",
        "Chase", "Scan", "Twinkle", "Fire", "Meteor",
        "Strobe", "Color Cycle", "Theater Rainbow", "Running Lights", "Sparkle",
        "Plasma", "Confetti", "Lightning", "Candle", "Police",
        "Heartbeat", "Juggle", "Gradient", "Noise",
        "Sinelon", "Bouncing Balls", "Fireworks", "Pacifica", "Colorwaves",
        "TwinkleFox", "TwinkleCat"
    };

    inline BuiltinPalette *customPalettes()
    {
        static BuiltinPalette pals[kCustomCount];
        static bool initialized = false;
        if (!initialized)
        {
            for (uint8_t i = 0; i < kCustomCount; i++)
            {
                const uint8_t srcIdx = kCustomStartId + i;
                pals[i] = (srcIdx < PAL_COUNT) ? kBuiltinPalettes[srcIdx] : kBuiltinPalettes[0];
            }
            initialized = true;
        }
        return pals;
    }

    inline RGB scale(RGB c, float f)
    {
        if (f < 0.f) f = 0.f;
        if (f > 1.f) f = 1.f;
        return RGB((uint8_t)(c.r * f), (uint8_t)(c.g * f), (uint8_t)(c.b * f));
    }

    inline RGB blend(RGB a, RGB b, float t)
    {
        if (t < 0.f) t = 0.f;
        if (t > 1.f) t = 1.f;
        return RGB((uint8_t)(a.r + (b.r - a.r) * t), (uint8_t)(a.g + (b.g - a.g) * t), (uint8_t)(a.b + (b.b - a.b) * t));
    }

    inline RGB addSat(RGB a, RGB b)
    {
        const int r = a.r + b.r, g = a.g + b.g, bl = a.b + b.b;
        return RGB((uint8_t)(r > 255 ? 255 : r), (uint8_t)(g > 255 ? 255 : g), (uint8_t)(bl > 255 ? 255 : bl));
    }

    inline RGB samplePalette(uint8_t palId, float t, RGB c1, RGB c2 = RGB(0, 0, 0), RGB c3 = RGB(255, 255, 255))
    {
        if (palId == 0 || palId >= PAL_COUNT)
            return c1;
        if (palId >= kDynamicStartId)
        {
            t -= floorf(t);
            if (t < 0.f) t += 1.f;
            const uint8_t dynIdx = palId - kDynamicStartId;
            if (dynIdx == 0) // Duo (Cor 1+2)
            {
                return t < 0.5f ? blend(c1, c2, t * 2.f) : blend(c2, c1, (t - 0.5f) * 2.f);
            }
            if (dynIdx == 1) // Tri (Cor 1+2+3)
            {
                if (t < 0.3333f) return blend(c1, c2, t * 3.f);
                if (t < 0.6666f) return blend(c2, c3, (t - 0.3333f) * 3.f);
                return blend(c3, c1, (t - 0.6666f) * 3.f);
            }
            // Gradiente Cor 1
            RGB dim = scale(c1, 0.18f);
            return t < 0.5f ? blend(c1, dim, t * 2.f) : blend(dim, c1, (t - 0.5f) * 2.f);
        }
        const BuiltinPalette &pal = (palId >= kCustomStartId && palId < kCustomStartId + kCustomCount)
            ? customPalettes()[palId - kCustomStartId]
            : kBuiltinPalettes[palId];
        uint8_t r, g, b;
        sampleStops(pal.stops, pal.count, t, r, g, b);
        return RGB(r, g, b);
    }

    inline RGB hsv(float h, float s, float v)
    {
        h -= floorf(h);
        float i = floorf(h * 6.f);
        float f = h * 6.f - i;
        float p = v * (1.f - s), q = v * (1.f - f * s), t = v * (1.f - (1.f - f) * s);
        float r, g, b;
        switch (((int)i) % 6)
        {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
        }
        return RGB((uint8_t)(r * 255), (uint8_t)(g * 255), (uint8_t)(b * 255));
    }

    inline void rgb2hsv(RGB c, float &h, float &s, float &v)
    {
        const float r = c.r / 255.f, g = c.g / 255.f, b = c.b / 255.f;
        const float mx = fmaxf(r, fmaxf(g, b)), mn = fminf(r, fminf(g, b)), d = mx - mn;
        v = mx;
        s = mx <= 0.f ? 0.f : d / mx;
        if (d <= 0.f) { h = 0.f; return; }
        if (mx == r) h = fmodf((g - b) / d, 6.f);
        else if (mx == g) h = (b - r) / d + 2.f;
        else h = (r - g) / d + 4.f;
        h /= 6.f;
        if (h < 0.f) h += 1.f;
    }

    inline float hash01(uint32_t n)
    {
        n = (n << 13) ^ n;
        n = n * (n * n * 15731u + 789221u) + 1376312589u;
        return (n & 0x7fffffffu) / 2147483648.f;
    }

    inline float noise1(float x)
    {
        const float fl = floorf(x), f = x - fl;
        const uint32_t i = (uint32_t)(int32_t)fl;
        const float t = f * f * (3.f - 2.f * f);
        const float a = hash01(i), b = hash01(i + 1);
        return a + (b - a) * t;
    }

    inline uint32_t estimateMA(uint32_t channelSum, uint16_t leds, uint16_t mAperLED)
    {
        return (uint32_t)((uint64_t)channelSum * mAperLED / 765u) + leds;
    }

    inline uint16_t limitFactor256(uint32_t estMA, uint16_t leds, uint32_t limitMA)
    {
        if (limitMA == 0 || estMA <= limitMA) return 256;
        const uint32_t idle = leds;
        if (limitMA <= idle) return 0;
        const uint32_t active = estMA - idle;
        return (uint16_t)(((uint64_t)(limitMA - idle) * 256u) / active);
    }

    class Engine
    {
    public:
        void resize(uint16_t n)
        {
            if (n < 1) n = 1;
            _physN = n;
            _px.assign(n, RGB());
            recomputeVirtualSize();
            _dirty = true;
        }

        void setTransition(uint16_t ms) { _trMs = ms; }
        void markDirty() { _dirty = true; }

        void set(Effect e, RGB c, uint8_t palette, uint8_t speed, uint8_t intensity,
                 bool reverse = false, bool mirror = false,
                 RGB c2 = RGB(0, 0, 0), RGB c3 = RGB(255, 255, 255),
                 uint8_t grouping = 1, uint8_t spacing = 0, int16_t offset = 0)
        {
            if (e >= FX_COUNT) e = FX_STATIC;
            if (palette >= PAL_COUNT) palette = 0;
            if (grouping < 1) grouping = 1;
            if (grouping > 16) grouping = 16;
            if (spacing > 15) spacing = 15;
            if (e != _fx)
            {
                _fx = e;
                _phase = 0.f;
                _acc = 0;
                for (auto &p : _vpx) p = RGB();
                for (auto &h : _heat) h = 0;
                _head = 0.f;
                _flash = 0;
                _lvl = _tgt = 1.f;
            }
            _tc = c;
            _color2 = c2;
            _color3 = c3;
            if (!_hasColor || _trMs == 0)
            {
                _color = c;
                _cf[0] = c.r; _cf[1] = c.g; _cf[2] = c.b;
                _hasColor = true;
            }
            _palette = palette;
            _speed = speed;
            _intensity = intensity;
            _reverse = reverse;
            _mirror = mirror;
            if (_grouping != grouping || _spacing != spacing)
            {
                _grouping = grouping;
                _spacing = spacing;
                recomputeVirtualSize();
            }
            _offset = offset;
            _dirty = true;
        }

        void set(Effect e, RGB c, uint8_t speed, uint8_t intensity)
        {
            set(e, c, 0, speed, intensity, false, false, RGB(0, 0, 0), RGB(255, 255, 255), 1, 0, 0);
        }

        uint16_t size() const { return _physN; }
        const std::vector<RGB> &pixels() const { return _px; }

        bool update(uint32_t now)
        {
            uint32_t dt = _last ? now - _last : 0;
            if (_last && dt < kMinFrameMs) return false;
            _last = now;
            if (dt > 250) dt = 250;

            float rate = 0.1f + (_speed / 255.f) * 3.9f;
            _phase += rate * (dt / 1000.f);
            if (_phase > 1000.f) _phase -= 1000.f;
            const bool colorMoving = stepColor(dt);

            switch (_fx)
            {
            case FX_STATIC:
                if (!_dirty && !colorMoving) return false;
                staticFill();
                break;
            case FX_BLINK: blink(); break;
            case FX_BREATHE: breathe(); break;
            case FX_RAINBOW: rainbow(); break;
            case FX_COLOR_WIPE: colorWipe(); break;
            case FX_CHASE: chase(); break;
            case FX_SCAN: scan(); break;
            case FX_TWINKLE: stepped(dt, &Engine::twinkleStep); break;
            case FX_FIRE: stepped(dt, &Engine::fireStep); break;
            case FX_METEOR: stepped(dt, &Engine::meteorStep); break;
            case FX_STROBE: strobe(); break;
            case FX_COLOR_CYCLE: fill(_palette == 0 ? hsv(frac(), 1.f, 1.f) : palColor(frac())); break;
            case FX_THEATER_RAINBOW: theaterRainbow(); break;
            case FX_RUNNING: runningLights(); break;
            case FX_SPARKLE: stepped(dt, &Engine::sparkleStep); break;
            case FX_PLASMA: plasma(); break;
            case FX_CONFETTI: stepped(dt, &Engine::confettiStep); break;
            case FX_LIGHTNING: stepped(dt, &Engine::lightningStep); break;
            case FX_CANDLE: stepped(dt, &Engine::candleStep); break;
            case FX_POLICE: police(); break;
            case FX_HEARTBEAT: heartbeat(); break;
            case FX_JUGGLE: stepped(dt, &Engine::juggleStep); break;
            case FX_GRADIENT: gradient(); break;
            case FX_NOISE: noise(); break;
            case FX_SINELON: stepped(dt, &Engine::sinelonStep); break;
            case FX_BOUNCING_BALLS: bouncingBalls(); break;
            case FX_FIREWORKS: stepped(dt, &Engine::fireworksStep); break;
            case FX_PACIFICA: pacifica(); break;
            case FX_COLORWAVES: colorwaves(); break;
            case FX_TWINKLEFOX: twinkleFox(); break;
            case FX_TWINKLECAT: twinkleCat(); break;
            default: break;
            }

            applySegmentTransforms();
            _dirty = false;
            return true;
        }

    private:
        static constexpr uint32_t kMinFrameMs = 16;
        static constexpr uint32_t kTickMs = 30;
        uint16_t _physN = 0;
        uint16_t _n = 0;
        std::vector<RGB> _vpx;
        std::vector<RGB> _px;
        std::vector<uint8_t> _heat;
        Effect _fx = FX_STATIC;
        uint8_t _palette = 0;
        RGB _color{255, 160, 60}, _tc{255, 160, 60};
        RGB _color2{0, 0, 0}, _color3{255, 255, 255};
        float _cf[3] = {255.f, 160.f, 60.f};
        bool _hasColor = false;
        uint16_t _trMs = 400;
        uint8_t _speed = 128, _intensity = 128;
        bool _reverse = false, _mirror = false;
        uint8_t _grouping = 1, _spacing = 0;
        int16_t _offset = 0;
        float _phase = 0.f;
        uint32_t _last = 0, _acc = 0;
        bool _dirty = true;
        float _head = 0.f;
        int _flash = 0;
        uint16_t _la = 0, _lb = 0;
        float _lvl = 1.f, _tgt = 1.f;
        uint32_t _rng = 0x12345678u;

        void recomputeVirtualSize()
        {
            if (_physN < 1) _physN = 1;
            const uint16_t stride = (uint16_t)_grouping + (uint16_t)_spacing;
            uint16_t vn = (_physN + stride - 1) / stride;
            if (vn < 1) vn = 1;
            if (vn != _n || _vpx.size() != vn)
            {
                _n = vn;
                _vpx.assign(vn, RGB());
                _heat.assign(vn, 0);
            }
        }

        uint32_t rnd()
        {
            _rng ^= _rng << 13;
            _rng ^= _rng >> 17;
            _rng ^= _rng << 5;
            return _rng;
        }

        float frac() const { return _phase - floorf(_phase); }
        RGB palColor(float t) const { return samplePalette(_palette, t, _color, _color2, _color3); }

        bool stepColor(uint32_t dt)
        {
            if (_trMs == 0) { _color = _tc; _cf[0] = _tc.r; _cf[1] = _tc.g; _cf[2] = _tc.b; return false; }
            const float step = 255.f * dt / _trMs;
            bool moved = false;
            const uint8_t t[3] = {_tc.r, _tc.g, _tc.b};
            for (int i = 0; i < 3; i++)
            {
                const float d = t[i] - _cf[i];
                if (fabsf(d) <= step) _cf[i] = t[i];
                else { _cf[i] += d > 0 ? step : -step; moved = true; }
            }
            _color = RGB((uint8_t)_cf[0], (uint8_t)_cf[1], (uint8_t)_cf[2]);
            return moved;
        }

        void fill(RGB c) { for (auto &p : _vpx) p = c; }
        void staticFill()
        {
            if (_palette == 0) { fill(_color); return; }
            for (uint16_t i = 0; i < _n; i++)
                _vpx[i] = palColor(_n > 1 ? (float)i / (_n - 1) : 0.f);
        }

        void applySegmentTransforms()
        {
            if (_px.size() != _physN) _px.assign(_physN, RGB());
            std::vector<RGB> work = _vpx;
            if (_mirror && _n > 1)
            {
                const uint16_t half = (_n + 1) / 2;
                for (uint16_t i = 0; i < half; i++)
                    work[_n - 1 - i] = work[i];
            }
            if (_reverse && _n > 1)
            {
                for (uint16_t i = 0; i < _n / 2; i++)
                {
                    RGB tmp = work[i];
                    work[i] = work[_n - 1 - i];
                    work[_n - 1 - i] = tmp;
                }
            }

            const uint16_t stride = (uint16_t)_grouping + (uint16_t)_spacing;
            for (uint16_t i = 0; i < _physN; i++)
            {
                const uint16_t vIdx = i / stride;
                const uint16_t posInGrp = i % stride;
                if (posInGrp < _grouping && vIdx < _n)
                    _px[i] = work[vIdx];
                else
                    _px[i] = _color2;
            }

            if (_offset != 0 && _physN > 1)
            {
                int32_t off = _offset % (int32_t)_physN;
                if (off < 0) off += (int32_t)_physN;
                if (off > 0)
                {
                    std::vector<RGB> tmp(_physN);
                    for (uint16_t i = 0; i < _physN; i++)
                    {
                        const uint16_t src = (uint16_t)(((int32_t)i - off + (int32_t)_physN) % (int32_t)_physN);
                        tmp[i] = _px[src];
                    }
                    _px.swap(tmp);
                }
            }
        }

        void blink()
        {
            const bool on = frac() < 0.5f;
            for (uint16_t i = 0; i < _n; i++)
                _vpx[i] = on ? palColor(_n > 1 ? (float)i / (_n - 1) : 0.f) : _color2;
        }

        void breathe()
        {
            const float v = (sinf(_phase * 6.2831853f) + 1.f) * 0.5f;
            for (uint16_t i = 0; i < _n; i++)
                _vpx[i] = blend(_color2, palColor(_phase * 0.15f + (float)i / (_n * 2.f)), v);
        }

        void rainbow()
        {
            const float reps = 1.f + (_intensity / 64);
            for (uint16_t i = 0; i < _n; i++)
            {
                const float t = (float)i / _n * reps + frac();
                _vpx[i] = _palette == 0 ? hsv(t, 1.f, 1.f) : palColor(t);
            }
        }

        void colorWipe()
        {
            float p = frac() * 2.f;
            uint16_t nLit = (uint16_t)((p < 1.f ? p : 2.f - p) * _n + 0.5f);
            for (uint16_t i = 0; i < _n; i++)
                _vpx[i] = i < nLit ? palColor((float)i / _n) : _color2;
        }

        void chase()
        {
            int gap = 6 - _intensity / 52;
            if (gap < 2) gap = 2;
            int off = (int)(_phase * 8.f);
            for (uint16_t i = 0; i < _n; i++)
                _vpx[i] = ((i + off) % gap == 0) ? palColor((float)i / _n + frac() * 0.5f) : _color2;
        }

        void scan()
        {
            float t = fmodf(_phase * 0.5f, 1.f) * 2.f;
            float p = t < 1.f ? t : 2.f - t;
            float centre = p * (_n - 1);
            float size = 1.f + (_intensity / 255.f) * (_n / 6.f + 2.f);
            RGB c = palColor(p);
            for (uint16_t i = 0; i < _n; i++)
                _vpx[i] = blend(_color2, c, 1.f - fabsf(i - centre) / size);
        }

        typedef void (Engine::*StepFn)();
        void stepped(uint32_t dt, StepFn fn)
        {
            _acc += dt;
            while (_acc >= kTickMs) { (this->*fn)(); _acc -= kTickMs; }
        }

        void twinkleStep()
        {
            for (uint16_t i = 0; i < _n; i++) _heat[i] = _heat[i] > 12 ? _heat[i] - 12 : 0;
            int tries = 1 + _speed / 64;
            for (int k = 0; k < tries; k++)
                if ((rnd() % 255) < (uint32_t)(10 + _intensity / 4)) _heat[rnd() % _n] = 255;
            for (uint16_t i = 0; i < _n; i++)
                _vpx[i] = blend(_color2, palColor((float)i / _n + frac()), _heat[i] / 255.f);
        }

        void fireStep()
        {
            int cooling = 55, sparking = 50 + _intensity / 2;
            for (uint16_t i = 0; i < _n; i++)
            {
                int cool = rnd() % ((cooling * 10) / _n + 2);
                _heat[i] = _heat[i] > cool ? _heat[i] - cool : 0;
            }
            for (int k = _n - 1; k >= 2; k--)
                _heat[k] = (_heat[k - 1] + _heat[k - 2] + _heat[k - 2]) / 3;
            if ((int)(rnd() % 255) < sparking)
            {
                int y = rnd() % (_n < 7 ? _n : 7);
                int h = _heat[y] + 160 + (int)(rnd() % 95);
                _heat[y] = h > 255 ? 255 : h;
            }
            for (uint16_t j = 0; j < _n; j++)
            {
                if (_palette == 0)
                {
                    uint8_t t192 = (uint8_t)((_heat[j] / 255.f) * 191);
                    uint8_t ramp = (uint8_t)((t192 & 0x3F) << 2);
                    if (t192 > 0x80) _vpx[j] = RGB(255, 255, ramp);
                    else if (t192 > 0x40) _vpx[j] = RGB(255, ramp, 0);
                    else _vpx[j] = RGB(ramp, 0, 0);
                }
                else
                {
                    float hn = _heat[j] / 255.f;
                    _vpx[j] = scale(palColor(hn), hn * 1.35f);
                }
            }
        }

        void meteorStep()
        {
            int fade = 20 + (255 - _intensity) / 4;
            for (auto &p : _vpx)
            {
                p.r = (uint8_t)((p.r * (256 - fade)) >> 8);
                p.g = (uint8_t)((p.g * (256 - fade)) >> 8);
                p.b = (uint8_t)((p.b * (256 - fade)) >> 8);
            }
            _head += 0.1f + (_speed / 255.f) * 1.4f;
            if (_head >= _n + 8) _head = 0.f;
            if ((int)_head < (int)_n) _vpx[(int)_head] = palColor(_head / _n);
        }

        void strobe()
        {
            const float w = 0.05f + 0.3f * (_intensity / 255.f);
            fill(fmodf(_phase * 4.f, 1.f) < w ? palColor(_phase * 0.5f) : _color2);
        }

        void theaterRainbow()
        {
            const int gap = 3, off = (int)(_phase * 6.f);
            for (uint16_t i = 0; i < _n; i++)
            {
                float t = frac() + (float)i / _n;
                _vpx[i] = ((i + off) % gap == 0) ? (_palette == 0 ? hsv(t, 1.f, 1.f) : palColor(t)) : _color2;
            }
        }

        void runningLights()
        {
            const float waves = 1.f + (_intensity / 64);
            for (uint16_t i = 0; i < _n; i++)
                _vpx[i] = blend(_color2, palColor((float)i / _n + frac() * 0.3f), (sinf(6.2831853f * (waves * i / _n - _phase)) + 1.f) * 0.5f);
        }

        void plasma()
        {
            const float t = _phase * 6.2831853f, spread = 0.15f + 0.35f * (_intensity / 255.f);
            for (uint16_t i = 0; i < _n; i++)
            {
                const float v = (sinf(i * 0.15f + t) + sinf(i * 0.07f - t * 1.3f) + sinf(i * 0.11f + t * 0.7f)) / 3.f;
                _vpx[i] = _palette == 0 ? hsv(0.6f + v * spread, 1.f, 1.f) : palColor(0.5f + v * (spread * 1.6f) + frac() * 0.25f);
            }
        }

        void police()
        {
            const float f = fmodf(_phase * 2.f, 1.f);
            const bool left = f < 0.5f;
            const float sub = fmodf(f * 2.f, 1.f);
            const bool on = fmodf(sub * 3.f, 1.f) < 0.6f;
            for (uint16_t i = 0; i < _n; i++)
            {
                const bool leftHalf = i < _n / 2 || _n == 1;
                _vpx[i] = (on && leftHalf == left) ? (left ? RGB(255, 0, 0) : RGB(0, 0, 255)) : RGB();
            }
        }

        void heartbeat()
        {
            const float f = fmodf(_phase * 0.5f, 1.f);
            const float a = (f - 0.12f) / 0.05f, b = (f - 0.32f) / 0.06f;
            const float amp = expf(-a * a) + 0.6f * expf(-b * b);
            for (uint16_t i = 0; i < _n; i++)
                _vpx[i] = blend(_color2, palColor((float)i / _n), amp);
        }

        void gradient()
        {
            float h, s, v;
            rgb2hsv(_color, h, s, v);
            const float spread = 0.1f + 0.5f * (_intensity / 255.f);
            for (uint16_t i = 0; i < _n; i++)
            {
                const float u = (float)i / _n + frac();
                const float w = 0.5f - 0.5f * cosf(6.2831853f * u);
                _vpx[i] = _palette == 0 ? hsv(h + spread * w, s, v) : palColor(w);
            }
        }

        void noise()
        {
            const float zoom = 0.10f + 0.25f * (_intensity / 255.f);
            for (uint16_t i = 0; i < _n; i++)
            {
                float v = noise1(i * zoom + _phase * 3.f) * 1.5f - 0.25f;
                if (v < 0.f) v = 0.f;
                if (v > 1.f) v = 1.f;
                _vpx[i] = _palette == 0 ? scale(_color, v) : palColor(v);
            }
        }

        void bouncingBalls()
        {
            const int balls = 1 + (int)((_intensity / 255.f) * 4.f);
            fill(_color2);
            for (int b = 0; b < balls; b++)
            {
                const float f = fmodf(_phase * (0.65f + 0.14f * b) + b * 0.27f, 1.f);
                const float u = 1.f - fabsf(2.f * f - 1.f);
                const float height = 1.f - (1.f - u) * (1.f - u);
                int idx = (int)(height * (_n - 1) + 0.5f);
                if (idx < 0) idx = 0;
                if (idx >= (int)_n) idx = (int)_n - 1;
                RGB c = _palette == 0 ? hsv((float)b / balls, 1.f, 1.f) : palColor(balls > 1 ? (float)b / (balls - 1) : 0.f);
                _vpx[idx] = addSat(_vpx[idx], c);
            }
        }

        void pacifica()
        {
            const float t = _phase * 1.4f;
            const float sf = 0.6f + (_intensity / 255.f) * 1.2f;
            for (uint16_t i = 0; i < _n; i++)
            {
                const float w1 = (sinf(i * 0.18f + t * 1.1f) + 1.f) * 0.5f;
                const float w2 = (sinf(i * 0.29f - t * 0.85f * sf) + 1.f) * 0.5f;
                const float w3 = (sinf(i * 0.43f + t * 1.7f) + 1.f) * 0.5f;
                const float crest = powf(w1 * 0.45f + w2 * 0.35f + w3 * 0.2f, 1.5f);
                if (_palette == 0)
                {
                    const RGB deep(2, 16, 48), mid(0, 115, 168), foam(165, 245, 255);
                    _vpx[i] = crest > 0.72f ? blend(mid, foam, (crest - 0.72f) / 0.28f) : blend(deep, mid, crest / 0.72f);
                }
                else
                    _vpx[i] = palColor(crest);
            }
        }

        void colorwaves()
        {
            const float waves = 1.5f + (_intensity / 255.f) * 3.5f;
            for (uint16_t i = 0; i < _n; i++)
            {
                const float u = (float)i / _n;
                const float s1 = sinf(u * 6.2831853f * waves + _phase * 2.2f);
                const float s2 = sinf(u * 6.2831853f * (waves * 0.6f) - _phase * 1.3f);
                const float t = 0.5f + 0.32f * s1 + 0.18f * s2 + frac();
                _vpx[i] = _palette == 0 ? hsv(t, 0.9f, 1.f) : palColor(t);
            }
        }

        void twinkleFox()
        {
            const float density = 0.25f + 0.65f * (_intensity / 255.f);
            for (uint16_t i = 0; i < _n; i++)
            {
                const float h = hash01((uint32_t)i * 97u + 13u);
                if (h > density) { _vpx[i] = _color2; continue; }
                const float freq = 0.45f + hash01((uint32_t)i * 31u + 7u) * 0.9f;
                const float phOff = hash01((uint32_t)i * 53u + 19u);
                const float w = sinf(6.2831853f * fmodf(_phase * freq + phOff, 1.f));
                const float amp = w > 0.f ? w * w * w : 0.f;
                const RGB col = _palette == 0 ? _color : palColor(hash01((uint32_t)i * 17u + 3u));
                _vpx[i] = blend(_color2, col, amp);
            }
        }

        void twinkleCat()
        {
            const float density = 0.22f + 0.72f * (_intensity / 255.f);
            for (uint16_t i = 0; i < _n; i++)
            {
                const float h = hash01((uint32_t)i * 131u + 29u);
                if (h > density) { _vpx[i] = _color2; continue; }
                const float freq = 0.55f + hash01((uint32_t)i * 47u + 11u) * 1.1f;
                const float phOff = hash01((uint32_t)i * 71u + 23u);
                const float u = fmodf(_phase * freq + phOff, 1.f);
                const float amp = u < 0.12f ? (u / 0.12f) : powf(1.f - (u - 0.12f) / 0.88f, 2.4f);
                const float palT = frac() * 0.35f + hash01((uint32_t)i * 19u + 5u);
                const RGB col = _palette == 0 ? _color : palColor(palT);
                _vpx[i] = blend(_color2, col, amp);
            }
        }

        void sparkleStep()
        {
            for (uint16_t i = 0; i < _n; i++) _heat[i] = _heat[i] > 60 ? _heat[i] - 60 : 0;
            const int tries = 1 + _speed / 90;
            for (int k = 0; k < tries; k++)
                if ((rnd() % 255) < (uint32_t)(15 + _intensity / 2)) _heat[rnd() % _n] = 255;
            for (uint16_t i = 0; i < _n; i++)
                _vpx[i] = blend(scale(palColor((float)i / _n), 0.22f), _color3, _heat[i] / 255.f);
        }

        void confettiStep()
        {
            for (auto &p : _vpx)
            {
                p.r = (uint8_t)((p.r * 235) >> 8);
                p.g = (uint8_t)((p.g * 235) >> 8);
                p.b = (uint8_t)((p.b * 235) >> 8);
            }
            const int tries = 1 + _speed / 90;
            for (int k = 0; k < tries; k++)
                if ((rnd() % 255) < (uint32_t)(20 + _intensity / 2))
                {
                    float h = (rnd() % 256) / 256.f;
                    _vpx[rnd() % _n] = _palette == 0 ? hsv(h, 1.f, 1.f) : palColor(h);
                }
        }

        void lightningStep()
        {
            if (_flash > 0)
            {
                const float b = 0.6f + 0.4f * ((rnd() % 100) / 100.f);
                const RGB c = palColor(0.85f);
                for (uint16_t i = _la; i <= _lb && i < _n; i++) _vpx[i] = scale(c, b);
                _flash--;
                return;
            }
            for (auto &p : _vpx)
            {
                p.r = (uint8_t)((p.r * 90) >> 8);
                p.g = (uint8_t)((p.g * 90) >> 8);
                p.b = (uint8_t)((p.b * 90) >> 8);
            }
            if ((rnd() % 1000) < (uint32_t)(3 + _intensity / 12))
            {
                _flash = 1 + rnd() % 4;
                const uint16_t len = 1 + _n / 4 + rnd() % (1 + _n / 4);
                _la = rnd() % _n;
                _lb = _la + len;
            }
        }

        void candleStep()
        {
            if (rnd() % 8 == 0) _tgt = 0.45f + 0.55f * ((rnd() % 100) / 100.f);
            _lvl += (_tgt - _lvl) * 0.25f;
            for (uint16_t i = 0; i < _n; i++)
                _vpx[i] = scale(palColor(0.5f + ((float)i / _n) * 0.3f), _lvl * (0.88f + 0.12f * ((rnd() % 100) / 100.f)));
        }

        void juggleStep()
        {
            for (auto &p : _vpx)
            {
                p.r = (uint8_t)((p.r * 200) >> 8);
                p.g = (uint8_t)((p.g * 200) >> 8);
                p.b = (uint8_t)((p.b * 200) >> 8);
            }
            const int dots = 3 + _intensity / 64;
            for (int k = 0; k < dots; k++)
            {
                const float pos = (sinf(6.2831853f * _phase * (0.35f + 0.21f * k) + k * 1.3f) + 1.f) * 0.5f * (_n - 1);
                const int idx = (int)pos;
                _vpx[idx] = addSat(_vpx[idx], _palette == 0 ? hsv((float)k / dots, 1.f, 1.f) : palColor((float)k / dots));
            }
        }

        void sinelonStep()
        {
            const int fade = 12 + (255 - _intensity) / 5;
            for (auto &p : _vpx)
            {
                p.r = (uint8_t)((p.r * (256 - fade)) >> 8);
                p.g = (uint8_t)((p.g * (256 - fade)) >> 8);
                p.b = (uint8_t)((p.b * (256 - fade)) >> 8);
            }
            int idx = (int)(((sinf(_phase * 3.1415926f) + 1.f) * 0.5f) * (_n - 1) + 0.5f);
            if (idx < 0) idx = 0;
            if (idx >= (int)_n) idx = (int)_n - 1;
            _vpx[idx] = addSat(_vpx[idx], palColor(frac()));
        }

        void fireworksStep()
        {
            for (auto &p : _vpx)
            {
                p.r = (uint8_t)((p.r * 212) >> 8);
                p.g = (uint8_t)((p.g * 212) >> 8);
                p.b = (uint8_t)((p.b * 212) >> 8);
            }
            if ((rnd() % 100) < (uint32_t)(4 + _intensity / 18))
            {
                const int center = (int)(rnd() % _n);
                const int spread = 2 + (int)((_intensity / 255.f) * 5.f);
                const float h = (rnd() % 256) / 256.f;
                const RGB col = _palette == 0 ? hsv(h, 1.f, 1.f) : palColor(h);
                for (int d = -spread; d <= spread; d++)
                {
                    const int p = center + d;
                    if (p >= 0 && p < (int)_n)
                    {
                        const float atten = 1.f - fabsf((float)d) / (spread + 1);
                        _vpx[p] = addSat(_vpx[p], scale(col, atten));
                    }
                }
            }
        }
    };
} // namespace led
