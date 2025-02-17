/*
  ==============================================================================

    GlobalDefinitions.h
    Created: 23 Dec 2021 2:15:59am
    Author:  Charles Schiermeyer

  ==============================================================================
*/

#pragma once

#if JUCE_LINUX
#include <inttypes.h>
#include <stdint.h> //for uint16_t on linux
#endif
#include <algorithm> //for std::find
#include <cstdint>
//

namespace Globals //BEGIN_NAMESPACE(Globals)
{
static constexpr int versionHint = 1;

static constexpr int SAMPLES_PER_FRAME = 100;
static constexpr int RENDER_FRAME_RATE = 30;
static constexpr int EDITOR_SYNTH_NUM = 1;

static constexpr int MAX_SYNTH_ENGINES = 4;
static constexpr int MAX_VOICE_COUNT = 8;

static constexpr int MAX_ENCODER_ROWS = 5;
static constexpr int MAX_ENCODERS = 6;

static constexpr int CONTROL_REFRESH_SAMPLES_INTERVAL = 64;

static constexpr float WTSVirtualScaleFactor = 0.88f;

static constexpr int NumThumbnailColumns = 5;
static constexpr int GutterSize = 1;
static constexpr int ThumbnailWidth = GutterSize * 5;

static constexpr size_t NUM_EXP_MATRIX_ROWS = 4;
static constexpr size_t NUM_MIDI_CHANNELS = 16;
static constexpr int Title1FontSize = 28;

static constexpr int NumDisplays = 6;

static double systemBootTime;
} // namespace Globals

typedef struct
{
    std::uint16_t x;
    std::uint16_t y;
} uint16Vec_type;
typedef struct
{
    std::int16_t x;
    std::int16_t y;
} int16Vec_type;
typedef struct
{
    float x;
    float y;
} float2Vec_type;
typedef struct
{
    float t;
    float x;
    float y;
} float3Vec_type;
typedef struct
{
    float k;
    std::uint16_t p;
    std::uint16_t q;
    float period;
} rosePhase_type;

enum class wtsSourcePort : int  // added in 3.6.21 0.0.12
// used in the CC# 'WTS_SRC_PORT' message as the
// value parameter that defines the current MIDI source port
{
    SRC_MIDI_5_PIN,
    SRC_MIDI_USB_DEV,
    SRC_MIDI_CV_GATE,
    SRC_MIDI_CNTRLR,
};

enum class SysexMessageType : int
// NOTE: There are 11 groups. Some only have 1 LED is the group.
// The specific group info is automatic, any LED in a group acts on that group
// No state required in the WTS engine.
// Ex: SetSingleLEDColorInGroup for Menu Enc1 turns off all other Menu Enc LEDs
// Groups are:
// (singles) Power, WTS, Latch, Play, Favs,
// (multibles) Synth A-D, Menu Encoders, Modulation Mode (Lfo - VAL),
//             Modulation Encoders (TBD), Shortcuts (Amp-Filt), User 1-5,
//             Module Select (Voice-Settings), & Vol-Pan
{
    SetAllLEDs,
    SetSingleLEDColor,
    SetSingleLEDColorInGroup, // turns all other LEDs off
    SetColorOfLEDGroup, // turns all LEDs in that group to the RGB color
};

namespace Utilities
{
template <typename ParameterIndxes, typename Index>
static bool findHelper (const ParameterIndxes& container, Index index)
{
    return std::find (container.begin(), container.end(), index) != container.end();
}
} //end namespace Utilities
