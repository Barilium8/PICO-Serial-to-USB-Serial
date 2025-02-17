/*
  ==============================================================================

    LED.h
    Created: 4 Sep 2023 3:14:58pm
    Author:  Charles Schiermeyer

  ==============================================================================
*/

#pragma once

#include "GlobalDefinitions.h"

enum LEDMessageData : int
{
    MessageType,
    LedNumber,
    IsOn,
    RedValue,
    GreenValue,
    BlueValue,
    END_OF_LIST,
};

namespace LEDs
{
/*
enum ChangedFlags
{
    Synths      = 1 << 1, //
    Modules     = 1 << 2, //
    Encoders    = 1 << 3, //
    UserPages   = 1 << 4, //
    LfoEnvs     = 1 << 5, //
    Mixer       = 1 << 6, //
    AmpFilter   = 1 << 7, //
    Latch       = 1 << 8, //
    Play        = 1 << 9,//
    Fav         = 1 << 10, //
    Wts         = 1 << 11, //
    Power       = 1 << 12,
    ModEncoders = 1 << 13,
};
*/
enum class Numbers : unsigned char
{
    Power,
    WTS,

    SynthA,
    SynthB,
    SynthC,
    SynthD,

    VAL,
    LFO,
    ENV,
    EXP,

    AmpModsShortcut,
    FilterCutoffModsShortcut,

    UserPage_1,
    UserPage_2,
    UserPage_3,
    UserPage_4,
    UserPage_5,
    UserPage_6,

    Volume,
    Pan,

    Menu_Encoder0,
    Menu_Encoder1,
    Menu_Encoder2,
    Menu_Encoder3,
    Menu_Encoder4,
    Menu_Encoder5,

    Voice,
    Terrain,
    Path,
    Filter,
    Mixer,
    PatchLib,
    Effects,
    Settings,

    Latch,
    Play,
    Favorites,

    Seq,
    Arp,
    SeqArp_Rec,
    SeqArp_Stop,
    SeqArp_PlayPause,

    Modulator_Encoder0,
    Modulator_Encoder1,
    Modulator_Encoder2,
    Modulator_Encoder3,

    END_OF_LIST
};

static inline const auto getModulatorLEDs()
{
    auto ledsToReturn = std::array{
        LEDs::Numbers::Modulator_Encoder0,
        LEDs::Numbers::Modulator_Encoder1,
        LEDs::Numbers::Modulator_Encoder2,
        LEDs::Numbers::Modulator_Encoder3,
    };

    return ledsToReturn;
}

static inline const auto getValLfoEnvExpLEDs()
{
    auto ledsToTurn = std::array{
        LEDs::Numbers::VAL,
        LEDs::Numbers::LFO,
        LEDs::Numbers::ENV,
        LEDs::Numbers::EXP,
    };

    return ledsToTurn;
}

static inline const auto getSynthLEDs()
{
    auto leds = std::array{
        LEDs::Numbers::SynthA,
        LEDs::Numbers::SynthB,
        LEDs::Numbers::SynthC,
        LEDs::Numbers::SynthD,
    };

    return leds;
}

static inline const auto getMenuEncoderLEDs()
{
    auto leds = std::array{
        LEDs::Numbers::Menu_Encoder0,
        LEDs::Numbers::Menu_Encoder1,
        LEDs::Numbers::Menu_Encoder2,
        LEDs::Numbers::Menu_Encoder3,
        LEDs::Numbers::Menu_Encoder4,
        LEDs::Numbers::Menu_Encoder5,
    };

    return leds;
}

static inline const auto getUserEncoderLEDs()
{
    auto leds = std::array{
        LEDs::Numbers::UserPage_1,
        LEDs::Numbers::UserPage_2,
        LEDs::Numbers::UserPage_3,
        LEDs::Numbers::UserPage_4,
        LEDs::Numbers::UserPage_5,
        LEDs::Numbers::UserPage_6,
    };

    return leds;
}

static inline const auto getModuleLEDs()
{
    auto leds = std::array{
        LEDs::Numbers::Voice,
        LEDs::Numbers::Terrain,
        LEDs::Numbers::Path,
        LEDs::Numbers::Filter,
        LEDs::Numbers::Mixer,
        LEDs::Numbers::PatchLib,
        LEDs::Numbers::Effects,
        LEDs::Numbers::Settings,
    };

    return leds;
}

static inline const auto getShortcutLEDs()
{
    auto leds = std::array{
        LEDs::Numbers::AmpModsShortcut,
        LEDs::Numbers::FilterCutoffModsShortcut,
    };

    return leds;
}

static inline const auto getAlternateModeLEDs()
{
    auto leds = std::array{
        LEDs::Numbers::Latch, // aka Hold
        LEDs::Numbers::Play, // aka Aud(ition)
        LEDs::Numbers::Favorites, // aka Favs
    };

    return leds;
}

static inline const auto getSeqArpModeLEDs()
{
    auto leds = std::array{
        LEDs::Numbers::Seq,
        LEDs::Numbers::Arp,
        LEDs::Numbers::SeqArp_Rec,
        LEDs::Numbers::SeqArp_Stop,
        LEDs::Numbers::SeqArp_PlayPause,
    };

    return leds;
}

// static inline const auto getPlayFavLEDs()
// {
//     auto leds = std::array{
//         LEDs::Numbers::Play,
//         LEDs::Numbers::Favorites,
//     };
//
//     return leds;
// }

static inline const auto getMixerMacLEDs()
{
    auto leds = std::array{
        LEDs::Numbers::Volume,
        LEDs::Numbers::Pan,
    };

    return leds;
}

enum class ShortCutStates : int
{
    Off,
    Amp,
    Cutoff,
    END_OF_LIST,
};

enum class PlayFavoritesStates : int
{
    Off,
    Play,
    Favorites,
    END_OF_LIST
};

} //end namespace LEDs

struct LEDMessage
{
    SysexMessageType sysexMessageType = SysexMessageType::SetSingleLEDColor;
    bool isOn = false;
    LEDs::Numbers ledNumber = LEDs::Numbers::END_OF_LIST;
    std::uint8_t color[3] = { 0, 0, 0 };
};
