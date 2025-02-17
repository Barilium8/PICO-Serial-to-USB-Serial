/*
  ==============================================================================

    ParameterIndex.h
    Created: 29 Jun 2022 4:08:08pm
    Author:  Charles Schiermeyer, Justin Johnson

  ==============================================================================
*/

#pragma once

#include <array>
#include <vector>
#include "GlobalDefinitions.h"

enum class MidiControllerIndex
{
    Bank_Select = 0,
    Mod_Wheel = 1,
    Breath_Control = 2,

    Foot_Pedal = 4,

    Main_Volume = 7,
    Main_Balance = 8,

    Expression_Pedal = 11,

    SustainPedalOnOff = 64,
    PortamentoOnOff = 65,

    All_Notes_Mute = 123,
};

namespace ParameterIndex
{
enum class Index
{
    // Global parameters
    Main_Volume,
    Poly_Mode,
    Global_Pitch_Tuning_Hz,
    Tempo_BPM,

    // Layer/Synth parameters
    Synth_Volume,
    Synth_Pan,
    Synth_Solo,
    Synth_Mute,

    Macro1_Amount,
    Macro2_Amount,
    Macro3_Amount,
    Macro4_Amount,

    FX_Type1,
    FX_Type2,
    FX_Type3,
    FX_Type4,

    FX_Chorus_Rate,
    FX_Chorus_Delay,
    FX_Chorus_Depth,
    FX_Chorus_Feedback,
    FX_Chorus_Mix,

    FX_Overdrive_Drive,
    FX_Overdrive_Gain,

    FX_Reverb_PreDelay,
    FX_Reverb_RoomSize,
    FX_Reverb_LowCutFreq,
    FX_Reverb_HighCutFreq,
    FX_Reverb_Mix,

    FX_Shimmer_Feedback,
    FX_Shimmer_Semitones,

    FX_Delay_Time,
    FX_Delay_Feedback,
    FX_Delay_LowCutFreq,
    FX_Delay_HighCutFreq,
    FX_Delay_Mix,

    FX_Decimate_Downsample,
    FX_Decimate_BitCrush,
    FX_Decimate_Mix,

    FX_Phaser_Taps,
    FX_Phaser_Rate,
    FX_Phaser_Depth,
    FX_Phaser_Feedback,
    FX_Phaser_Mix,

    // Voice parameters
    Voice_Unison_Pan,
    Voice_Unison_Mode,
    Voice_Unison_Detune,
    Voice_Drift,
    Voice_Spread,
    Voice_Tune_Mode,
    Voice_Coarse_Tune,
    Voice_Fine_Tune,
    Voice_Mono_Mode,
    Voice_Sub0_Volume,
    Voice_Sub1_Volume,
    Voice_Sub2_Volume,
    Voice_Alias_Amount,
    Voice_Engine,
    Voice_Noise_Type,
    Voice_Noise_Mix,
    Voice_Noise_Stereo,
    Synth_ChSelect,
    Synth_LowNote_Range,
    Synth_HighNote_Range,
    Synth_CrossFade_Range,
    Voice_Portamento,
    Voice_Level,

    FilterSlope,
    FilterType,
    FilterCutoff,
    FilterResonance,
    FilterDrive,
    Filter_KB_track,
    FilterModeDiscrete,
    FilterModeMorph,
    FilterVowel,

    TerrainAType,
    TerrainMorph,
    TerrainAPan,
    TerrainAMod1,
    TerrainAMod2,
    TerrainAMod3,
    TerrainBType,
    TerrainBPan,
    TerrainBMod1,
    TerrainBMod2,
    TerrainBMod3,

    PathShape,
    PathPhase,
    PathRotation,
    PathPosX,
    PathPosY,
    PathSize,
    PathXRad,
    PathYRad,
    PathEccentricity,
    PathOscSync,
    PathPosAngle,
    PathPosRadius,
    PathPhaseDistortion,
    PathPhaseDistortionType,

    ControlVoltage_1,
    ControlVoltage_2,

    LFO_Edit_Select,
    LFO_RetrigMode,
    LFO_AttackTime,
    LFO_Amount,
    LFO_Shape,
    LFO_RateIndex,
    LFO_Phase,
    LFO_DC_Offset,
    LFO_Skew,

    ENV_Delay,
    ENV_Attack,
    ENV_Hold,
    ENV_Decay,
    ENV_Sustain,
    ENV_Release,
    ENV_Amount,
    ENV_Attack_Curve,
    ENV_Decay_Curve,
    ENV_Release_Curve,
    ENV_Edit_Select,
    ENV_Speed,

    // Exp mod-matrix
    EXP_Source,
    EXP_Amount,
    EXP_Destination,
    EXP_Polarity,

    Inbound_CC_A,
    Inbound_CC_B,
    Inbound_CC_C,
    Inbound_CC_D,
    Inbound_CC_E,
    Inbound_CC_F,

    Global_LFO_1_Enabled,
    Global_LFO_2_Enabled,
    Global_ENV_1_Enabled,
    Global_ENV_2_Enabled,

    // User interface parameter settings
    EncoderSelect,
    UserEncoderPage,
    MixerEncoderPage,
    MenuRowNumber,
    SelectedModule,
    SelectedSynth,
    SelectedMenuEncoder,
    SelectedUserEncoder,
    LfoRowNumber,
    EnvRowNumber,
    ExpRowNumber,

    // General settings
    RandomProbabilityThreshold,
    Col_Pal,
    SyncAmpEnvReleaseTime,
    NavigationMode,
    ContextViewMode,
    SelectedModulationEncoder,
    MixerEncoderMode,
    SelectedMixerEncoder,
    AmpFilterShortcutMode,
    ArpMode,
    LatchMode,
    PlayFavoritesMode,
    TPIR_POV_View,
    MostRecentButtonEvent,

    NOT_USED_MENU_ITEM,
    END_OF_LIST,
}; //end enum Index

static inline const auto& getInboundCCs()
{
    static constexpr auto inboundCCs = std::array{
        Index::Inbound_CC_A,
        Index::Inbound_CC_B,
        Index::Inbound_CC_C,
        Index::Inbound_CC_D,
        Index::Inbound_CC_E,
        Index::Inbound_CC_F,
    };

    return inboundCCs;
}

static inline std::array<ParameterIndex::Index, static_cast<size_t> (ParameterIndex::Index::END_OF_LIST)>& GetParameterIndexList()
{
    static std::array<ParameterIndex::Index, static_cast<size_t> (ParameterIndex::Index::END_OF_LIST)> list;

    static bool initialized = false;
    if (initialized == false)
    {
        size_t i = 0;
        for (auto& e : list)
        {
            e = static_cast<ParameterIndex::Index> (i);
            ++i;
        }
        initialized = true;
    }

    return list;
}

constexpr auto PathParameterIndexes = std::array{
    Index::PathShape, // Shape must always be the first element because it is not modulatable
    Index::PathPosX, //X_Position,
    Index::PathPosY, //Y_Position,
    Index::PathSize, //Size,
    Index::PathXRad, //X_Radius,
    Index::PathYRad, //Y_Radius,
    Index::PathRotation, //Rotation,
    Index::PathPhase, //Phase,
    Index::PathEccentricity,
    Index::PathPosAngle,
    Index::PathPosRadius,
    Index::PathOscSync,
    Index::PathPhaseDistortion,
    Index::PathPhaseDistortionType,
};

constexpr auto TerrainParameterIndexes = std::array{
    Index::TerrainAType, // AType and BType must always be the first elements because they are not modulatable
    Index::TerrainBType,
    Index::TerrainMorph,

    Index::TerrainAPan,
    Index::TerrainAMod1,
    Index::TerrainAMod2,
    Index::TerrainAMod3,

    Index::TerrainBPan,
    Index::TerrainBMod1,
    Index::TerrainBMod2,
    Index::TerrainBMod3,
};

constexpr auto FXTypeIndexes = std::array{
    Index::FX_Type1,
    Index::FX_Type2,
    Index::FX_Type3,
    Index::FX_Type4,
};

constexpr auto FXParameterIndexes = std::array{
    Index::FX_Type1,
    Index::FX_Type2,
    Index::FX_Type3,
    Index::FX_Type4,
    Index::FX_Chorus_Rate,
    Index::FX_Chorus_Delay,
    Index::FX_Chorus_Depth,
    Index::FX_Chorus_Feedback,
    Index::FX_Chorus_Mix,
    Index::FX_Overdrive_Drive,
    Index::FX_Overdrive_Gain,
    Index::FX_Reverb_PreDelay,
    Index::FX_Reverb_RoomSize,
    Index::FX_Reverb_LowCutFreq,
    Index::FX_Reverb_HighCutFreq,
    Index::FX_Reverb_Mix,
    Index::FX_Shimmer_Feedback,
    Index::FX_Shimmer_Semitones,
    Index::FX_Delay_Time,
    Index::FX_Delay_Feedback,
    Index::FX_Delay_LowCutFreq,
    Index::FX_Delay_HighCutFreq,
    Index::FX_Delay_Mix,
    Index::FX_Decimate_Downsample,
    Index::FX_Decimate_BitCrush,
    Index::FX_Phaser_Taps,
    Index::FX_Phaser_Rate,
    Index::FX_Phaser_Depth,
    Index::FX_Phaser_Feedback,
    Index::FX_Phaser_Mix,
};

static inline const auto& getLFOParameterIndexes()
{
    static constexpr auto LFOParameterIndexes = std::array{
        Index::LFO_Shape, //Shape,
        Index::LFO_RateIndex, //FreqHZ,
        Index::LFO_AttackTime, //Attack,
        Index::LFO_Amount, //Amount,
        Index::LFO_DC_Offset, //DC_Offset,
        Index::LFO_Skew, //Skew,
        Index::LFO_Phase, //Phase_Degrees,
        Index::LFO_RetrigMode, //Retrig_Mode_Enabled,
    };

    return LFOParameterIndexes;
}

static inline const auto& getENVParameterIndexes()
{
    static constexpr auto ENVParameterIndexes = std::array{
        //these are already in the same order as the Params::EnvSegment enum
        Index::ENV_Delay,
        Index::ENV_Attack,
        Index::ENV_Hold,
        Index::ENV_Decay,
        Index::ENV_Sustain,
        Index::ENV_Release,
        Index::ENV_Amount,
        Index::ENV_Speed,
        Index::ENV_Attack_Curve,
        Index::ENV_Decay_Curve,
        Index::ENV_Release_Curve,
    };
    return ENVParameterIndexes;
}

static inline const auto& getEXPParameterIndexes()
{
    static constexpr auto EXPParameterIndexes = std::array{
        Index::EXP_Source,
        Index::EXP_Amount,
        Index::EXP_Destination,
        Index::EXP_Polarity,
    };

    return EXPParameterIndexes;
}

static inline const auto& getGlobalModParameterIndexes()
{
    static constexpr auto gmParameterIndexes = std::array{
        Index::Global_LFO_1_Enabled,
        Index::Global_LFO_2_Enabled,
        Index::Global_ENV_1_Enabled,
        Index::Global_ENV_2_Enabled,
    };

    return gmParameterIndexes;
}

static inline const auto& getLfoParameterIndexesGrid()
{
    static constexpr auto lfoParameterIndexes = std::array{
        std::array{
            Index::LFO_Amount,
            Index::LFO_Shape,
            Index::LFO_RateIndex,
            Index::LFO_DC_Offset },
        std::array{
            Index::LFO_AttackTime,
            Index::LFO_Skew,
            Index::LFO_Phase,
            Index::LFO_RetrigMode,
        }
    };

    return lfoParameterIndexes;
}

static inline const auto& getEnvParameterIndexesGrid()
{
    static constexpr auto envParameterIndexes = std::array{
        std::array{
            Index::ENV_Amount,
            Index::ENV_Attack,
            Index::ENV_Decay,
            Index::ENV_Release,
        },
        std::array{
            Index::ENV_Speed,
            Index::ENV_Attack_Curve,
            Index::ENV_Decay_Curve,
            Index::ENV_Release_Curve,
        },
        std::array{
            Index::ENV_Delay,
            Index::ENV_Hold,
            Index::ENV_Sustain,
            Index::SyncAmpEnvReleaseTime,
        },
    };

    return envParameterIndexes;
}

static inline const auto& getParameterIndexesGrid()
{
    static constexpr auto parameterIndexes = std::array{
        Index::LFO_Amount,
        Index::LFO_DC_Offset,
        Index::ENV_Amount,
        Index::ENV_Sustain,
    };

    return parameterIndexes;
}

static inline const auto& getParameterIndexesNames()
{
    static auto names = std::array{
        "LFO AMT",
        "LFO DC",
        "ENV AMT",
        "ENV SUS",
    };

    return names;
}

static inline const auto& getUserEncoderGrid()
{
    static auto userEncoderParameterIndexes = std::array{
        std::array{ Index::FilterCutoff, Index::FilterResonance },
        std::array{ Index::TerrainAMod1, Index::TerrainAMod2 },
        std::array{ Index::PathRotation, Index::PathEccentricity },
        std::array{ Index::PathPosX, Index::PathPosY },
        std::array{ Index::PathOscSync, Index::PathPhaseDistortion },
        std::array{ Index::Voice_Unison_Mode, Index::Voice_Unison_Detune },
    };

    return userEncoderParameterIndexes;
}

constexpr auto VoiceParameterIndexes = std::array{
    Index::Voice_Coarse_Tune, // Params::OscParam::CoarseTune,
    Index::Voice_Fine_Tune, // Params::OscParam::FineTune,
    Index::Voice_Unison_Mode, // Params::OscParam::UnisonMode,
    Index::Voice_Unison_Detune, // Params::OscParam::Detune,
    Index::Voice_Unison_Pan,
    Index::Voice_Spread, // Params::OscParam::Spread,
    Index::Voice_Mono_Mode, // Params::OscParam::Mono,
    Index::Voice_Sub0_Volume,
    Index::Voice_Sub1_Volume,
    Index::Voice_Sub2_Volume,
    Index::Voice_Engine,
    Index::Voice_Alias_Amount,
    Index::Voice_Noise_Type,
    Index::Voice_Noise_Mix,
    Index::Voice_Noise_Stereo,
    Index::Voice_Level,
    //Even though MidiCH shows up in the Osc Module (row3), we exclude it from this list because
    //each synth's default midi channel is not 1, but is 1,2,3,4, respectively.

    Index::Macro1_Amount,
    Index::Macro2_Amount,
    Index::Macro3_Amount,
    Index::Macro4_Amount,

    Index::SyncAmpEnvReleaseTime,

    // Params::OscParam::END_OF_LIST
    // Index::Osc_Level, //TODO: Params::OscParams::Level
    // Index::Osc_Drift, //TODO: Params::OscParams::Drift
};

constexpr auto MixerParameterIndexes = std::array{
    Index::Synth_Volume,
    Index::Synth_Pan,
    Index::Synth_Solo,
    Index::Synth_Mute,
    Index::Synth_ChSelect,
};

static inline bool isMixer (Index index)
{
    return Utilities::findHelper (MixerParameterIndexes, index);
}

constexpr auto FilterParameterIndexes = std::array{
    Index::FilterModeDiscrete,
    Index::FilterCutoff,
    Index::FilterResonance,
    Index::FilterDrive,
    Index::FilterSlope,
    Index::FilterModeMorph,
    Index::FilterVowel,
    Index::FilterType,
    //Index::Filter_KB_track, //TODO: Params::FilterParam::KB_Track
};

constexpr auto FXSlotParameterIndexes = std::array{
    Index::FX_Type1,
    Index::FX_Type2,
    Index::FX_Type3,
    Index::FX_Type4,
};

constexpr auto MacroParameterIndexes = std::array{
    Index::Macro1_Amount,
    Index::Macro2_Amount,
    Index::Macro3_Amount,
    Index::Macro4_Amount,
};

constexpr auto SynthStateParameterIndexes = std::array{
    Index::MenuRowNumber,
    Index::SelectedModule,
    Index::SelectedMenuEncoder,
    Index::SelectedUserEncoder,
    Index::NavigationMode,
    Index::ContextViewMode,
    Index::SelectedModulationEncoder,
    Index::MixerEncoderMode,
    Index::SelectedMixerEncoder,
    Index::AmpFilterShortcutMode,
    Index::ArpMode,
    Index::LatchMode,
    Index::PlayFavoritesMode,
    Index::TPIR_POV_View,
    Index::MostRecentButtonEvent,
};

static inline bool isMacro (ParameterIndex::Index index)
{
    return Utilities::findHelper (MacroParameterIndexes, index);
}

static inline auto GetSynthEngineParameterIndexes()
{
    static std::vector<ParameterIndex::Index> synthParameterIndexes;
    if (synthParameterIndexes.empty())
    {
        auto append = [&] (const auto& parameterIndexes)
        {
            synthParameterIndexes.insert (synthParameterIndexes.end(), parameterIndexes.begin(), parameterIndexes.end());
        };

        // Modules
        append (VoiceParameterIndexes);
        append (TerrainParameterIndexes);
        append (PathParameterIndexes);
        append (MixerParameterIndexes);
        append (FXParameterIndexes);
        append (FilterParameterIndexes);
        append (SynthStateParameterIndexes);
    }

    return synthParameterIndexes;
}
} // namespace ParameterIndex
