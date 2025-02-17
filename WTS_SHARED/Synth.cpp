/*
  ==============================================================================

    Synth.cpp
    Created: 13 Dec 2023 10:50:15pm
    Author:  Charles Schiermeyer

  ==============================================================================
*/

#include "Synth.h"

std::array<Synth, static_cast<size_t>(Synth::END_OF_LIST)> SynthsList = std::array<Synth, static_cast<size_t>(Synth::END_OF_LIST)>
{{
    Synth::A,
    Synth::B,
    Synth::C,
    Synth::D,
}};
