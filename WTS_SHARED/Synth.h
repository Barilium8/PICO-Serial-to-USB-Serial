/*
  ==============================================================================

    Synth.h
    Created: 13 Dec 2023 10:50:15pm
    Author:  Charles Schiermeyer

  ==============================================================================
*/

#pragma once

#include <array>
#include <cstddef>

enum class Synth : size_t
{
    A,
    B,
    C,
    D,
    END_OF_LIST,
};

extern std::array<Synth, static_cast<size_t>(Synth::END_OF_LIST)> SynthsList;
