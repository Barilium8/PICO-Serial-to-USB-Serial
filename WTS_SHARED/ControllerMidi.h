//
// Created by Justin Johnson on 17/07/2024.
//

#pragma once

/** Midi initialisation messages for the hardware controller */
enum class ControllerMidiEnums
{
    // CC# that informs that MIDI data has arrived from a different MIDI port
    WTS_SRC_PORT = 84,  // added in 3.6.21 0.0.12

    WTSButtonAppOff = 85,     // added in 3.6.21 0.0.9
    WTSButtonSystemOff = 86,  // added in 3.6.21 0.0.9

    InitAcknowledge = 87,
    InitEnd = 88,
    InitRequest = 89,

    Button = 90,
};
