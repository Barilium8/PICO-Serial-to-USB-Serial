// Controller button and encoder definitions

#pragma once

/** Rotary encoders sorted by top-left to bottom-right by section on the controller face */
enum class ControllerEncoderEnums : unsigned char
{
    MainVolumeEncoder,

    MixerEncoder0,
    MixerEncoder1,
    MixerEncoder2,
    MixerEncoder3,

    MenuEncoder0,
    MenuEncoder1,
    MenuEncoder2,
    MenuEncoder3,
    MenuEncoder4,
    MenuEncoder5,

    ModulatorEncoder0,
    ModulatorEncoder1,
    ModulatorEncoder2,
    ModulatorEncoder3,

    JogWheelEncoder,

    UserEncoder0,
    UserEncoder1,

    END_OF_LIST,
};

/** CV & Expression Pedal inputs from HW controller ADC arrive as MSB/LSB CCs */
enum class ControllerCvExpEnums : unsigned char
{
    CV_1_MSB = static_cast<unsigned char> (ControllerEncoderEnums::END_OF_LIST),
    CV_1_LSB,
    CV_2_MSB,
    CV_2_LSB,
    CV_3_MSB,
    CV_3_LSB,

    CV_4_MSB,
    CV_4_LSB,
    CV_5_MSB,
    CV_5_LSB,
    CV_6_MSB,
    CV_6_LSB,

    Exp_Pedal_1_MSB,
    Exp_Pedal_1_LSB,
    Exp_Pedal_2_MSB,
    Exp_Pedal_2_LSB,

    END_OF_LIST,
};

/** CV & Expression Pedal inputs from HW controller ADC arrive as MSB/LSB CCs */
enum class ControllerGateEnums : unsigned char
{
    //Gate_1  = static_cast<unsigned char> (ControllerButtonEnums::END_OF_LIST),
    Gate_1  = static_cast<unsigned char> (ControllerCvExpEnums::END_OF_LIST),
    Gate_2,
    Gate_3,
    Gate_4,
    Gate_5,
    Gate_6,

    Gate_Pedal,

    Analog_Clock_Tic,

    END_OF_LIST,
};


/** Buttons sorted (generally) but top left to bottom right and by section on the controller face */
enum class ControllerButtonEnums : unsigned char
{
    SoloButtonA,
    SoloButtonB,
    SoloButtonC,
    SoloButtonD,
    MuteButtonA,
    MuteButtonB,
    MuteButtonC,
    MuteButtonD,

    VolMacroPanButton,

    WTSButton,
    SynthAButton,
    SynthBButton,
    SynthCButton,
    SynthDButton,

    InitButtonPress,
    InitButtonRelease,
    HelpButtonPress,
    HelpButtonRelease,
    POVButton,
    GridButton, //++

    MenuButton0,
    MenuButton1,
    MenuButton2,
    MenuButton3,
    MenuButton4,
    MenuButton5,

    VALButton, //++
    LFOButton,
    ENVButton,
    EXPButton,    // moved in 3.6.21 0.0.9
    FilterButton, // moved in 3.6.21 0.0.9
    AmpButton,    // moved in 3.6.21 0.0.9

    ModulatorButton0,
    ModulatorButton1,
    ModulatorButton2,
    ModulatorButton3,

    JogEnterButton,

    UserButton0,
    UserButton1,
    UserButton2,
    UserButton3,
    UserButton4, // added in 3.6.21 0.0.9
    UserButton5, // added in 3.6.21 0.0.9

    MenuUpButton,
    MenuDownButton,

    VoiceSelectButton,
    TerrainSelectButton,
    PathSelectButton,
    FilterSelectButton,
    MixerSelectButton,
    PatchesSelectButton,
    EffectsSelectButton,
    SettingsSelectButton,

    RndButtonPress,
    RndButtonRelease,
    LatchButton,
    PlayButton,
    ShiftButtonPress,
    ShiftButtonRelease,
    FavsButton,
    UndoButtonPress,   //++
    UndoButtonRelease,   //++
    RedoButtonPress,   //++
    RedoButtonRelease,   //++
    LearnButtonPress,   //++
    LearnButtonRelease,   //++

    LoadButtonPress,
    LoadButtonRelease,
    SaveButtonPress,
    SaveButtonRelease,
    CopyButtonPress,
    CopyButtonRelease,
    PasteButtonPress,
    PasteButtonRelease,

    SeqButton,   //++
    ArpButton,   //++
    RestButton,   //++
    TieButton,   //++
    RecButton,   //++
    StopButton,   //++
    PlayPauseButton,   //++

    NotUsed,
    END_OF_LIST,
};
