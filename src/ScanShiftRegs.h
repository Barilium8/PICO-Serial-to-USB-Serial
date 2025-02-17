//  ScanShiftRegs.h - Library for reading shift registers for button inputs
//  Created by Steven Barile, Feb 2, 2019, heavily updated Feb 17, 2022

#ifndef _SCAN_SHIFT_REG_h
#define _SCAN_SHIFT_REG_h

//#include <digitalPinFast.h>

#define MAX_NUM_MODULES 16
//#define NUM_BITS_PER_DETENT 4

#define A_BUTTON 0
#define AN_ENCODER 1

#define CL_Button8   0
#define CL_Button12  1
#define CL_Button16  2
#define CL_Encoder4  3
#define CL_Encoder8  4
#define CL_Roller8   5
#define CL_OnBoard8  6

#define CL_WTS_SR1_A 7
#define CL_WTS_SR1_B 8
#define CL_WTS_SR2_A 9
#define CL_WTS_SR2_B 10

#define CL_WTS_V2B_SR1_A 11
#define CL_WTS_V2B_SR1_B 12
#define CL_WTS_V2B_SR2_A 13
#define CL_WTS_V2B_SR2_B 14
#define CL_WTS_V2B_SR2_C 15

#define NUM_MODULE_TYPES 16

struct moduleConfigType {
    uint8_t numBits;
    uint8_t numButtons;
    uint8_t butStartBit;
    uint8_t numEncs;
    uint8_t encStartBit;
    uint32_t buttonMask;
};

struct butInfoType {
    uint8_t modNum;
    uint8_t modButNum;
    uint8_t ordButNum;
    uint8_t butKind;
    uint16_t elpsTime;
};

struct encInfoType {
    uint8_t modNum;
    uint8_t modEncNum;
    uint8_t ordEncNum;
    int8_t encRotDir;
    int8_t encRotVel;
};


// Callback fcn pointer vars
typedef void (*ButtonPressed_fcnPointer)(uint8_t srPortNum, butInfoType butInfo);
typedef void (*ButtonHeld_fcnPointer)(uint8_t srPortNum, butInfoType butInfo);
typedef void (*ButtonReleased_fcnPointer)(uint8_t srPortNum, butInfoType butInfo);
typedef void (*EncChanged_fcnPointer)(uint8_t srPortNum, encInfoType encInfo);


    class ScanShiftRegs {

        moduleConfigType theModuleConfig[NUM_MODULE_TYPES] = {
            // numBits, numButtons, butStartBit, numEncs, encStartBit, buttonMask

            // Nano - bit order for mask
            // {  8,  8, 0, 0, 0, 0xFF000000 },   // CL_Button8
            // { 16, 12, 0, 0, 0, 0xFFFF0000 },   // CL_Button12
            // { 16, 16, 0, 0, 0, 0xFFFF0000 },   // CL_Button16
            // { 16, 8,  8, 4, 0, 0x00FF0000 },   // CL_Encoder4
            // { 24, 8, 16, 8, 0, 0x0000FF00 },   // CL_Encoder8
            // { 16, 0,  0, 8, 0, 0x0000FF00 },   // CL_Roller8

            // Teensy 4.0 - bit order for mask
            {  8,  8, 0, 0, 0, 0x000000FF },   // CL_Button8
            { 16, 12, 0, 0, 0, 0x0000FFFF },   // CL_Button12
            { 16, 16, 0, 0, 0, 0x0000FFFF },   // CL_Button16
            { 16, 8,  8, 4, 0, 0x00FFFF00 },   // CL_Encoder4
            { 24, 8, 16, 8, 0, 0x00FF0000 },   // CL_Encoder8
            { 16, 0,  0, 8, 0, 0x00FF0000 },   // CL_Roller8
            {  8,  8, 0, 0, 0, 0x000000FF },   // CL_OnBoard8

            { 32,   6, 26,  13, 0,  0xFC000000 }, // CL_WTS_SR1_A
            {  8,   8,  0,   0, 0,  0x000000FF }, // CL_WTS_SR1_B
            { 24,  24,  0,   0, 0,  0xFFFFFFFF }, // CL_WTS_SR2_A
            { 24,  24,  0,   0, 0,  0xFFFFFFFF }, // CL_WTS_SR2_B

            { 32,  10, 22,  11, 0,  0xFFC00000 }, // CL_WTS_V2B_SR1_A
            { 16,  16,  0,   0, 0,  0x0000FFFF }, // CL_WTS_V2B_SR1_B
            { 16,   8,  8,   4, 0,  0x0000FF00 }, // CL_WTS_V2B_SR2_A
            { 32,  26,  6,   3, 0,  0xFFFFFFC0 }, // CL_WTS_V2B_SR2_B
            {  8,   8,  0,   0, 0,  0x000000FF }, // CL_WTS_V2B_SR2_C // new for HW controller r3 6.21
        };

        // void checkForButChange(uint8_t j, boolean bitVal);
        // void checkForEncChange(uint8_t j, boolean a_Bit, boolean b_Bit);
        // uint32_t ReadBits(uint8_t modNum);

        // void updateButtonStates();
        // void updateEncoderStates();

        // Vars
        // using NEW in the constructor...
        uint32_t* butMask;
        uint32_t* prevState;
        uint32_t* heldFcnCalled;
        uint32_t* butStartTime;
        uint8_t* prevEnc2BitVal;
        uint8_t* oneDetent;

        uint8_t totalNumberButs = 0;
        uint8_t totalNumberEncs = 0;

        uint8_t _numModules;
        uint8_t* _moduleTypeList;
        // store PINs in Class
        uint8_t _shiftRegPort;
        uint8_t _numInputs;
        uint16_t _holdTime;
        uint8_t _latchPin;
        uint8_t _dataPin;
        uint8_t _clockPin;
        uint8_t _encCount;

        // store 32 bits of the last read button states
        uint32_t shiftRegBits = 0;
        uint32_t prevShiftRegBits = 0;

        // Encode Acceleration[encrNum][ms]
        #define NUM_ENCODERS 13 // can't figure out how to do this dynamically! :-(
        #define NUM_ENCODER_VEL_SAMPLESS 8
        uint32_t encAccel[NUM_ENCODERS][NUM_ENCODER_VEL_SAMPLESS];
        int8_t checkPrevDirs(uint8_t encNum, int8_t checkDir);
        uint8_t encAcceleration(uint8_t encoderNum);

        // used to hold last 3 encoder direction samples - used for smoothing enc rotation
        int8_t thePrevDirs[NUM_ENCODERS][4]; // index 0-2 = last three rotation directions, 3 = return value

        ButtonPressed_fcnPointer onButtonPressed_;
        ButtonHeld_fcnPointer onButtonHeld_;
        ButtonReleased_fcnPointer onButtonReleased_;
        EncChanged_fcnPointer onEncChanged_;

    public:
        ScanShiftRegs(const uint8_t shiftRegID, const uint8_t numModules, uint8_t moduleTypeList[]);
        void DebugDump();
        void setPins(uint8_t latchPin, uint8_t clockPin, uint8_t dataPin);
        void setHoldTime(uint16_t holdTime);
        uint16_t getHoldTime();
        uint32_t getCurrentBits();

        void scanModules(); // pole this fcn to read the shift-reg attached buttons - every ~50ms from timer or loop fcn

        void setHandle_ButPressed(ButtonPressed_fcnPointer fcnPointer);
        void setHandle_ButHeld(ButtonHeld_fcnPointer fcnPointer);
        void setHandle_ButReleased(ButtonReleased_fcnPointer fcnPointer);
        void setHandle_EncChanged(EncChanged_fcnPointer fcnPointer);

    };

#endif