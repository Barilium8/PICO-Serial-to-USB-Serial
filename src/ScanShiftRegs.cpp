//  Buttons.h - Library for reading shift registers for button inputs
//  Created by Steven Barile, Feb 2, 2019, heavily updated Feb 17, 2022

//  Writing a Library for Arduino...
//  https://www.arduino.cc/en/Hacking/LibraryTutorial

#include <Arduino.h>
#include "ScanShiftRegs.h"
//#include "Log.h"

#include <stdio.h>
#include "hardware/gpio.h"
#include "pico/binary_info.h"
//#include "pico/stdlib.h"

//========================================================

//MARCOs for printing bits (0/1's)
//Ecoder8 #define PRINTB32(Num) for (int i=0; i<32; i++) { if(i<17){ if(i%8==0){Serial.print("    ");} if(i%2==0){Serial.print(" ");} } else{ if(i%4==0){Serial.print(" ");} } Serial.write(((Num >> i) & 1) == 1 ? '1' : '0'); } Serial.println();
#define PRINTB32(Num) for(int i=0; i<32; i++) { if(i%8==0){Serial.print("    ");} if(i%2==0){Serial.print(" ");}  Serial.write(((Num >> i) & 1) == 1 ? '1' : '0'); } Serial.println();
//Orig #define PRINTB32(Num) for (int i=31; i>=0; i--) Serial.write(((Num >> i) & 1) == 1 ? '1' : '0'); } Serial.println();        // Prints a binary number with following Placeholder Zeros  (Automatic Handling)

        ScanShiftRegs::ScanShiftRegs(const uint8_t shiftRegPort, const uint8_t numModules, uint8_t moduleTypeList[]) {
            //Serial.print(F("In ScanShiftRegs::ScanShiftRegs..."));
            _shiftRegPort = shiftRegPort;
            _numModules = numModules;
            if (_numModules > MAX_NUM_MODULES) {
                _numModules = MAX_NUM_MODULES;
                //Serial.print(F("MAX number of modules can NOT be > ")); Serial.print(MAX_NUM_MODULES);
            }
            _moduleTypeList = &moduleTypeList[0];

            butMask = new uint32_t[_numModules];
            prevState = new uint32_t[_numModules];
            heldFcnCalled = new uint32_t[_numModules];

            for(auto i=0; i<_numModules; i++) {
                uint8_t currentModuleConfig = moduleTypeList[i];
                totalNumberButs += theModuleConfig[currentModuleConfig].numButtons;
                totalNumberEncs += theModuleConfig[currentModuleConfig].numEncs;
                //if (theModuleConfig[currentModuleConfig].numButtons == 0) { butMask[i] = 0; }
                //else { butMask[i] = (uint32_t) pow( 2, theModuleConfig[currentModuleConfig].numButtons) << theModuleConfig[currentModuleConfig].butStartBit; }
                butMask[i] = theModuleConfig[currentModuleConfig].buttonMask;
                prevState[i] = 0;
                heldFcnCalled[i] = 0;
            }

            butStartTime = new uint32_t[totalNumberButs];
            prevEnc2BitVal = new uint8_t[totalNumberEncs];
            oneDetent = new uint8_t[totalNumberEncs];

            for(auto i=0; i<totalNumberEncs; i++) {
                butStartTime[i] = 0;
                prevEnc2BitVal[i] = 0;
                oneDetent[i] = 0;
            }

            // pre-fill arrays with 1s
            // for(auto i=0; i<totalNumberEncs; i++) {
            //     thePrevDirs[i][0] = thePrevDirs[i][1] = thePrevDirs[i][2] = thePrevDirs[i][3] = 1;
            // }

            // example code.... (not used)
            // uint8_t* test = new uint8_t[_numModules];
            // for (auto i=0; i<_numModules; i++) {
            //     test[i] = (uint8_t)random(10);
            // }

        } // end fcn Constructor

        //==========================================================

        void ScanShiftRegs::DebugDump() {
            Serial.println();
            Serial.print("Bus ");
            Serial.print(_shiftRegPort);
            Serial.print(" has ");
            Serial.print(_numModules);
            Serial.print(" modules with ");
            Serial.print(totalNumberButs);
            Serial.print(" buts and ");
            Serial.print(totalNumberEncs);
            Serial.println(" encs");

            for(auto currentModuleNum=0; currentModuleNum<_numModules; currentModuleNum++) {
                uint8_t anIndex = _moduleTypeList[currentModuleNum];
                Serial.print(" Module# "); Serial.print(currentModuleNum);
                Serial.print(" type = "); Serial.println(anIndex);
                Serial.print("  num of bits = "); Serial.println(theModuleConfig[anIndex].numBits);
                Serial.print("   # buts = "); Serial.print(theModuleConfig[anIndex].numButtons);
                Serial.print(" on bit:"); Serial.println(theModuleConfig[anIndex].butStartBit);
                Serial.print("   # encs = "); Serial.print(theModuleConfig[anIndex].numEncs);
                Serial.print(" on bit:"); Serial.println(theModuleConfig[anIndex].encStartBit);
                Serial.print("   mask["); Serial.print(currentModuleNum); Serial.print("]="); PRINTB32(butMask[currentModuleNum]);
                Serial.print(F("   prev Bits: ")); PRINTB32(prevState[currentModuleNum]);
                Serial.println();
            }
        }

        //==========================================================

        void ScanShiftRegs::setPins(uint8_t latchPin, uint8_t clockPin, uint8_t dataPin) {
            _latchPin = latchPin;
            _clockPin = clockPin;
            _dataPin = dataPin;

            //pinMode(_latchPin, OUTPUT);
            gpio_set_function(_dataPin, GPIO_FUNC_PWM);
            gpio_set_dir(_latchPin, GPIO_OUT);
            delayMicroseconds(2);
            //digitalWrite(_latchPin, HIGH); // set low, latch on low-to-high transition
            digitalWrite(_latchPin, LOW); // set low, latch on low-to-high transition

            //pinMode(_clockPin, OUTPUT);
            gpio_set_function(_dataPin, GPIO_FUNC_PWM);
            gpio_set_dir(_clockPin, GPIO_OUT);
            delayMicroseconds(2);
            digitalWrite(_clockPin, HIGH); // set low, clock on low-to-high transition

            pinMode(_dataPin, INPUT_PULLUP);
            ////gpio_set_function(_dataPin, GPIO_FUNC_PWM);
            //_gpio_init(_dataPin);
            //gpio_set_dir(_dataPin, GPIO_IN);
            ////gpio_set_pulls(_dataPin, true, false);
            //gpio_pull_up(_dataPin);

            DebugDump();

            // was...
            // theLatchPin.setNewFastPin(_latchPin);
            // theClockPin.setNewFastPin(_clockPin);
            // theDataPin.setNewFastPin(_dataPin);

            // theLatchPin.pinModeFast(OUTPUT);
            // theClockPin.pinModeFast(OUTPUT);
            // theDataPin.pinModeFast(INPUT);

        } // end fcn setPins

        void ScanShiftRegs::setHoldTime(uint16_t holdTime) {
            _holdTime = holdTime;
        } // end fcn SetParams

        uint16_t ScanShiftRegs::getHoldTime() {
            return(_holdTime);
        } // end fcn getCurrentBits

        uint32_t ScanShiftRegs::getCurrentBits() {
            return(shiftRegBits);
        } // end fcn getCurrentBits

        //==========================================================

        void ScanShiftRegs::scanModules() { // pole this fcn to read the shift-reg buttons - every ~50ms from timer or loop fcn is good
            //LOGLINE_SHIFT_REG_DEBUG("In ScanShiftRegs::scanModules");

            uint32_t acumBitVals = 0;
            boolean theBit, thePrevBit, a_Bit, b_Bit;
            uint16_t elpsTime = 0;

            uint8_t currentModuleConfig;
            uint8_t numBitsInCurrentModule;
            uint8_t accumedNumButs = 0, accumedNumEncs = 0;
            uint8_t accumedNumButStartBits = 0;
            uint8_t accumedNumEncStartBits = 0;
            butInfoType butInfo;
            encInfoType encInfo;

            //LOGLINE_SHIFT_REG_DEBUG(); LOGLINE_SHIFT_REG_DEBUG();

            // digitalWrite(_latchPin, LOW);  // latch - clk disabled
            // delayMicroseconds(5);
            digitalWrite(_latchPin, LOW); //active Low latch
            digitalWrite(_latchPin, HIGH); //active Low latch
            delayMicroseconds(5);
            // Read (shift-in) All the modules on a givin srPort
            for(uint8_t currentModuleNum=0; currentModuleNum<_numModules; currentModuleNum++) {  // for each module on the bus...
                currentModuleConfig = _moduleTypeList[currentModuleNum];
                numBitsInCurrentModule = theModuleConfig[currentModuleConfig].numBits;
                // Serial.println(String("---- in Module =") + currentModuleNum + " of " + _numModules +
                //                          " " + theModuleConfig[_moduleTypeList[currentModuleNum]].numBits + "bits ----" +
                //                          theModuleConfig[_moduleTypeList[currentModuleNum]].numButtons);

                //LOG_SHIFT_REG_DEBUG(F(" numBitsInMod=")); LOGLINE_SHIFT_REG_DEBUG(numBitsInCurrentModule);

                if (currentModuleNum > 0) {  // start acummulating at "currentModuleNum-1"
                    uint8_t tempModuleConfig = _moduleTypeList[currentModuleNum-1];
                    accumedNumButs += theModuleConfig[tempModuleConfig].numButtons;
                    accumedNumEncs += theModuleConfig[tempModuleConfig].numEncs;
                    //accumedNumButStartBits += theModuleConfig[currentModuleConfig].butStartBit;
                    //accumedNumEncStartBits += theModuleConfig[currentModuleConfig].encStartBit;
                    // Serial.println(String("  accumedNumEncs=") + accumedNumEncs +
                    //                         " .numEncs=" + theModuleConfig[tempModuleConfig].numEncs +
                    //                         " ModConfig=" + tempModuleConfig +
                    //                         " prevMod#(-1)=" + currentModuleNum);
                }
                else {
                    //Serial.println(String("  accumedNumEncs=0"));
                }

                // read ALL the bits from the "currentModuleNum"-th module
                for(auto currentBitNum = 0; currentBitNum < numBitsInCurrentModule; currentBitNum++) {

                    if (currentBitNum > 31) {
                        Serial.print(F("!!! TOO MANY BITS TO READ IN THIS MODULE !!!")); Serial.print(currentBitNum);
                        Serial.print(F(" numBitsInMod=")); Serial.println(numBitsInCurrentModule);
                        //Serial.println(String("-mod#") + currentModuleNum + "," + theModuleConfig[currentModuleConfig].numBits + "b-");
                        //currentBitNum = 31;
                    }
                    //theBit = !digitalRead(_dataPin);
                    theBit = !gpio_get(_dataPin);
                    //if (theBit) { acumBitVals |= 1UL << currentBitNum; }
                    // digitalWrite(_clockPin, HIGH); // Pulse the Clock (rising edge shifts the next bit).
                    // delayMicroseconds(2);
                    // digitalWrite(_clockPin, LOW);
                    // delayMicroseconds(2);
                    digitalWrite(_clockPin, LOW); // Pulse the Clock (rising edge shifts the next bit).
                    delayMicroseconds(2);
                    digitalWrite(_clockPin, HIGH);
                    delayMicroseconds(2);
                    if (theBit) { acumBitVals |= 1UL << currentBitNum; }
                } // end for

                 // TEST BITs for state changes - test ONLY if button bits are != 0 (a button pressed),
                //        because buttons can be held thus a HIGH "1" in both states requires a held-time test
                uint32_t tempCr32 = (acumBitVals & butMask[currentModuleNum]);
//Serial.print(" B  curr M-Bits: "); PRINTB32(tempCr32);
                uint32_t tempPv32 = (prevState[currentModuleNum] & butMask[currentModuleNum]);
                if ((tempCr32 != 0) || (tempPv32 != 0)) {
                    // Test for Button state
                    // LOGLINE_SHIFT_REG_DEBUG("");
                    // LOG_SHIFT_REG_DEBUG(F("-mod#")); LOG_SHIFT_REG_DEBUG(currentModuleNum);
                    // LOG_SHIFT_REG_DEBUG(F(",")); LOG_SHIFT_REG_DEBUG(theModuleConfig[currentModuleConfig].numBits);
                    // LOG_SHIFT_REG_DEBUG(F(", #Bbits")); LOGLINE_SHIFT_REG_DEBUG(theModuleConfig[currentModuleConfig].numButtons);
                    // Serial.print(F("B  curr M-Bits: ")); PRINTB32(tempCr32); //acumBitVals & butMask[currentModuleNum]);
                    // Serial.print(F("B  prev M-Bits: ")); PRINTB32(tempPv32); //prevState[currentModuleNum] & butMask[currentModuleNum]);
                    // if (currentModuleNum==0) { Serial.print(F("B 0  curr Bits:   ")); PRINTB32(acumBitVals); }
                    // if (currentModuleNum==1) { Serial.print(F("B 1  curr Bits:   ")); PRINTB32(acumBitVals); }
                    // if (currentModuleNum==2) { Serial.print(F("B 2  curr Bits:   ")); PRINTB32(acumBitVals); }
                    // Serial.print(F("B  prev Bits:   ")); PRINTB32(prevState[currentModuleNum]);
                    // Serial.print(F("B  mask Bits:   ")); PRINTB32(butMask[currentModuleNum]);
                    // Serial.print(F("B ~mask Bits: ")); PRINTB32(~butMask[currentModuleNum]);

                    for(uint8_t butNum = theModuleConfig[currentModuleConfig].butStartBit;
                                butNum < (theModuleConfig[currentModuleConfig].numButtons + theModuleConfig[currentModuleConfig].butStartBit);
                                butNum++) {

                        theBit = (acumBitVals >> butNum) & 1;
                        thePrevBit = (prevState[currentModuleNum] >> butNum) & 1;
                        uint8_t butUINum = accumedNumButs + butNum - theModuleConfig[currentModuleConfig].butStartBit; // - accumedNumButStartBits;

                        butInfo.modNum = currentModuleNum;
                        butInfo.modButNum = butNum - theModuleConfig[currentModuleConfig].butStartBit;
                        butInfo.ordButNum = butUINum;
                        butInfo.butKind = A_BUTTON;  //  NOT A VALID SOLUTION !!!!!
                        //LOG_SHIFT_REG_DEBUG(F("-modBut#")); LOG_SHIFT_REG_DEBUG(butInfo.modButNum);
                        //LOG_SHIFT_REG_DEBUG(F(", ordBut#")); LOGLINE_SHIFT_REG_DEBUG(butInfo.ordButNum);

                        // BUTTON HELD
                        if ( (theBit==1) && (thePrevBit==1) ) {
                            if ( ((heldFcnCalled[currentModuleNum] >> butNum) & 1U) == 0 ) {
                                elpsTime = millis() - butStartTime[butUINum]; // subtract stored button press time from current time
                                if (elpsTime > _holdTime) {
                                    heldFcnCalled[currentModuleNum]  |= 1UL << butNum; // set the "held" bit
                                    butInfo.elpsTime = elpsTime;
                                    onButtonHeld_(_shiftRegPort, butInfo);

                                } // end if
                            } // end if Called HELD
                        } // end if BUTTON HELD

                        // BUTTON PRESSED
                        else if ((theBit==1) && (thePrevBit==0)) {
                            butStartTime[butUINum] = millis();
                            butInfo.elpsTime = 0;
                            onButtonPressed_(_shiftRegPort, butInfo);
                        } // end if BUTTON PRESSED

                        // BUTTON RELEASED
                        else if ((theBit==0) && (thePrevBit==1)) {
                            elpsTime = millis() - butStartTime[butUINum]; // subtract stored button press time from current time
                            butInfo.elpsTime = elpsTime;
                            onButtonReleased_(_shiftRegPort, butInfo);
                            heldFcnCalled[currentModuleNum] &= ~(1UL << butNum); // clear the "held" bit
                        } // end if BUTTON RELEASED

                    } // end for button bits
                } // end if no bits set

                // TEST BITs for state changes - test encoder new bits differ from prev bits
                //if (acumBitVals != prevState[currentModuleNum]) {
                //if (((acumBitVals && !butMask[currentModuleNum]) != 0) || ((prevState[currentModuleNum] && !butMask[currentModuleNum]) != 0)) {
                tempCr32 = (acumBitVals & ~butMask[currentModuleNum]);
                tempPv32 = (prevState[currentModuleNum] & ~butMask[currentModuleNum]);
                if ( tempCr32 != tempPv32 ) {
                //if ( (acumBitVals & ~butMask[currentModuleNum]) != (prevState[currentModuleNum] & ~butMask[currentModuleNum]) ) {
                // TEST BITs for state changes - test ONLY if button bits are != 0 (a button pressed),
                //        because encoders can "park" on a HIGH "1"
                // LOGLINE_SHIFT_REG_DEBUG("");
                // LOG_SHIFT_REG_DEBUG(F("- E mod#")); LOG_SHIFT_REG_DEBUG(currentModuleNum);
                // LOG_SHIFT_REG_DEBUG(F(",")); LOG_SHIFT_REG_DEBUG(theModuleConfig[currentModuleConfig].numBits);
                // LOG_SHIFT_REG_DEBUG(F(", #Ebits")); LOGLINE_SHIFT_REG_DEBUG(theModuleConfig[currentModuleConfig].numEncs);
                // Serial.print(F("E  curr M-Bits: ")); PRINTB32(acumBitVals & ~butMask[currentModuleNum]);
                // Serial.print(F("E  prev M-Bits: ")); PRINTB32(prevState[currentModuleNum] & ~butMask[currentModuleNum]);
                //if (currentModuleNum==0) { Serial.print(F("E 0 curr Bits:   ")); PRINTB32(acumBitVals); }
                //if (currentModuleNum==1) { Serial.print(F("E 1 curr Bits:   ")); PRINTB32(acumBitVals); }
                //Serial.print(F("E  prev Bits:   ")); PRINTB32(prevState[currentModuleNum]);
                // Serial.print(F("E  mask Bits: ")); PRINTB32(butMask[currentModuleNum]);
                // Serial.print(F("E ~mask Bits:   ")); PRINTB32(~butMask[currentModuleNum]);

                    // Test for Encoder state
                    uint8_t encNum = 0; // the encoder number on the HW UI per module
                    for(uint8_t encA_BitNum = theModuleConfig[currentModuleConfig].encStartBit;
                        encA_BitNum < ((theModuleConfig[currentModuleConfig].numEncs * 2) + theModuleConfig[currentModuleConfig].encStartBit);
                        encA_BitNum += 2) {

                        b_Bit = (acumBitVals >> encA_BitNum) & 1;
                        a_Bit = (acumBitVals >> (encA_BitNum+1)) & 1;
                        int8_t enc2BitVal;
                        // 1st convert Enc Gray Code to Binary
                        // Enc A & B is gray code, we convert gray to 2-bit counting --> 00=0, 01=1, 11=2, 10=3
                        if      ((a_Bit == 0) & (b_Bit == 0)) enc2BitVal = 0;
                        else if ((a_Bit == 0) & (b_Bit == 1)) enc2BitVal = 1;
                        else if ((a_Bit == 1) & (b_Bit == 1)) enc2BitVal = 2;
                        else  enc2BitVal = 3;  // ((encA_State == 1) & (encB_State == 0))

                        // shiftedAcumBitVals = shiftedAcumBitVals >> 2; // shift less optimization

                        uint8_t encUINum = accumedNumEncs + encNum - theModuleConfig[currentModuleConfig].encStartBit;

                        // determine encoder's spin direction and inc/dec the ENC's virtual val
                        if ((enc2BitVal+1)%4 == prevEnc2BitVal[encUINum]) {
                            encInfo.modNum = currentModuleNum;
                            encInfo.modEncNum = encNum - theModuleConfig[currentModuleConfig].encStartBit;
                            encInfo.ordEncNum = encUINum;
                            encInfo.encRotDir = checkPrevDirs(encUINum, 1);
                            encInfo.encRotVel = encAcceleration(encUINum);
                            if (encInfo.encRotDir != 0) { onEncChanged_(_shiftRegPort, encInfo); }
                        }

                        //else if ((enc2BitVal-1)%4 == prevEnc2BitVal[encUINum]) { // this logically looks good, but the %4 fails when -1, so do this....
                        else if (  (((enc2BitVal-1)==-1) ? 3 : (enc2BitVal-1)%4) == prevEnc2BitVal[encUINum]) {  // this funky
                            encInfo.modNum = currentModuleNum;
                            encInfo.modEncNum = encNum - theModuleConfig[currentModuleConfig].encStartBit;
                            encInfo.ordEncNum = encUINum;
                            encInfo.encRotDir = checkPrevDirs(encUINum, -1);
                            encInfo.encRotVel = encAcceleration(encUINum);
                            if (encInfo.encRotDir != 0) { onEncChanged_(_shiftRegPort, encInfo); }
                        }

                        prevEnc2BitVal[encUINum] = enc2BitVal;
                        encNum++;
                    } // end for Encoder bits
                } // end if enc bits differ

                prevState[currentModuleNum] = acumBitVals;
                acumBitVals = 0;

            } // end for each module on the bus...

            digitalWrite(_latchPin, LOW);  // un-latch

        } // end fcn scanModules

//=========================================================

int8_t ScanShiftRegs::checkPrevDirs(uint8_t encNum, int8_t checkDir) {
    //Serial.print(String("#") + encNum + "  " + checkDir + " > " + thePrevDirs[encNum][0] + " " + thePrevDirs[encNum][1] + " " + thePrevDirs[encNum][2]);

    if ((checkDir == thePrevDirs[encNum][0]) &&
        (checkDir == thePrevDirs[encNum][1]) &&
        (checkDir == thePrevDirs[encNum][2]) ) {
        thePrevDirs[encNum][3] = checkDir; // set new prevailing direction
    }
    else {
        // simple ring buffer shift
        thePrevDirs[encNum][2] = thePrevDirs[encNum][1];
        thePrevDirs[encNum][1] = thePrevDirs[encNum][0];
        thePrevDirs[encNum][0] = checkDir;

        //if (thePrevDirs[encNum][0] == thePrevDirs[encNum][1] == thePrevDirs[encNum][2]) { thePrevDirs[encNum][3] = thePrevDirs[encNum][0]; } // set new prevailing direction
        if ((thePrevDirs[encNum][0] == thePrevDirs[encNum][1]) &&
            (thePrevDirs[encNum][1] == thePrevDirs[encNum][2])) {
            thePrevDirs[encNum][3] = thePrevDirs[encNum][0]; // set new prevailing direction
        }
        else { thePrevDirs[encNum][3] = 0; } // set "no" prevailing direction

    } // end else

    //Serial.println(String("  rtn:#") + thePrevDirs[encNum][3]);
    return thePrevDirs[encNum][3];

}

//=========================================================

uint8_t ScanShiftRegs::encAcceleration(uint8_t encoderNum) {
      uint8_t fcnRtn = 0;
      uint8_t prevHead = 0;

      static uint8_t encAccelHead[NUM_ENCODERS] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0};

      uint8_t eAHead = encAccelHead[encoderNum];

      encAccel[encoderNum][eAHead] = millis();

      // take care of uint wrap around
      if (eAHead > 1) { prevHead = eAHead - 2; }
      else if (eAHead == 1) { prevHead = 8-1; }
      else if (eAHead == 0) { prevHead = 8-2; }

      // looks at difference of current sample and 2 samples ago
      uint32_t delta = encAccel[encoderNum][eAHead] - encAccel[encoderNum][prevHead];

      if (delta < 5) { // <5 ms between encoder pad wipe
        fcnRtn = 4;
        //Serial.println("Accel'ing FAST   rtn '2'");
      }
      else if (delta < 15) { // <15 ms between encoder pad wipe
        fcnRtn = 3;
        //Serial.println("Accel'ing FAST   rtn '2'");
      }
      else if (delta < 28) { // <28 ms between encoder pad wipe
        fcnRtn = 2;
        //Serial.println("Accel'ing MEDIUM rtn '1'");
      }
      else if (delta < 120) { // <120 ms between encoder pad wipe
        fcnRtn = 1;
        //Serial.println("Accel'ing SLOW   rtn '0'");
      }
      else { // >= 120 ms between encoder pad wipe
        fcnRtn = 0;
        //Serial.println("Accel'ing SLOW   rtn '0'");
      }

      //LOGLINE_BUT_ENC_DEBUG(String("delta ms = ") + delta);
      encAccelHead[encoderNum] = (eAHead + 1) % NUM_ENCODER_VEL_SAMPLESS;
      return fcnRtn;

} // end fcn encAcceleration

//========================================================


        // *******************************************
        // ********** Reg'ing CallBacks Fcns *********
        // *******************************************

        // recieve call back function pointer so lib can inform caller that Buttons & Encoders have been touched
        void ScanShiftRegs::setHandle_ButPressed(ButtonPressed_fcnPointer fcnPointer) { onButtonPressed_ = fcnPointer; }
        void ScanShiftRegs::setHandle_ButHeld(ButtonHeld_fcnPointer fcnPointer) { onButtonHeld_ = fcnPointer; }
        void ScanShiftRegs::setHandle_ButReleased(ButtonReleased_fcnPointer fcnPointer) { onButtonReleased_ = fcnPointer; }
        void ScanShiftRegs::setHandle_EncChanged(EncChanged_fcnPointer fcnPointer) { onEncChanged_ = fcnPointer; }