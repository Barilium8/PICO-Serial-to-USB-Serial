bool isThisCCaNRPN(uint8_t CC) {
   if ( ((CC >= 96) && (CC <= 99)) || (CC == 6) || (CC == 38) ) {
      //LOGLINE_MIDI_DEBUG("   is a NRPN");
      return true;
   }
   return false;
} // end fcn


// bool isASelectableSilder(uint8_t CC) {  // in the selectable slider list
//     for (auto i=0; i<NUM_MODABLE_SLIDERS; ++i) {
//       if (modKnobValSysExtoCC_LUT[i].theCC == CC) {
//         //LOGLINE_MIDI_DEBUG("   is a selectable slider");
//         return true;
//       }
//     }
//     return false;
// } // end fcn
