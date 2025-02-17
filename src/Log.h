#ifndef __INC_LOG__
#define __INC_LOG__

#ifdef ARDUINO

#define USE_LOGIC_PROBES_VIA_REMOTE7 0

#define LOG_LEVEL_ERROR  0
#define LOG_LEVEL_WARN   1
#define LOG_LEVEL_INFO   2
#define LOG_LEVEL_DEBUG  3

//*****************************
#define LOGGING_ENABLED 1
//*****************************

#define LOG_LEVEL LOG_LEVEL_DEBUG
#define LOG_STATE_ENABLED 0
#define LOG_MIDI_ENABLED 1
#define LOG_LED_ENABLED 0
#define LOG_BUT_ENC_ENABLED 0
#define LOG_SHIFT_REG_ENABLED 0

#define LOG_STATE_ERROR(x)
#define LOGLINE_STATE_ERROR(x)
#define LOG_STATE_WARN(x)
#define LOGLINE_STATE_WARN(x)
#define LOG_STATE_INFO(x)
#define LOGLINE_STATE_INFO(x)
#define LOG_STATE_DEBUG(x)
#define LOGLINE_STATE_DEBUG(x)

#define LOG_MIDI_ERROR(x)
#define LOGLINE_MIDI_ERROR(x)
#define LOG_MIDI_WARN(x)
#define LOGLINE_MIDI_WARN(x)
#define LOG_MIDI_INFO(x)
#define LOGLINE_MIDI_INFO(x)
#define LOG_MIDI_DEBUG(x)
#define LOGLINE_MIDI_DEBUG(x)

#define LOG_LED_ERROR(x)
#define LOGLINE_LED_ERROR(x)
#define LOG_LED_WARN(x)
#define LOGLINE_LED_WARN(x)
#define LOG_LED_INFO(x)
#define LOGLINE_LED_INFO(x)
#define LOG_LED_DEBUG(x)
#define LOGLINE_LED_DEBUG(x)

#define LOG_BUT_ENC_ERROR(x)
#define LOGLINE_BUT_ENC_ERROR(x)
#define LOG_BUT_ENC_WARN(x)
#define LOGLINE_BUT_ENC_WARN(x)
#define LOG_BUT_ENC_INFO(x)
#define LOGLINE_BUT_ENC_INFO(x)
#define LOG_BUT_ENC_DEBUG(x)
#define LOGLINE_BUT_ENC_DEBUG(x)

#define LOG_SHIFT_REG_ERROR(x)
#define LOGLINE_SHIFT_REG_ERROR(x)
#define LOG_SHIFT_REG_WARN(x)
#define LOGLINE_SHIFT_REG_WARN(x)
#define LOG_SHIFT_REG_INFO(x)
#define LOGLINE_SHIFT_REG_INFO(x)
#define LOG_SHIFT_REG_DEBUG(x)
#define LOGLINE_SHIFT_REG_DEBUG(x)

#if LOGGING_ENABLED && LOG_STATE_ENABLED
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#undef LOG_STATE_DEBUG
#undef LOGLINE_STATE_DEBUG
#define LOG_STATE_DEBUG Serial0_USB.print
#define LOGLINE_STATE_DEBUG Serial0_USB.println
#endif
#if LOG_LEVEL >= LOG_LEVEL_INFO
#undef LOG_STATE_INFO
#undef LOGLINE_STATE_INFO
#define LOG_STATE_INFO Serial0_USB.print
#define LOGLINE_STATE_INFO Serial0_USB.println
#endif
#if LOG_LEVEL >= LOG_LEVEL_WARN
#undef LOG_STATE_WARN
#undef LOGLINE_STATE_WARN
#define LOG_STATE_WARN Serial0_USB.print
#define LOGLINE_STATE_WARN Serial0_USB.println
#endif
#if LOG_LEVEL >= LOG_LEVEL_ERROR
#undef LOG_STATE_ERROR
#undef LOGLINE_STATE_ERROR
#define LOG_STATE_ERROR Serial0_USB.print
#define LOGLINE_STATE_ERROR Serial0_USB.println
#endif
#endif // LOGGING_ENABLED && LOG_STATE_ENABLED

#if LOGGING_ENABLED && LOG_MIDI_ENABLED
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#undef LOG_MIDI_DEBUG
#undef LOGLINE_MIDI_DEBUG
#define LOG_MIDI_DEBUG Serial0_USB.print
#define LOGLINE_MIDI_DEBUG Serial0_USB.println
#endif
#if LOG_LEVEL >= LOG_LEVEL_INFO
#undef LOG_MIDI_INFO
#undef LOGLINE_MIDI_INFO
#define LOG_MIDI_INFO Serial0_USB.print
#define LOGLINE_MIDI_INFO Serial0_USB.println
#endif
#if LOG_LEVEL >= LOG_LEVEL_WARN
#undef LOG_MIDI_WARN
#undef LOGLINE_MIDI_WARN
#define LOG_MIDI_WARN Serial0_USB.print
#define LOGLINE_MIDI_WARN Serial0_USB.println
#endif
#if LOG_LEVEL >= LOG_LEVEL_ERROR
#undef LOG_MIDI_ERROR
#undef LOGLINE_MIDI_ERROR
#define LOG_MIDI_ERROR Serial0_USB.print
#define LOGLINE_MIDI_ERROR Serial0_USB.println
#endif
#endif // LOGGING_ENABLED && LOG_MIDI_ENABLED

#if LOGGING_ENABLED && LOG_LED_ENABLED
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#undef LOG_LED_DEBUG
#undef LOGLINE_LED_DEBUG
#define LOG_LED_DEBUG Serial0_USB.print
#define LOGLINE_LED_DEBUG Serial0_USB.println
#endif
#if LOG_LEVEL >= LOG_LEVEL_INFO
#undef LOG_LED_INFO
#undef LOGLINE_LED_INFO
#define LOG_LED_INFO Serial0_USB.print
#define LOGLINE_LED_INFO Serial0_USB.println
#endif
#if LOG_LEVEL >= LOG_LEVEL_WARN
#undef LOG_LED_WARN
#undef LOGLINE_LED_WARN
#define LOG_LED_WARN Serial0_USB.print
#define LOGLINE_LED_WARN Serial0_USB.println
#endif
#if LOG_LEVEL >= LOG_LEVEL_ERROR
#undef LOG_LED_ERROR
#undef LOGLINE_LED_ERROR
#define LOG_LED_ERROR Serial0_USB.print
#define LOGLINE_LED_ERROR Serial0_USB.println
#endif
#endif // LOGGING_ENABLED && LOG_LED_ENABLED

#if LOGGING_ENABLED && LOG_BUT_ENC_ENABLED
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#undef LOG_BUT_ENC_DEBUG
#undef LOGLINE_BUT_ENC_DEBUG
#define LOG_BUT_ENC_DEBUG Serial0_USB.print
#define LOGLINE_BUT_ENC_DEBUG Serial0_USB.println
#endif
#if LOG_LEVEL >= LOG_LEVEL_INFO
#undef LOG_BUT_ENC_INFO
#undef LOGLINE_BUT_ENC_INFO
#define LOG_BUT_ENC_INFO Serial0_USB.print
#define LOGLINE_BUT_ENC_INFO Serial0_USB.println
#endif
#if LOG_LEVEL >= LOG_LEVEL_WARN
#undef LOG_BUT_ENC_WARN
#undef LOGLINE_BUT_ENC_WARN
#define LOG_BUT_ENC_WARN Serial0_USB.print
#define LOGLINE_BUT_ENC_WARN Serial0_USB.println
#endif
#if LOG_LEVEL >= LOG_LEVEL_ERROR
#undef LOG_BUT_ENC_ERROR
#undef LOGLINE_BUT_ENC_ERROR
#define LOG_BUT_ENC_ERROR Serial0_USB.print
#define LOGLINE_BUT_ENC_ERROR Serial0_USB.println
#endif
#endif // LOGGING_ENABLED && LOG_BUT_ENC_ENABLED

#if LOGGING_ENABLED && LOG_SHIFT_REG_ENABLED
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#undef LOG_SHIFT_REG_DEBUG
#undef LOGLINE_SHIFT_REG_DEBUG
#define LOG_SHIFT_REG_DEBUG Serial0_USB.print
#define LOGLINE_SHIFT_REG_DEBUG Serial0_USB.println
#endif
#if LOG_LEVEL >= LOG_LEVEL_INFO
#undef LOG_SHIFT_REG_INFO
#undef LOGLINE_SHIFT_REG_INFO
#define LOG_SHIFT_REG_INFO Serial0_USB.print
#define LOGLINE_SHIFT_REG_INFO Serial0_USB.println
#endif
#if LOG_LEVEL >= LOG_LEVEL_WARN
#undef LOG_SHIFT_REG_WARN
#undef LOGLINE_SHIFT_REG_WARN
#define LOG_SHIFT_REG_WARN Serial0_USB.print
#define LOGLINE_SHIFT_REG_WARN Serial0_USB.println
#endif
#if LOG_LEVEL >= LOG_LEVEL_ERROR
#undef LOG_SHIFT_REG_ERROR
#undef LOGLINE_SHIFT_REG_ERROR
#define LOG_SHIFT_REG_ERROR Serial0_USB.print
#define LOGLINE_SHIFT_REG_ERROR Serial0_USB.println
#endif
#endif // LOGGING_ENABLED && LOG_SHIFT_REG_ENABLED


#endif // ARDUINO

#endif // __INC_LOG__
