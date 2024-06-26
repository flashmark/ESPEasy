#ifdef USES_P110

// #######################################################################################################
// ########################### Plugin 110 VL53L0X I2C Ranging LIDAR      #################################
// #######################################################################################################
// ###################################### stefan@clumsy.ch      ##########################################
// #######################################################################################################

/** Changelog:
 * 2024-04-27 tonhuisman: Read sensor asynchronously to enable (the new default) trigger on changed value
 * 2024-04-26 tonhuisman: Migrate 'Send event when value unchanged' and 'Trigger delta' settings from P113 (at last...)
 *                        Add Direction value, -1 = closer, 0 = unchanged, 1 = further away
 * 2022-06-22 tonhuisman: Remove delay() call from begin(), handle delay via PLUGIN_FIFTY_PER_SECOND
 *                        Reformat source (uncrustify)
 * 2021-04-05 tonhuisman: Removed check for VL53L1X as that is not compatible with this driver (Got its own plugin P113)
 * 2021-02-06 tonhuisman: Refactored to use PluginStruct to enable multiple-instance use with an I2C Multiplexer
 * 2021-01-07 tonhuisman: Moved from PluginPlayground (P133) to main repo (P110), fixed some issues
 */

// needs VL53L0X library from pololu https://github.com/pololu/vl53l0x-arduino

#include "src/PluginStructs/P110_data_struct.h"

#define PLUGIN_110
#define PLUGIN_ID_110         110
#define PLUGIN_NAME_110       "Distance - VL53L0X (200cm)"
#define PLUGIN_VALUENAME1_110 "Distance"
#define PLUGIN_VALUENAME2_110 "Direction"


///////////////////////////
// VL53L0X Command Codes //
///////////////////////////

boolean Plugin_110(uint8_t function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
    {
      Device[++deviceCount].Number           = PLUGIN_ID_110;
      Device[deviceCount].Type               = DEVICE_TYPE_I2C;
      Device[deviceCount].VType              = Sensor_VType::SENSOR_TYPE_SINGLE;
      Device[deviceCount].Ports              = 0;
      Device[deviceCount].PullUpOption       = false;
      Device[deviceCount].InverseLogicOption = false;
      Device[deviceCount].FormulaOption      = true;
      Device[deviceCount].ValueCount         = 2;
      Device[deviceCount].SendDataOption     = true;
      Device[deviceCount].TimerOption        = true;
      Device[deviceCount].TimerOptional      = true;
      Device[deviceCount].GlobalSyncOption   = true;
      Device[deviceCount].PluginStats        = true;
      break;
    }

    case PLUGIN_GET_DEVICENAME:
    {
      string = F(PLUGIN_NAME_110);
      break;
    }

    case PLUGIN_GET_DEVICEVALUENAMES:
    {
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_110));
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[1], PSTR(PLUGIN_VALUENAME2_110));
      break;
    }

    case PLUGIN_I2C_HAS_ADDRESS:
    case PLUGIN_WEBFORM_SHOW_I2C_PARAMS:
    {
      const uint8_t i2cAddressValues[] = { 0x29, 0x30 };

      if (function == PLUGIN_WEBFORM_SHOW_I2C_PARAMS) {
        addFormSelectorI2C(F("i2cAddr"), 2, i2cAddressValues, P110_I2C_ADDRESS);
        #ifndef BUILD_NO_DEBUG
        addFormNote(F("SDO Low=0x29, High=0x30"));
        #endif // ifndef BUILD_NO_DEBUG
      } else {
        success = intArrayContains(2, i2cAddressValues, event->Par1);
      }
      break;
    }

    #if FEATURE_I2C_GET_ADDRESS
    case PLUGIN_I2C_GET_ADDRESS:
    {
      event->Par1 = P110_I2C_ADDRESS;
      success     = true;
      break;
    }
    #endif // if FEATURE_I2C_GET_ADDRESS

    case PLUGIN_WEBFORM_LOAD:
    {
      {
        const __FlashStringHelper *optionsMode2[3] = {
          F("Normal"),
          F("Fast"),
          F("Accurate") };
        const int optionValuesMode2[3] = { 80, 20, 320 };
        addFormSelector(F("Timing"), F("ptiming"), 3, optionsMode2, optionValuesMode2, P110_TIMING);
      }

      {
        const __FlashStringHelper *optionsMode3[2] = {
          F("Normal"),
          F("Long") };
        const int optionValuesMode3[2] = { 0, 1 };
        addFormSelector(F("Range"), F("prange"), 2, optionsMode3, optionValuesMode3, P110_RANGE);
      }
      addFormCheckBox(F("Send event when value unchanged"), F("notchanged"), P110_SEND_ALWAYS == 1);
      addFormNote(F("When checked, 'Trigger delta' setting is ignored!"));

      addFormNumericBox(F("Trigger delta"), F("delta"), P110_DELTA, 0, 100);
      addUnit(F("0-100mm"));
      #ifndef LIMIT_BUILD_SIZE
      addFormNote(F("Minimal change in Distance to trigger an event."));
      #endif // ifndef LIMIT_BUILD_SIZE

      success = true;
      break;
    }

    case PLUGIN_WEBFORM_SAVE:
    {
      P110_I2C_ADDRESS = getFormItemInt(F("i2cAddr"));
      P110_TIMING      = getFormItemInt(F("ptiming"));
      P110_RANGE       = getFormItemInt(F("prange"));
      P110_SEND_ALWAYS = isFormItemChecked(F("notchanged")) ? 1 : 0;
      P110_DELTA       = getFormItemInt(F("delta"));

      success = true;
      break;
    }

    case PLUGIN_INIT:
    {
      initPluginTaskData(event->TaskIndex, new (std::nothrow) P110_data_struct(P110_I2C_ADDRESS, P110_TIMING, P110_RANGE == 1));
      P110_data_struct *P110_data = static_cast<P110_data_struct *>(getPluginTaskData(event->TaskIndex));

      success = (nullptr != P110_data) && P110_data->begin(); // Start the sensor
      break;
    }
    case PLUGIN_READ:
    {
      P110_data_struct *P110_data = static_cast<P110_data_struct *>(getPluginTaskData(event->TaskIndex));

      if (nullptr != P110_data) {
        const uint16_t dist      = P110_data->getDistance();
        const uint16_t p_dist    = UserVar.getFloat(event->TaskIndex, 0);
        const int16_t  direct    = dist == p_dist ? 0 : (dist < p_dist ? -1 : 1);
        const bool     triggered = (dist > (p_dist + P110_DELTA)) || (dist < (p_dist - P110_DELTA));

        #ifdef P110_INFO_LOG

        if (loglevelActiveFor(LOG_LEVEL_INFO)) {
          addLog(LOG_LEVEL_INFO, strformat(F("VL53L0x: Perform read: trig: %d, prev: %d, dist: %d"), triggered, p_dist, dist));
        }
        #endif // ifdef P110_INFO_LOG

        if (triggered || (P110_SEND_ALWAYS == 1)) {
          UserVar.setFloat(event->TaskIndex, 0, dist); // Value is classified as invalid when > 8190, so no conversion or 'split' needed
          UserVar.setFloat(event->TaskIndex, 1, direct);
          success = true;
        }
      }
      break;
    }

    case PLUGIN_TEN_PER_SECOND: // Handle sensor reading
    {
      P110_data_struct *P110_data = static_cast<P110_data_struct *>(getPluginTaskData(event->TaskIndex));

      if (nullptr != P110_data) {
        P110_data->readDistance();

        if (P110_data->isReadSuccessful() && (Settings.TaskDeviceTimer[event->TaskIndex] == 0)) { // Trigger as soon as there's a valid
                                                                                                  // measurement and 0 interval is set
          Scheduler.schedule_task_device_timer(event->TaskIndex, millis() + 10);
        }
      }
      break;
    }

    case PLUGIN_FIFTY_PER_SECOND: // Handle startup delay
    {
      P110_data_struct *P110_data = static_cast<P110_data_struct *>(getPluginTaskData(event->TaskIndex));

      if (nullptr != P110_data) {
        success = P110_data->plugin_fifty_per_second();
      }
      break;
    }
  }
  return success;
}

#endif // ifdef USES_P110
