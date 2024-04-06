#include "_Plugin_Helper.h"
#ifdef USES_P254

// #######################################################################################################
// #################################### Plugin-254: Ouput heating pump controller ########################
// #######################################################################################################
// P254 "Output - Heating pump"
// Plugin for my own floor heating pump control algorithm
// Control is typically in a home automation controller like Domoticz or HASS. I don't want to depend on
// connectivity and implement it locally. The algorithm is easier implemented as plugin instead of using
// ESPEasy rules. This plugin is not intended to move public domain as the use case is pretty limited
//
// #######################################################################################################

/** Changelog:
 * 2024-02-24 flashmark:  Start plugin development.
 * (Newest changes on top)
 **/

// #include section

// Standard ESPEasy plugin definitions
#define PLUGIN_254
#define PLUGIN_ID_254     254                     // plugin id NOTE: seems to be limeted to unsigned int
#define PLUGIN_NAME_254   "Regulator - Heating pump" // "Plugin Name" is what will be dislpayed in the selection list
#define PLUGIN_VALUENAME1_254 "Output"            // The state of the pump controller output: off/on
#define PLUGIN_254_DEBUG  true                    // set to true for extra log info in the debug

//   PIN/port configuration is stored in the following:
//   CONFIG_PIN1 - Relay output, switching pump on/off
//   CONFIG_PIN2 - Indicator LED red color
//   CONFIG_PIN3 - Indicator LED green color
//   CONFIG_PORT - Indicator LED blue color (abuse port indicator as 4th GPIO indicator)
#define P254_GPIO_RELAY     CONFIG_PIN1
#define P254_GPIO_LED_RED   CONFIG_PIN2
#define P254_GPIO_LED_GREEN CONFIG_PIN3
#define P254_GPIO_LED_BLUE  CONFIG_PORT

//   Custom configuration is stored in the following:
//   PCONFIG(x)
//   x can be between 1 - 8 and can store values between -32767 - 32768 (16 bit)
//   N.B. these are aliases for a longer less readable amount of code. See _Plugin_Helper.h
//
//   Each parameter gets a unique ID (GUID) to recognize it on the GUI (HTML page)
//   The GUID shall be short to reduce memory load. Make meaning full for GUI debugging only.
//
// Input tempetarure is read from another task (plugin) by accessing TASK and VALUE within that task
#define P254_TEMP_TASK          PCONFIG(0)
#define P254_GUID_TEMP_TASK     "f0"
#define P254_TEMP_VAL           PCONFIG(1)
#define P254_GUID_TEMP_VAL      "f1"

// Temperature to switch on the pump [deg C]
#define P254_TEMP_LEVEL         PCONFIG_FLOAT(0)
#define P254_GUID_TEMP_LEVEL    "f2"

// Hysteresis on temperature for switching pump off [deg C]
#define P254_TEMP_HYST          PCONFIG_FLOAT(1)
#define P254_GUID_TEMP_HYST     "f3"

// Minimum time pump should run [min]
#define P254_MIN_TIME           PCONFIG(2)
#define P254_GUID_MIN_TIME      "f4"

// Maximum time pump should be idle [hour]
#define P254_INTERVAL_TIME      PCONFIG(3)
#define P254_GUID_INTERVAL_TIME "f5"

// Time for forced circulation [min]
#define P254_FORCE_TIME         PCONFIG(4)
#define P254_GUID_FORCE_TIME    "f6"

// Operation mode [enum]
#define P254_OPMODE             PCONFIG(5)
#define P254_GUID_OPMODE        "f7"

// Timestamp for last control action. Must be stored persistant
#define P254_TIMESTAMP          PCONFIG_ULONG(0)

// Not intended for manipulation through GUI
// Control state as calculated by the control algorithm
#define P254_CONTROL_STATE      PCONFIG(6)

// Not intended for manipulation through GUI
// Control state as calculated by the control algorithm
#define P254_REMOTE_STATE       PCONFIG(7)

// Set of individual bits packed in a single PCONFIG
// Use P254_getConfigBit() and P254_setConfigBit() to access the individual bits
#define P254_FLAGS              PCONFIG(8)
#define P254_INV_OUTPUT         0
#define P254_GUID_INV_OUTPUT    "b0"
#define P254_INV_LED_RED        1
#define P254_GUID_INV_LED_RED   "b1"
#define P254_INV_LED_GREEN      2
#define P254_GUID_INV_LED_GREEN "b2"
#define P254_INV_LED_BLUE       3
#define P254_GUID_INV_LED_BLUE  "b3"

// #define P254_OUTPUT_TYPE_INDEX  2

#define millis2seconds(x) ((x) / ((ulong)1000))
#define millis2minutes(x) ((x) / ((ulong)1000 * 60))
#define millis2hours(x)   ((x) / ((ulong)1000 * 60 * 24))

// Operation modes for the control algorithm
enum P254_opmode
{
  P254_OPMODE_OFF,     // Pump is fully shut down, no forced curculation
  P254_OPMODE_STANDBY, // Pump is only switched on for forced maintenance runs
  P254_OPMODE_ON,      // Pump is always switched on
  P254_OPMODE_TEMP,    // Control algorithm based on temperature only
  P254_OPMODE_REMOTE   // Both temperature and remote command can switch on pump
};
#define P254_OPMODE_SIZE  5

// Control state for the control algorithm
enum P254_control_state
{
  P254_STATE_IDLE,    // Pump is not running
  P254_STATE_HEATING, // Pump is required for heating
  P254_STATE_EXTEND,  // Pump is running due to minimum duration
  P254_STATE_FORCE    // Pump is forced on due to maximum interval time exceeded
};

// Prototype for the main evaluation of teh control algorithm
void P254_evaluate(struct EventStruct *event);

boolean P254_getConfigBit(uint16_t config, int pos)
{
  boolean res = ((config >> pos) & 0x0001) == 1;
  return res;
}

uint16_t P254_setConfigBit(uint16_t config, int pos, boolean val)
{
  uint16_t mask = (0x0001 << pos);
  if (val) 
  {
    config |= mask;
  }
  else
  {
    mask ^= 0xFFFF;
    config &= mask;
  }
  return config;
}

// A plugin has to implement the following function
// ------------------------------------------------
boolean Plugin_254(uint8_t function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
    {
      // This case defines the device characteristics, edit appropriately

      Device[++deviceCount].Number           = PLUGIN_ID_254;                    // Plugin ID number.   (PLUGIN_ID_xxx)
      Device[deviceCount].Type               = DEVICE_TYPE_CUSTOM0;              // How the device is connected. e.g. DEVICE_TYPE_SINGLE =>
                                                                                 // connected through 1 datapin
      Device[deviceCount].VType              = Sensor_VType::SENSOR_TYPE_SWITCH; // Type of value the plugin will return. e.g.
                                                                                 // SENSOR_TYPE_STRING
      Device[deviceCount].Ports              = 0;                                // Port to use when device has multiple I/O pins  (N.B. not
                                                                                 // used much)
      Device[deviceCount].ValueCount         = 1;                                // The number of output values of a plugin. The value
                                                                                 // should match the number of keys PLUGIN_VALUENAME1_xxx
      Device[deviceCount].OutputDataType     = Output_Data_type_t::Default;      // Subset of selectable output data types  (Default = no
                                                                                 // selection)
      Device[deviceCount].PullUpOption       = false;                            // Allow to set internal pull-up resistors.
      Device[deviceCount].InverseLogicOption = false;                            // Allow to invert the boolean state (e.g. a switch)
      Device[deviceCount].FormulaOption      = false;                            // Allow to enter a formula to convert values during read.
                                                                                 // (not possible with Custom enabled)
      Device[deviceCount].Custom             = false;
      Device[deviceCount].SendDataOption     = true;                             // Allow to send data to a controller.
      Device[deviceCount].GlobalSyncOption   = true;                             // No longer used. Was used for ESPeasy values sync between
                                                                                 // nodes
      Device[deviceCount].TimerOption        = true;                             // Allow to set the "Interval" timer for the plugin.
      Device[deviceCount].TimerOptional      = false;                            // When taskdevice timer is not set and not optional, use
                                                                                 // default "Interval" delay (Settings.Delay)
      Device[deviceCount].DecimalsOnly       = false;                            // Allow to set the number of decimals (otherwise treated a
                                                                                 // 0 decimals)
      Device[deviceCount].SendDataOption     = true;
      Device[deviceCount].PluginStats        = true;
      break;
    }

    case PLUGIN_GET_DEVICENAME:
    {
      // return the device name
      string = F(PLUGIN_NAME_254);
      break;
    }

    case PLUGIN_GET_DEVICEVALUENAMES:
    {
      // called when the user opens the module configuration page
      // it allows to add a new row for each output variable of the plugin
      // For plugins able to choose output types, see P026_Sysinfo.ino.
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_254));
      break;
    }

    case PLUGIN_WEBFORM_SHOW_GPIO_DESCR:
    {
      const __FlashStringHelper *labels[] = {
        F("RELAY"), F("RED"), F("GREEN"), F("BLUE")
      };
      int values[] = {
        P254_GPIO_RELAY,
        P254_GPIO_LED_RED,
        P254_GPIO_LED_GREEN,
        P254_GPIO_LED_BLUE
      };
      constexpr size_t nrElements = NR_ELEMENTS(values);

      for (size_t i = 0; i < nrElements; ++i) {
        if (i != 0) { addHtml(event->String1); }
        addHtml(labels[i]);
        addHtml(F(":&nbsp;"));
        addHtml(formatGpioLabel(values[i], true));
      }
      success = true;
      break;
    }

    case PLUGIN_GET_DEVICEVALUECOUNT:
    {
      // This is only called when Device[deviceCount].OutputDataType is not Output_Data_type_t::Default
      // The position in the config parameters used in this example is PCONFIG(Pxxx_OUTPUT_TYPE_INDEX)
      // Must match the one used in case PLUGIN_GET_DEVICEVTYPE  (best to use a define for it)
      // see P026_Sysinfo.ino for more examples.
      // event->Par1 = getValueCountFromSensorType(static_cast<Sensor_VType>(PCONFIG(P254_OUTPUT_TYPE_INDEX)));
      // success     = true;
      break;
    }

    case PLUGIN_GET_DEVICEVTYPE:
    {
      // This is only called when Device[deviceCount].OutputDataType is not Output_Data_type_t::Default
      // The position in the config parameters used in this example is PCONFIG(Pxxx_OUTPUT_TYPE_INDEX)
      // Must match the one used in case PLUGIN_GET_DEVICEVALUECOUNT  (best to use a define for it)
      // IDX is used here to mark the PCONFIG position used to store the Device VType.
      // see _P026_Sysinfo.ino for more examples.
      // event->idx        = P254_OUTPUT_TYPE_INDEX;
      // event->sensorType = static_cast<Sensor_VType>(PCONFIG(event->idx));
      // success           = true;
      break;
    }

    case PLUGIN_SET_DEFAULTS:
    {
      // Set a default config here, which will be called when a plugin is assigned to a task.

      P254_GPIO_RELAY     = -1;                   // GPIO for relay output not assigned
      P254_GPIO_LED_RED   = -1;                   // GPIO for red LED output not assigned
      P254_GPIO_LED_GREEN = -1;                   // GPIO for green LED output not assigned
      P254_GPIO_LED_BLUE  = -1;                   // GPIO for blue LED output not assigned
      P254_TEMP_TASK      = -1;                   // No temperature source assigned
      P254_TEMP_VAL       = -1;                   // No temperature source assigned
      P254_TEMP_LEVEL     = 25.0f;                // Switch pump on above 25 [deg C]
      P254_TEMP_HYST      = 5.0f;                 // Switch off hysteresis 5 [deg C] below switch on value
      P254_MIN_TIME       = 30;                   // Once switched on pump should run at least 30 [min]
      P254_INTERVAL_TIME  = 24;                   // Pump shall run after 24 [hour] stand still
      P254_FORCE_TIME     = 5;                    // Forced circulation for 5 [min]
      P254_OPMODE         = P254_OPMODE_OFF;      // Don't touch unless selected by operator
      P254_CONTROL_STATE  = (int)P254_STATE_IDLE; // Initalize control algorithm to IDLE
      P254_REMOTE_STATE   = 0;                    // Remote control is OFF
      P254_TIMESTAMP      = millis();             // Init time bookkeeping
    }

    case PLUGIN_WEBFORM_LOAD:
    {
       #ifdef PLUGIN_254_DEBUG
      {
        // Add some debug information
        String msg = F("Debugging data: State= ");
        msg += P254_printControlState(P254_CONTROL_STATE);
        msg += (F(", Pump= "));
        msg += (P254_CONTROL_STATE) == P254_STATE_IDLE ? 0 : 1;
        msg += F(", Remote= ");
        msg += P254_REMOTE_STATE;
        msg += F(", Timer= ");
        msg += (int)(millis2seconds(timePassedSince(P254_TIMESTAMP)));
        addFormNote(msg);
      }
      #endif

      // We load/save the TaskDevicePins ourselves.
      addFormPinSelect(PinSelectPurpose::Generic_output,
                       formatGpioName_output(F("RELAY")),
                       F("taskdevicepin1"),
                       P254_GPIO_RELAY);
      if (validGpio(P254_GPIO_RELAY))
      {
        addFormCheckBox(F("Invert Output"),
                      F(P254_GUID_INV_OUTPUT),
                      P254_getConfigBit(P254_FLAGS, P254_INV_OUTPUT));
      }
      addFormPinSelect(PinSelectPurpose::Generic_output,
                       formatGpioName_output(F("LED RED")),
                       F("taskdevicepin2"),
                       P254_GPIO_LED_RED);
      if (validGpio(P254_GPIO_LED_RED))
      {
        addFormCheckBox(F("Invert LED Red"),
                      F(P254_GUID_INV_LED_RED),
                      P254_getConfigBit(P254_FLAGS, P254_INV_LED_RED));
      }
      addFormPinSelect(PinSelectPurpose::Generic_output,
                       formatGpioName_output(F("LED GREEN")),
                       F("taskdevicepin3"),
                       P254_GPIO_LED_GREEN);
      if (validGpio(P254_GPIO_LED_GREEN))
      {
        addFormCheckBox(F("Invert LED Green"),
                      F(P254_GUID_INV_LED_GREEN),
                      P254_getConfigBit(P254_FLAGS, P254_INV_LED_GREEN));
      }
      addFormPinSelect(PinSelectPurpose::Generic_output,
                       formatGpioName_output(F("LED BLUE")),
                       F("taskdevicepin4"),
                       P254_GPIO_LED_BLUE);
      if (validGpio(P254_GPIO_LED_BLUE))
      {
        addFormCheckBox(F("Invert LED Blue"),
                      F(P254_GUID_INV_LED_BLUE),
                      P254_getConfigBit(P254_FLAGS, P254_INV_LED_BLUE));
      }

      //  Select task and value for input
      addRowLabel(F("Input Task"));
      addTaskSelect(F(P254_GUID_TEMP_TASK), P254_TEMP_TASK);

      if (validTaskIndex(P254_TEMP_TASK))
      {
        LoadTaskSettings(P254_TEMP_TASK); // we need to load the values from another task for selection!
        addRowLabel(F("Input Value"));
        addTaskValueSelect(F(P254_GUID_TEMP_VAL), P254_TEMP_VAL, P254_TEMP_TASK);
      }

      // FormSelector with all operation mode options
      String options[P254_OPMODE_SIZE] = {F("Off"), F("Standby"), F("On"), F("Control"), F("Remote")};
      int optionValues[P254_OPMODE_SIZE] = {P254_OPMODE_OFF, P254_OPMODE_STANDBY, P254_OPMODE_ON, P254_OPMODE_TEMP, P254_OPMODE_REMOTE};
      addFormSelector(F("Control mode"), F(P254_GUID_OPMODE), P254_OPMODE_SIZE, options, optionValues, P254_OPMODE);

      // Temperature control switch on level [deg C]
      addFormFloatNumberBox(F("Switch on temperature"), F(P254_GUID_TEMP_LEVEL), P254_TEMP_LEVEL, 0.0f, 10e6f, 2U);
      addUnit(F("deg C"));

      // Temperature control hysteresis [deg C]
      addFormFloatNumberBox(F("Switch hysteresis temperature"), F(P254_GUID_TEMP_HYST), P254_TEMP_HYST, 0.0f, 10e6f, 2U);
      addUnit(F("deg C"));

      // Minimum on time [min]
      addFormNumericBox(F("Minimum running time"), F(P254_GUID_MIN_TIME), P254_MIN_TIME, 0);
      addUnit(F("minute"));

      // Interval time (max idle time) [hour]
      addFormNumericBox(F("Maximum idle time"), F(P254_GUID_INTERVAL_TIME), P254_INTERVAL_TIME, 0);
      addUnit(F("hour"));

      // Interval circulation time (forced circulation) [min]
      addFormNumericBox(F("Forced circulation time"), F(P254_GUID_FORCE_TIME),    P254_FORCE_TIME,    1);
      addUnit(F("minute"));
      success = true;
      break;
    }

    case PLUGIN_WEBFORM_SAVE:
    {
      // this case defines the code to be executed when the form is submitted
      // the plugin settings should be saved to PCONFIG(x)
      // PCONFIG(0) = getFormItemInt(F("dsptype"));
      // pin configuration will be read from CONFIG_PIN1 and stored
      // If custom tasksettings need to be stored, then here is the place to add that

      // after the form has been saved successfuly, set success and break
      P254_GPIO_RELAY     = getFormItemInt(F("taskdevicepin1"));
      P254_GPIO_LED_RED   = getFormItemInt(F("taskdevicepin2"));
      P254_GPIO_LED_GREEN = getFormItemInt(F("taskdevicepin3"));
      P254_GPIO_LED_BLUE  = getFormItemInt(F("taskdevicepin4"));

      P254_TEMP_TASK = getFormItemInt(F(P254_GUID_TEMP_TASK));
      P254_TEMP_VAL  = getFormItemInt(F(P254_GUID_TEMP_VAL));

      P254_TEMP_LEVEL    = getFormItemFloat(F(P254_GUID_TEMP_LEVEL));
      P254_TEMP_HYST     = getFormItemFloat(F(P254_GUID_TEMP_HYST));
      P254_MIN_TIME      = getFormItemInt(F(P254_GUID_MIN_TIME));
      P254_INTERVAL_TIME = getFormItemInt(F(P254_GUID_INTERVAL_TIME));
      P254_FORCE_TIME    = getFormItemInt(F(P254_GUID_FORCE_TIME));
      P254_OPMODE        = getFormItemInt(F(P254_GUID_OPMODE));
      
      P254_FLAGS = P254_setConfigBit(P254_FLAGS, P254_INV_OUTPUT,    isFormItemChecked(F(P254_GUID_INV_OUTPUT)));
      P254_FLAGS = P254_setConfigBit(P254_FLAGS, P254_INV_LED_RED,   isFormItemChecked(F(P254_GUID_INV_LED_RED)));
      P254_FLAGS = P254_setConfigBit(P254_FLAGS, P254_INV_LED_GREEN, isFormItemChecked(F(P254_GUID_INV_LED_GREEN)));
      P254_FLAGS = P254_setConfigBit(P254_FLAGS, P254_INV_LED_BLUE,  isFormItemChecked(F(P254_GUID_INV_LED_BLUE)));

      success = true;
      break;
    }
    case PLUGIN_INIT:
    {
      // this case defines code to be executed when the plugin is initialised

      // after the plugin has been initialised successfuly, set success and break
      success = true;
      break;
    }

    case PLUGIN_READ:
    {
      // code to be executed to read data
      // It is executed according to the delay configured on the device configuration page, only once

      P254_evaluate(event);   // Get a fresh control state

      // We return the status of the pump: switched on (1) or switched off (0)
      // This is calculated from the control state as stored in P254_CONTROL_STATE
      UserVar.setFloat(event->TaskIndex, 0, (float)(((P254_control_state)P254_CONTROL_STATE == P254_STATE_IDLE) ? 0 : 1));
      success = true;
      break;
    }

    case PLUGIN_WRITE:
    {
      // this case defines code to be executed when the plugin executes an action (command).
      // Commands can be accessed via rules or via http.
      // As an example, http://192.168.1.12//control?cmd=dothis
      // implies that there exists the comamnd "dothis"

      //if (false)    // plugin_not_initialised
      //{
      //  break;
      //}

      // parse string to extract the command
      String command = parseString(string, 1); // already converted to lowercase
      String log = F("P254 write: ");
      if (equals(command, F("pumpcontrol"))) {
        String subcmd = parseString(string, 2);
        if (equals(subcmd, F("remote"))) {
          log += F(" pumpcontrol");
          String value = parseString(string, 3);
          if (equals(value, F("on")))
          {
            log += F(" on");
            P254_REMOTE_STATE = 1;
            success = true;
          }
          else if (equals(value, F("off")))
          {
            log += F(" off");
            P254_REMOTE_STATE = 0;
            success = true;
          }
          else
          {
            log += F(" **no value**");
          }
        }
        else 
        {
          log += F(" **no command**");
        }
      }
      addLogMove(LOG_LEVEL_DEBUG, log);
      break;
    }

    case PLUGIN_EXIT:
    {
      // perform cleanup tasks here. For example, free memory, shut down/clear a display

      break;
    }

    case PLUGIN_ONCE_A_SECOND:
    {
      // code to be executed once a second. Tasks which do not require fast response can be added here
      P254_evaluate(event);
      success = true;
    }

    case PLUGIN_TEN_PER_SECOND:
    {
      // code to be executed 10 times per second. Tasks which require fast response can be added here
      // be careful on what is added here. Heavy processing will result in slowing the module down!

      success = true;
    }
  } // switch
  return success;
}   // function

#ifdef PLUGIN_254_DEBUG
String P254_printControlState(int state)
{
  switch (state)
  {
    case P254_STATE_IDLE:     return F("Idle"); break;
    case P254_STATE_HEATING:  return F("Heating"); break;
    case P254_STATE_EXTEND:   return F("Extend"); break;
    case P254_STATE_FORCE:    return F("Force"); break;
    default:                  return F("***ERROR***"); break;
  }
}
#endif // ifdef PLUGIN_254_DEBUG

// Execute the control algorithm
// Evaluation shall not be dependent on when or how often this function is called
// All stateful infromation is stored in the plugin context environment accessible through variable event
void P254_evaluate(struct EventStruct *event)
{
  float temperature                    = 0.0f; // Control temperature as read from the sensor task [deg C]
  P254_control_state old_control_state = (P254_control_state)P254_CONTROL_STATE;
  P254_control_state new_control_state = old_control_state;
  ulong timestamp                      = P254_TIMESTAMP;
  bool  remote_state                   = (P254_REMOTE_STATE != 1); // Control input state from remote controller
  bool  relay_output                   = false; // Calculated new relay output state
  bool  led_red_output                 = false; // Calculated new red LED output state
  bool  led_green_output               = false; // Calculated new green LED output state
  bool  led_blue_output                = false; // Calculated new blue LED output state

  // Get the control temperature from the external task. If task does not exist use the default
  if (validTaskIndex(P254_TEMP_TASK))
  {
    temperature = UserVar.getFloat(P254_TEMP_TASK, P254_TEMP_VAL); // [deg C]
  }

  // Calculate the new control state based upon the selected operation mode
  // This also evaluates the gathered input values
  switch ((P254_opmode)(P254_OPMODE))
  {
    // Control is switched off completely
    case P254_OPMODE_OFF:
      if (old_control_state != P254_STATE_IDLE)
      {
        timestamp         = millis();
        new_control_state = P254_STATE_IDLE;
      }
      break;

    // Control is always on
    case P254_OPMODE_ON:
      if (old_control_state == P254_STATE_IDLE)
      {
        timestamp         = millis();
        new_control_state = P254_STATE_HEATING;
      }
      else
      {
        // Keep timestamp from moment pump was swiched on
        new_control_state = P254_STATE_HEATING;
      }
      break;

    // Control is switched off with maintnenance interval enabled
    case P254_OPMODE_STANDBY:
      if (old_control_state == P254_STATE_IDLE)   // Pump was idling
      {
        if (millis2hours(timePassedSince(timestamp)) >= P254_INTERVAL_TIME)
        {
          timestamp         = millis();
          new_control_state = P254_STATE_FORCE;
        }
      }
      else if (millis2minutes(timePassedSince(timestamp)) < P254_FORCE_TIME)
      {
        // Pump was running shorter than the forced on time
        new_control_state = P254_STATE_FORCE; // Keep running in state FORCE
      }
      else
      {
        timestamp         = millis();
        new_control_state = P254_STATE_IDLE;    // Switch off
      }
      break;

    // Control based on temperature and optional remote control
    case P254_OPMODE_TEMP:
      remote_state = false; // Don't look at requests from remote systems
    // Continue with shared handling, break deliberately not used
    // ----------------------------------------------------------
    case P254_OPMODE_REMOTE:
      switch (old_control_state)
      {
        // Pump is idle
        case P254_STATE_IDLE:

          if ((temperature > P254_TEMP_LEVEL) || remote_state)
          {
            // Heating or remote request to start the pump
            timestamp         = millis();
            new_control_state = P254_STATE_HEATING;
          }
          else if (millis2hours(timePassedSince(timestamp)) >= P254_INTERVAL_TIME)
          {
            // Idling for a long period, forced maintenance run
            timestamp         = millis();
            new_control_state = P254_STATE_FORCE;
          }
          break;

        // Pump is on due to heating request
        case P254_STATE_HEATING:
          if (temperature < (P254_TEMP_LEVEL - P254_TEMP_HYST))
          {
            if (millis2minutes(timePassedSince(timestamp)) >= P254_MIN_TIME)
            {
              timestamp         = millis();
              new_control_state = P254_STATE_IDLE;
            }
            else
            {
              // Keep timestamp from moment pump was swiched on
              new_control_state = P254_STATE_EXTEND;
            }
          }
          break;

        // Pump is on to extend a started cycle till minimum time exceeded
        case P254_STATE_EXTEND:
          if ((temperature > P254_TEMP_LEVEL))
          {
            new_control_state = P254_STATE_HEATING;
          }
          else if (millis2minutes(timePassedSince(timestamp)) >= P254_MIN_TIME)
          {
            timestamp         = millis();
            new_control_state = P254_STATE_IDLE;
          }
          break;

        // Pump was forced on when operation mode switched to temperature control
        case P254_STATE_FORCE:
          if ((temperature > P254_TEMP_LEVEL))
          {
            // Keep timestamp from moment pump was swiched on
            new_control_state = P254_STATE_HEATING;
          }
          else if (millis2minutes(timePassedSince(timestamp)) >= P254_FORCE_TIME)
          {
            timestamp         = millis();
            new_control_state = P254_STATE_IDLE;
          }
          break;
      }
      break; // switch(opmode) case P254_OPMODE_TEMP, P254_OPMODE_REMOTE

    // Unexpected opmode, force to OPMODE_OFF
    default:
      P254_OPMODE = P254_OPMODE_OFF;
      break;
  }

  // Effectuate the newly calculated control state
  switch (new_control_state)
  {
    case P254_STATE_IDLE:
      relay_output     = false; // Relay output state
      led_red_output   = false; // Red LED output state
      led_green_output = true;  // Green LED output state
      led_blue_output  = false; // Blue LED output state
      break;
    case P254_STATE_HEATING:
      relay_output     = true;  // Relay output state
      led_red_output   = true;  // Red LED output state
      led_green_output = false; // Green LED output state
      led_blue_output  = false; // Blue LED output state
      break;
    case P254_STATE_EXTEND:
      relay_output     = true;  // Relay output state
      led_red_output   = false; // Red LED output state
      led_green_output = false; // Green LED output state
      led_blue_output  = true;  // Blue LED output state
      break;
    case P254_STATE_FORCE:
      relay_output     = true;  // Relay output state
      led_red_output   = false; // Red LED output state
      led_green_output = false; // Green LED output state
      led_blue_output  = true;  // Blue LED output state
      break;
  }

  if (validGpio(P254_GPIO_RELAY))
  {
    relay_output ^= P254_getConfigBit(P254_FLAGS, P254_INV_OUTPUT); // Invert when selected
    pinMode(P254_GPIO_RELAY, OUTPUT);
    digitalWrite(P254_GPIO_RELAY, relay_output ? HIGH : LOW);
  }
  if (validGpio(P254_GPIO_LED_RED))
  {
    led_red_output ^= P254_getConfigBit(P254_FLAGS, P254_INV_LED_RED); // Invert when selected
    pinMode(P254_GPIO_LED_RED, OUTPUT);
    digitalWrite(P254_GPIO_LED_RED, led_red_output ? HIGH : LOW);
  }
  if (validGpio(P254_GPIO_LED_GREEN))
  {
    led_green_output ^= P254_getConfigBit(P254_FLAGS, P254_INV_LED_GREEN); // Invert when selected
    pinMode(P254_GPIO_LED_GREEN, OUTPUT);
    digitalWrite(P254_GPIO_LED_GREEN, led_green_output ? HIGH : LOW);
  }
  if (validGpio(P254_GPIO_LED_BLUE))
  {
    led_blue_output ^= P254_getConfigBit(P254_FLAGS, P254_INV_LED_BLUE); // Invert when selected
    pinMode(P254_GPIO_LED_BLUE, OUTPUT);
    digitalWrite(P254_GPIO_LED_BLUE, led_blue_output ? HIGH : LOW);
  }
  
  #ifdef PLUGIN_254_DEBUG
    if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
      String log = F("P254: State= ");
      log += P254_printControlState(new_control_state);
      log += F(" -> ");
      log += F("; relay= ");
      log += relay_output;
      log += F("; red= ");
      log += led_red_output;
      log += F("; green= ");
      log += led_green_output;
      log += F("; blue= ");
      log += led_blue_output;
      log += F("; timer= ");
      log += timePassedSince(timestamp) / 1000;
      log += F("; mode= ");
      log += P254_OPMODE;
      log += F("; temperature= ");
      log += temperature;
      log += F("; remote= ");
      log += remote_state;
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
  #endif // ifdef P164_ENS160_DEBUG

  // Write back updated persistant control data
  P254_CONTROL_STATE = new_control_state;
  P254_TIMESTAMP     = timestamp;
}

#endif // USES_P254
