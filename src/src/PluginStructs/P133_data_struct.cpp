#include "../PluginStructs/P133_data_struct.h"

#ifdef USES_P133

/**************************************************************************
* Constructor
**************************************************************************/
P133_data_struct::P133_data_struct(P133_selectMode_e   selectMode,
                                   ltr390_gain_t       uvGain,
                                   ltr390_resolution_t uvResolution,
                                   ltr390_gain_t       alsGain,
                                   ltr390_resolution_t alsResolution,
                                   bool                initReset)
  : _selectMode(selectMode), _uvGain(uvGain), _uvResolution(uvResolution),
  _alsGain(alsGain), _alsResolution(alsResolution), _initReset(initReset)
{}

/*****************************************************
* Destructor
*****************************************************/
P133_data_struct::~P133_data_struct() {
  if (nullptr != ltr390) { delete ltr390; }
  ltr390 = nullptr;
}

/*****************************************************
* plugin_read
*****************************************************/
bool P133_data_struct::plugin_read(struct EventStruct *event)           {
  if (initialised) {
    // Last obtained values
    UserVar.setFloat(event->TaskIndex, 0, uvValue);
    UserVar.setFloat(event->TaskIndex, 1, uviValue);
    UserVar.setFloat(event->TaskIndex, 2, alsValue);
    UserVar.setFloat(event->TaskIndex, 3, luxValue);
    sensorRead = false;
    return true;
  }
  return false;
}

/*****************************************************
* plugin_ten_per_second
*****************************************************/
bool P133_data_struct::plugin_ten_per_second(struct EventStruct *event) {
  if (initialised && ltr390->newDataAvailable()) {
    if (mode == LTR390_MODE_UVS) {
      uvValue  = ltr390->readUVS();
      uviValue = ltr390->getUVI();
    } else {
      alsValue = ltr390->readALS();
      luxValue = ltr390->getLUX();
    }

    // Dual Mode: Switch mode after measuring a value for a few loop cycles, so the data is stable
    if (_selectMode == P133_selectMode_e::DualMode) {
      if ((loopCounter == 0) || (loopCounter > P133_LOOP_INTERVAL)) {
        if (mode == LTR390_MODE_UVS) {
          mode = LTR390_MODE_ALS;
          ltr390->setGain(_alsGain);
          ltr390->setResolution(_alsResolution);
        } else {
          mode = LTR390_MODE_UVS;
          ltr390->setGain(_uvGain);
          ltr390->setResolution(_uvResolution);
        }
        ltr390->setMode(mode);

        loopCounter = P133_LOOP_INTERVAL;
      } else {
        loopCounter--;
      }
    }

    # if PLUGIN_133_DEBUG

    if (!sensorRead && loglevelActiveFor(LOG_LEVEL_INFO)) {
      addLogMove(LOG_LEVEL_INFO,
                 strformat(F("LTR390: data read. Mode: %d, UV: %d, UVindex: %.2f, ALS: %d, Lux: %.2f"),
                           mode, uvValue, uviValue, alsValue, luxValue));
    }
    # endif // if PLUGIN_133_DEBUG
    sensorRead = true;
    return true;
  }
  return false;
}

/**************************************************************************
* plugin_init Initialize sensor and prepare for reading
**************************************************************************/
bool P133_data_struct::plugin_init(struct EventStruct *event) {
  if (!initialised) {
    initialised = init_sensor(); // Check if device is present
  }

  if (initialised) {
    switch (_selectMode) {
      case P133_selectMode_e::DualMode:
      case P133_selectMode_e::UVMode:
        mode = LTR390_MODE_UVS;
        ltr390->setGain(_uvGain);
        ltr390->setResolution(_uvResolution);
        break;
      case P133_selectMode_e::ALSMode:
        mode = LTR390_MODE_ALS;
        ltr390->setGain(_alsGain);
        ltr390->setResolution(_alsResolution);
        break;
    }
    ltr390->setMode(mode);

    # if PLUGIN_133_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      addLogMove(LOG_LEVEL_INFO,
                 strformat(F("LTR390: Configured, mode: %d, UV gain: %d, UV resolution: %d, ALS gain: %d, ALS resolution: %d"),
                           mode, _uvGain, _uvResolution, _alsGain, _alsResolution));
    }
    # endif // if PLUGIN_133_DEBUG
  }

  return initialised;
}

/**************************************************************************
* Check LTR390 presence and initialize
**************************************************************************/
bool P133_data_struct::init_sensor() {
  if (!initialised) {
    ltr390 = new (std::nothrow) UVlight_LTR390();

    if (nullptr != ltr390) {
      initialised = ltr390->init(_initReset);
    }

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = concat(F("LTR390: Initialized: "), initialised ? F("OK") : F("ERROR"));
      log += strformat(F(", chip ID: 0x%02x"), ltr390->getChipID());
      addLogMove(LOG_LEVEL_INFO, log);
    }
  }

  return initialised;
}

#endif // ifdef USES_P133
