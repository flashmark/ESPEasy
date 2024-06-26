#ifndef PLUGINSTRUCTS_P069_DATA_STRUCT_H
#define PLUGINSTRUCTS_P069_DATA_STRUCT_H

#include "../../_Plugin_Helper.h"
#ifdef USES_P069


struct P069_data_struct : public PluginTaskData_base {
public:

  P069_data_struct(uint8_t addr);

  P069_data_struct() = delete;
  virtual ~P069_data_struct() = default;

  float getTemperatureInDegrees() const;

private:

  uint8_t _i2c_device_address;
};

#endif // ifdef USES_P069
#endif // ifndef PLUGINSTRUCTS_P069_DATA_STRUCT_H
