#ifndef DATATYPES_SETTINGSTYPE_H
#define DATATYPES_SETTINGSTYPE_H

#include "../../ESPEasy_common.h"
#include "../DataTypes/TaskIndex.h"

class SettingsType {
public:

  enum class Enum {
    BasicSettings_Type = 0,
    TaskSettings_Type,
    CustomTaskSettings_Type,
    ControllerSettings_Type,
    CustomControllerSettings_Type,
    NotificationSettings_Type,
    SecuritySettings_Type,
    ExtdControllerCredentials_Type,
#if FEATURE_ALTERNATIVE_CDN_URL
    CdnSettings_Type,
#endif

    SettingsType_MAX
  };

  enum class SettingsFileEnum : uint8_t {
    FILE_CONFIG_type,
    FILE_NOTIFICATION_type,
    FILE_SECURITY_type,
    FILE_UNKNOWN_type
  };

  static const __FlashStringHelper * getSettingsTypeString(Enum settingsType);
  static bool   getSettingsParameters(Enum settingsType,
                                      int  index,
                                      int& offset,
                                      int& max_size);
  static bool getSettingsParameters(Enum settingsType,
                                    int  index,
                                    int& max_index,
                                    int& offset,
                                    int& max_size,
                                    int& struct_size);

  static int              getMaxFilePos(Enum settingsType);
  static int              getFileSize(Enum settingsType);

#ifndef BUILD_MINIMAL_OTA
  static unsigned int     getSVGcolor(Enum settingsType);
#endif // ifndef BUILD_MINIMAL_OTA

  static SettingsFileEnum getSettingsFile(Enum settingsType);
  static String           getSettingsFileName(Enum settingsType,
                                              int  index = INVALID_TASK_INDEX);
  static const __FlashStringHelper * getSettingsFileName(SettingsType::SettingsFileEnum file_type);
  static size_t           getInitFileSize(SettingsType::SettingsFileEnum file_type);
};


#endif // DATATYPES_SETTINGSTYPE_H
