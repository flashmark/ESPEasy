#ifndef CUSTOMBUILD_STORAGE_LAYOUT_H
#define CUSTOMBUILD_STORAGE_LAYOUT_H

#include "../../ESPEasy_common.h"

/*
   These parameters determine the layout of the settings.
   To see a graphic representation of the active settings in ESPEasy:
   - Run the command:  meminfodetail
   - Go to sysinfo page and scroll to end of page

   These values can be overriden, either here, or via the Custom.h
   The consequence is that the settings used with alternatives for these offsets will not be supported by other builds.

   Some of these values are related to values set in ESPEasyLimits.h
 */


/*
   The settings files have some reserved space for each struct stored in there.

    CONFIG_FILE_SIZE        File size of config.dat

   Settings struct is positioned at the start of the settings file Config.dat

    DAT_BASIC_SETTINGS_SIZE Reserved size for Settings struct

   Parameters used for locating the task data:

    DAT_OFFSET_TASKS        Position of first TaskSettings in the Settings file
    DAT_TASKS_SIZE          Reserved size for TaskSettings
    DAT_TASKS_CUSTOM_SIZE   Reserved size for CustomTaskSettings

    TaskSettings and CustomTaskSettings are stored interleaved.
    Thus the distance to the next TaskSettings is:

    DAT_TASKS_DISTANCE  =   DAT_TASKS_SIZE + DAT_TASKS_CUSTOM_SIZE

   Parameters used for locating the controller data:

    DAT_OFFSET_CONTROLLER         Position of first ControllerSettings in the Settings file
    DAT_CONTROLLER_SIZE           Reserved size for ControllerSettings

    DAT_OFFSET_CUSTOM_CONTROLLER  Position of first CustomControllerSettings in the Settings file
    DAT_CUSTOM_CONTROLLER_SIZE    Reserved size for CustomControllerSettings

    ControllerSettings and CustomControllerSettings are not located interleaved in the settings file.
    So there is no distance value for controller settings.  (equal to the controller size)

   Notification settings are located in a different file.

    DAT_NOTIFICATION_SIZE         Reserved size for NotificationSettings


 */

#ifndef DAT_BASIC_SETTINGS_SIZE
// For size of SettingsStruct stored at this area in config.dat
// See: run_compiletime_checks()
#ifdef ESP8266
# if FEATURE_NON_STANDARD_24_TASKS
# define DAT_BASIC_SETTINGS_SIZE          4096 // Current Settings Struct size is ~2.3k, leave some room to extend
#else
# define DAT_BASIC_SETTINGS_SIZE          3072 // Current Settings Struct size is ~1.3k, leave some room to extend
#endif
#endif
#ifdef ESP32
# define DAT_BASIC_SETTINGS_SIZE          6144 // Current Settings Struct size is ~3k, leave some room to extend
#endif
#endif // ifndef DAT_BASIC_SETTINGS_SIZE


#ifndef DAT_TASKS_SIZE
# define DAT_TASKS_SIZE                   1024
#endif // ifndef DAT_TASKS_SIZE
#ifndef DAT_TASKS_CUSTOM_OFFSET
# define DAT_TASKS_CUSTOM_OFFSET          1024 // Equal to DAT_TASKS_SIZE
#endif // ifndef DAT_TASKS_CUSTOM_OFFSET
#ifndef DAT_TASKS_CUSTOM_SIZE
# define DAT_TASKS_CUSTOM_SIZE            1024
#endif // ifndef DAT_TASKS_CUSTOM_SIZE
// FEATURE_EXTENDED_CUSTOM_SETTINGS: Default will be determined in define_plugin_sets.h, based on used plugins
#ifndef FEATURE_EXTENDED_CUSTOM_SETTINGS
# ifdef BUILD_MINIMAL_OTA
#  define FEATURE_EXTENDED_CUSTOM_SETTINGS  0 // Never on minimal builds?
# else // ifdef BUILD_MINIMAL_OTA
#  define FEATURE_EXTENDED_CUSTOM_SETTINGS  1
# endif // ifdef BUILD_MINIMAL_OTA
#endif // if FEATURE_EXTENDED_CUSTOM_SETTINGS
#ifndef DAT_TASKS_CUSTOM_EXTENSION_SIZE
# if FEATURE_EXTENDED_CUSTOM_SETTINGS
#  define DAT_TASKS_CUSTOM_EXTENSION_SIZE  4096 // 4kB extension with external file extcfg<tasknr>.dat
# else // if FEATURE_EXTENDED_CUSTOM_SETTINGS
#  define DAT_TASKS_CUSTOM_EXTENSION_SIZE  0    // No extension, but defined to avoid #if checks all over the code
# endif // if FEATURE_EXTENDED_CUSTOM_SETTINGS
#endif // ifndef DAT_TASKS_CUSTOM_EXTENSION_SIZE
// Filename _must_ include the task number (1-based, as shown in the UI) and the % is also used elsewhere, so keep the %02d !
#ifndef DAT_TASKS_CUSTOM_EXTENSION_FILEMASK
# define DAT_TASKS_CUSTOM_EXTENSION_FILEMASK "extcfg%02d.dat"
#endif // ifndef DAT_TASKS_CUSTOM_EXTENSION_FILEMASK
#ifndef DAT_TASKS_DISTANCE
# define DAT_TASKS_DISTANCE               2048 // DAT_TASKS_SIZE + DAT_TASKS_CUSTOM_SIZE
#endif // ifndef DAT_TASKS_DISTANCE

#ifndef DAT_CONTROLLER_SIZE
# define DAT_CONTROLLER_SIZE              1024
#endif // ifndef DAT_CONTROLLER_SIZE
#ifndef DAT_CUSTOM_CONTROLLER_SIZE
# define DAT_CUSTOM_CONTROLLER_SIZE       1024
#endif // ifndef DAT_CUSTOM_CONTROLLER_SIZE

#ifndef DAT_NOTIFICATION_SIZE
# define DAT_NOTIFICATION_SIZE            1024
#endif // ifndef DAT_NOTIFICATION_SIZE

#ifndef DAT_SECURITYSETTINGS_SIZE
# define DAT_SECURITYSETTINGS_SIZE        1024
#endif // ifndef DAT_SECURITYSETTINGS_SIZE

#ifndef DAT_EXTDCONTR_CRED_OFFSET
# define DAT_EXTDCONTR_CRED_OFFSET   1024 // Equal to DAT_SECURITYSETTINGS_SIZE
#endif // ifndef DAT_EXTDCONTR_CRED_OFFSET

#ifndef DAT_EXTDCONTR_CRED_SIZE
# define DAT_EXTDCONTR_CRED_SIZE     1024
#endif // ifndef DAT_EXTDCONTR_CRED_SIZE

#ifndef DAT_CDN_SIZE
# define DAT_CDN_SIZE     1024
#endif



/*

   Suggestion for 24 tasks setup:

 #define TASKS_MAX                          24
 #define DAT_OFFSET_CONTROLLER            (DAT_OFFSET_TASKS + (DAT_TASKS_DISTANCE * TASKS_MAX))                        // each controller =
   1k, 4 max
 #define DAT_OFFSET_CUSTOM_CONTROLLER     (DAT_OFFSET_CONTROLLER + (DAT_CUSTOM_CONTROLLER_SIZE * CONTROLLER_MAX))  // each custom controller
   config =
   1k, 4 max


   Alternative:

 #define TASKS_MAX                          24
 #define DAT_OFFSET_CONTROLLER            4096    // 0x1000 each controller = 1k, 4 max
 #define DAT_OFFSET_CUSTOM_CONTROLLER     8192    // 0x2000 each custom controller config = 1k, 4 max.
 #define DAT_OFFSET_TASKS                16384    // 0x4000 each task = 2k, (1024 basic + 1024 bytes custom), 12 max

 */


#if defined(ESP8266)
  # if FEATURE_NON_STANDARD_24_TASKS
  #  ifndef DAT_OFFSET_TASKS
  #   define DAT_OFFSET_TASKS                 4096                                                             // 0x1000 each task = 2k,
                                                                                                               // (1024 basic + 1024 bytes
                                                                                                               // custom)
  #  endif // ifndef DAT_OFFSET_TASKS
  #  ifndef DAT_OFFSET_CONTROLLER
  #   define DAT_OFFSET_CONTROLLER            (DAT_OFFSET_TASKS + (DAT_TASKS_DISTANCE * TASKS_MAX))  // each controller = 1k, 3 max, DAT_OFFSET_CDN is at position of any 4th controller.
  #  endif // ifndef DAT_OFFSET_CONTROLLER
  #  ifndef DAT_OFFSET_CUSTOM_CONTROLLER
  #   define DAT_OFFSET_CUSTOM_CONTROLLER     (DAT_OFFSET_CONTROLLER + (DAT_CONTROLLER_SIZE * CONTROLLER_MAX))  // each custom controller config = 1k, 3 max
  #  endif // ifndef DAT_OFFSET_CUSTOM_CONTROLLER
  # ifndef DAT_OFFSET_CDN
  #  define DAT_OFFSET_CDN                    (DAT_OFFSET_CUSTOM_CONTROLLER - DAT_CDN_SIZE)  // single CDN settings block of 1k
  # endif 

  #  ifndef CONFIG_FILE_SIZE
  #   define CONFIG_FILE_SIZE                65536
  #  endif // ifndef CONFIG_FILE_SIZE

  # else // if FEATURE_NON_STANDARD_24_TASKS

  #  ifndef DAT_OFFSET_TASKS
  #   define DAT_OFFSET_TASKS                 4096 // each task = 2k, (1024 basic + 1024 bytes custom), 12 max
  #  endif // ifndef DAT_OFFSET_TASKS
  #  ifndef DAT_OFFSET_CONTROLLER
  #   define DAT_OFFSET_CONTROLLER           28672 // each controller = 1k, 4 max
  #  endif // ifndef DAT_OFFSET_CONTROLLER
  #  ifndef DAT_OFFSET_CUSTOM_CONTROLLER
  #   define DAT_OFFSET_CUSTOM_CONTROLLER    32768 // each custom controller config = 1k, 4 max.
  #  endif // ifndef DAT_OFFSET_CUSTOM_CONTROLLER
  # ifndef DAT_OFFSET_CDN
  #  define DAT_OFFSET_CDN                   (DAT_OFFSET_TASKS - DAT_CDN_SIZE)  // single CDN settings block of 1k
  # endif 
  #  ifdef LIMIT_BUILD_SIZE

// Limit the config size for 1M builds, since their file system is also quite small
  #   ifndef CONFIG_FILE_SIZE
  #    define CONFIG_FILE_SIZE               36864 // DAT_OFFSET_CUSTOM_CONTROLLER + 4x DAT_CUSTOM_CONTROLLER_SIZE
  #   endif // ifndef CONFIG_FILE_SIZE
  #  else // ifdef LIMIT_BUILD_SIZE
  #   ifndef CONFIG_FILE_SIZE
  #    define CONFIG_FILE_SIZE               65536
  #   endif // ifndef CONFIG_FILE_SIZE
  #  endif // ifdef LIMIT_BUILD_SIZE
  # endif // if FEATURE_NON_STANDARD_24_TASKS
#endif     // if defined(ESP8266)

#if defined(ESP32)
  # ifndef DAT_OFFSET_TASKS
  #  define DAT_OFFSET_TASKS                 32768 // each task = 2k, (1024 basic + 1024 bytes custom), 32 max
  # endif // ifndef DAT_OFFSET_TASKS
  # ifndef DAT_OFFSET_CONTROLLER
  #  define DAT_OFFSET_CONTROLLER           8192   // each controller = 1k, 4 max
  # endif // ifndef DAT_OFFSET_CONTROLLER
  # ifndef DAT_OFFSET_CUSTOM_CONTROLLER
  #  define DAT_OFFSET_CUSTOM_CONTROLLER    12288  // each custom controller config = 1k, 4 max.
  # endif // ifndef DAT_OFFSET_CUSTOM_CONTROLLER
  # ifndef DAT_OFFSET_CDN
  #  define DAT_OFFSET_CDN                   (DAT_OFFSET_CONTROLLER - DAT_CDN_SIZE)  // single CDN settings block of 1k
  # endif 
  # ifndef CONFIG_FILE_SIZE
  #  define CONFIG_FILE_SIZE               131072
  # endif // ifndef CONFIG_FILE_SIZE
#endif    // if defined(ESP32)


#endif    // CUSTOMBUILD_STORAGE_LAYOUT_H
