#ifndef DATASTRUCTS_SECURITYSTRUCT_H
#define DATASTRUCTS_SECURITYSTRUCT_H

#include "../../ESPEasy_common.h"
#include "../CustomBuild/ESPEasyLimits.h"
#include "../DataStructs/ChecksumType.h"

/*********************************************************************************************\
 * SecurityStruct
\*********************************************************************************************/
struct SecurityStruct
{
  enum class WiFiCredentialsSlot {
    first,
    second
  };


  SecurityStruct();

  ChecksumType computeChecksum() const;

  // Return true when stored checksum matches.
  bool checksumMatch() const;

  // Check and update checksum when content was changed.
  // Return true when stored checksum is updated.
  bool updateChecksum();

  void validate();

  // Clear the checksum to make sure file will be saved
  void forceSave();

  void clearWiFiCredentials();

  void clearWiFiCredentials(WiFiCredentialsSlot slot);

  bool hasWiFiCredentials() const;

  bool hasWiFiCredentials(WiFiCredentialsSlot slot) const;

  String getPassword() const;

  char          WifiSSID[32];
  char          WifiKey[64];
  char          WifiSSID2[32];
  char          WifiKey2[64];
  char          WifiAPKey[64];
  char          ControllerUser[CONTROLLER_MAX][26];
  char          ControllerPassword[CONTROLLER_MAX][64];
  char          Password[26];
  uint8_t       AllowedIPrangeLow[4] = {0}; // TD-er: Use these
  uint8_t       AllowedIPrangeHigh[4] = {0};
  uint8_t       IPblockLevel = 0;

  //its safe to extend this struct, up to 4096 bytes, default values in config are 0. Make sure crc is last
  uint8_t       ProgmemMd5[16] = {0}; // crc of the binary that last saved the struct to file.
  uint8_t       md5[16] = {0};
};


#endif // DATASTRUCTS_SECURITYSTRUCT_H
