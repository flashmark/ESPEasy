#include "../Globals/RuntimeData.h"


std::map<uint32_t, ESPEASY_RULES_FLOAT_TYPE> customFloatVar;

//float UserVar[VARS_PER_TASK * TASKS_MAX];

UserVarStruct UserVar;


ESPEASY_RULES_FLOAT_TYPE getCustomFloatVar(uint32_t index) {
  auto it = customFloatVar.find(index);

  if (it != customFloatVar.end()) {
    return it->second;
  }
  return 0.0;
}

void setCustomFloatVar(uint32_t index, const ESPEASY_RULES_FLOAT_TYPE& value) {
  customFloatVar[index] = value;
}

bool getNextCustomFloatVar(uint32_t& index, ESPEASY_RULES_FLOAT_TYPE& value) {
  auto it = customFloatVar.find(index);

  if (it == customFloatVar.end()) { return false; }
  ++it;

  if (it == customFloatVar.end()) { return false; }
  index = it->first;
  value = it->second;
  return true;
}
