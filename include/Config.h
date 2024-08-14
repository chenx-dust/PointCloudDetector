#pragma once

#include <yaml-cpp/yaml.h>

extern YAML::Node CF_CONFIG, CF_DEFAULT;

#define CFG1(key1, type) CF_CONFIG[key1].as<type>(CF_DEFAULT[key1].as<type>())
#define CFG2(key1, key2, type) CF_CONFIG[key1][key2].as<type>(CF_DEFAULT[key1][key2].as<type>())
#define CFG3(key1, key2, key3, type) CF_CONFIG[key1][key2][key3].as<type>(CF_DEFAULT[key1][key2][key3].as<type>())
#define CFG4(key1, key2, key3, key4, type) CF_CONFIG[key1][key2][key3][key4].as<type>(CF_DEFAULT[key1][key2][key3][key4].as<type>())
#define CFG5(key1, key2, key3, key4, key5, type) CF_CONFIG[key1][key2][key3][key4][key5].as<type>(CF_DEFAULT[key1][key2][key3][key4][key5].as<type>())

inline void Config_load()
{
    CF_CONFIG = YAML::LoadFile("config.yaml");
    CF_DEFAULT = YAML::LoadFile("default.yaml");
}
