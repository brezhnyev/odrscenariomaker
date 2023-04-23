#pragma once

#include "scenario.h"
#include <string>
#include <yaml-cpp/yaml.h>

class Serializer
{
public:
    static std::string serialize_yaml(Selectable * object);
    static void deserialize_yaml(Scenario & scenario, const std::string & data);

private:
    static void serialize_yaml(YAML::Node & node, Selectable * object);
    static void deserialize_yaml(YAML::Node node, Selectable & object);
};