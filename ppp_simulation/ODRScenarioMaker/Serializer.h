#pragma once

#include "scenario.h"

#include <yaml-cpp/yaml.h>

#include <string>

class Serializer
{
public:
    std::string serialize_yaml(Selectable * object);
    void serialize_yaml(YAML::Node & node, Selectable * object);

    Scenario deserialize_yaml(const std::string & data);
    void deserialize_yaml(YAML::Node node, Selectable & object);
};