#pragma once

#include "Selectable.h"

#include <yaml-cpp/yaml.h>

#include <string>

class Serializer
{
public:
    std::string serialize_yaml(Selectable * object);
    void serialize_yaml(YAML::Node & node, Selectable * object);

    std::string deserialize_yaml(Selectable * object);
};