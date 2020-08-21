#pragma once

#include "scenario.h"
#include <string>
#include <boost/next_prior.hpp>
#include <yaml-cpp/yaml.h>

class Serializer
{
public:
    std::string serialize_yaml(Selectable * object);
    Scenario deserialize_yaml(const std::string & data);

private:
    void serialize_yaml(YAML::Node & node, Selectable * object);
    void deserialize_yaml(YAML::Node node, Selectable & object);
};