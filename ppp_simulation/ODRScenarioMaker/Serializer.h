#pragma once

#include "scenario.h"
#include <string>
#include <boost/next_prior.hpp>
#include <yaml-cpp/yaml.h>

class Serializer
{
public:
    static std::string serialize_yaml(Selectable * object);
    static Scenario deserialize_yaml(const std::string & data);

private:
    static void serialize_yaml(YAML::Node & node, Selectable * object);
    static void deserialize_yaml(YAML::Node node, Selectable & object);
};