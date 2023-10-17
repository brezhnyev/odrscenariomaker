#include <string>
#include <iostream>

const char * xosc_template_header = R"(
<?xml version='1.0' encoding='UTF-8'?>
<OpenSCENARIO>
  <FileHeader author="" date="" description="" revMajor="1" revMinor="0"/>
  <ParameterDeclarations>
  </ParameterDeclarations>
  <CatalogLocations/>
  <RoadNetwork>
    <LogicFile filepath=""/>
    <SceneGraphFile filepath=""/>
  </RoadNetwork>)";

const char * xosc_template_start_entities = R"(
  <Entities>)";

const char * xosc_template_end_entities = R"(
  </Entities>)";

const char * xosc_template_vehicle = R"(
    <ScenarioObject name="">
      <Vehicle name="" vehicleCategory="">
        <ParameterDeclarations/>
        <BoundingBox>
          <Center x="" y="" z=""/>
          <Dimensions width="" length="" height=""/>
        </BoundingBox>
        <Performance maxSpeed="69.44444444444444" maxAcceleration="4.5" maxDeceleration="8.0"/>
        <Axles>
          <FrontAxle maxSteering="0.6108652381980153" wheelDiameter="0.508" trackWidth="1.543" positionX="2.621" positionZ="0.254"/>
          <RearAxle maxSteering="0.0" wheelDiameter="0.508" trackWidth="1.514" positionX="0.0" positionZ="0.254"/>
        </Axles>
        <Properties>
          <Property name="type" value=""/>
        </Properties>
      </Vehicle>
    </ScenarioObject>)";

const char * xosc_template_pedestrian = R"(
    <ScenarioObject name="Ped">
      <Pedestrian model="" mass="90.0" name="Ped" pedestrianCategory="pedestrian">
        <ParameterDeclarations/>
        <BoundingBox>
          <Center x="" y="" z=""/>
          <Dimensions width="" length="" height=""/>
        </BoundingBox>
        <Properties/>
      </Pedestrian>
    </ScenarioObject>)";
  // </Entities>

const char * xosc_template_footer = R"(
  </Storyboard>
</OpenSCENARIO>)";

const char * xosc_template_start_storyboard = R"(
  <Storyboard>
    <Init>
      <Actions>
        <GlobalAction>
          <EnvironmentAction>
            <Environment name="Environment 1">
              <TimeOfDay animation="false" dateTime="2022-08-30T12:00:00"/>
              <Weather cloudState="free">
                <Sun intensity="0.85" azimuth="0.0" elevation="1.31"/>
                <Fog visualRange="100000.0"/>
                <Precipitation precipitationType="dry" intensity="0.0"/>
              </Weather>
              <RoadCondition frictionScaleFactor="1.0"/>
            </Environment>
          </EnvironmentAction>
        </GlobalAction>)";

const char * xosc_template_action_member = R"(
        <Private entityRef="">
          <PrivateAction>
            <TeleportAction>
              <Position>
                <WorldPosition x="" y="" z="" h="" p="" r="" />
              </Position>
            </TeleportAction>
          </PrivateAction>
          <PrivateAction>
            <LongitudinalAction>
              <SpeedAction>
                <SpeedActionDynamics dynamicsShape="step" value="0.0" dynamicsDimension="time"/>
                <SpeedActionTarget>
                  <AbsoluteTargetSpeed value=""/>
                </SpeedActionTarget>
              </SpeedAction>
            </LongitudinalAction>
          </PrivateAction>
        </Private>)";

const char * xosc_template_end_init_storyboard = R"(
      </Actions>
    </Init>)";

const char * xosc_template_story = R"(
    <Story name="Story 1">
    </Story>)";