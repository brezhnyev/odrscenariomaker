#include <string>
#include <iostream>

const char * xosc_template = R"(<?xml version='1.0' encoding='UTF-8'?>
<OpenSCENARIO>
  <FileHeader author="Capgemini" date="" description="" revMajor="1" revMinor="0"/>
  <ParameterDeclarations>
  </ParameterDeclarations>
  <CatalogLocations/>
  <RoadNetwork>
    <LogicFile filepath=""/>
    <SceneGraphFile filepath=""/>
  </RoadNetwork>
  <Entities>
  </Entities>
  <Storyboard>
    <Init>
      <Actions>
      </Actions>
    </Init>
  <StopTrigger/>
  </Storyboard>
</OpenSCENARIO>)";

// Init components

const char * xosc_template_vehicle = R"(
    <ScenarioObject name="">
      <Vehicle name="" vehicleCategory="car">
        <ParameterDeclarations/>
        <BoundingBox>
          <Center x="0" y="0" z=""/>
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
    </ScenarioObject>
)";

const char * xosc_template_pedestrian = R"(
    <ScenarioObject name="">
      <Pedestrian model="" mass="90.0" name="Ped" pedestrianCategory="pedestrian">
        <ParameterDeclarations/>
        <BoundingBox>
          <Center x="0" y="0" z=""/>
          <Dimensions width="" length="" height=""/>
        </BoundingBox>
        <Properties/>
      </Pedestrian>
    </ScenarioObject>
)";

// storyboard init components:

const char * xosc_template_global_action = R"(
        <GlobalAction>
          <EnvironmentAction>
            <Environment name="Environment 1">
              <TimeOfDay animation="false" dateTime=""/>
              <Weather cloudState="free">
                <Sun intensity="0.85" azimuth="0.0" elevation="1.31"/>
                <Fog visualRange="100000.0"/>
                <Precipitation precipitationType="dry" intensity="0.0"/>
              </Weather>
              <RoadCondition frictionScaleFactor="1.0"/>
            </Environment>
          </EnvironmentAction>
        </GlobalAction>
)";

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
                  <AbsoluteTargetSpeed value="0.0"/>
                </SpeedActionTarget>
              </SpeedAction>
            </LongitudinalAction>
          </PrivateAction>
        </Private>
)";

// storyboard story components:

const char * xosc_template_story = R"(
    <Story name="">
      <ParameterDeclarations/>
      <Act name="Act 1">
        <ManeuverGroup maximumExecutionCount="1" name="ManeuverSequence">
          <Actors selectTriggeringEntities="false">
            <EntityRef entityRef=""/>
          </Actors>
          <Maneuver name="WaypathManeuver">
          </Maneuver>
        </ManeuverGroup>
        <StartTrigger>
          <ConditionGroup>
            <Condition name="StartTime" conditionEdge="rising" delay="0.0">
              <ByValueCondition>
                <SimulationTimeCondition value="0.0" rule="equalTo"/>
              </ByValueCondition>
            </Condition>
          </ConditionGroup>
        </StartTrigger>
        <StopTrigger>
            <ConditionGroup>
              <Condition name="Travelled Distance" delay="0.0" conditionEdge="rising">
                <ByEntityCondition>
                  <TriggeringEntities triggeringEntitiesRule="any">
                    <EntityRef entityRef="hero"/>
                  </TriggeringEntities>
                  <EntityCondition>
                    <TraveledDistanceCondition value="100.0"/>
                  </EntityCondition>
                </ByEntityCondition>
              </Condition>
              <Condition name="Max Simulation Time" delay="0.0" conditionEdge="rising">
                <ByValueCondition>
                  <SimulationTimeCondition rule="greaterThan" value="100.0" />
                </ByValueCondition>
              </Condition>
            </ConditionGroup>
        </StopTrigger>
      </Act>
    </Story>
)";

const char * xosc_template_waypath_event = R"(
            <Event name="ActorEvent" priority="overwrite">
              <Action name="RouteCreation">
                <PrivateAction>
                  <RoutingAction>
                    <AssignRouteAction>
                      <Route name="ActorTrajectory" closed="false">
                      </Route>
                    </AssignRouteAction>
                  </RoutingAction>
                </PrivateAction>
              </Action>
              <StartTrigger>
                <ConditionGroup>
                  <Condition name="MyCondition" delay="0" conditionEdge="rising">
                    <ByValueCondition>
                      <SimulationTimeCondition value="0.0" rule="equalTo"/>
                    </ByValueCondition>
                  </Condition>
                </ConditionGroup>
              </StartTrigger>
            </Event>
)";

const char * xosc_template_speed_event = R"(
            <Event name="" priority="overwrite">
              <Action name="">
                <PrivateAction>
                  <LongitudinalAction>
                    <SpeedAction>
                      <SpeedActionDynamics dynamicsShape="step" value="3" dynamicsDimension="distance"/>
                      <SpeedActionTarget>
                        <AbsoluteTargetSpeed value=""/>
                      </SpeedActionTarget>
                    </SpeedAction>
                  </LongitudinalAction>
                </PrivateAction>
              </Action>
              <StartTrigger>
                <ConditionGroup>
                  <Condition name="StartCondition" delay="0" conditionEdge="rising">
                    <ByEntityCondition>
                      <TriggeringEntities triggeringEntitiesRule="any">
                        <EntityRef entityRef=""/>
                      </TriggeringEntities>
                      <EntityCondition>
                        <ReachPositionCondition tolerance="1.0">
                          <Position>
                            <WorldPosition x="" y="" z=""/>
                          </Position>
                        </ReachPositionCondition>
                      </EntityCondition>
                    </ByEntityCondition>
                  </Condition>
                </ConditionGroup>
              </StartTrigger>
            </Event>
)";

const char * xosc_template_waypoint = R"(
                          <Waypoint routeStrategy="shortest">
                            <Position>
                              <WorldPosition x="" y="" z="" h="0"/>
                            </Position>
                          </Waypoint>
)";

const char * xosc_template_hero_stop_act_trigger = R"(
        <StopTrigger>
            <ConditionGroup>
              <Condition name="Travelled Distance" delay="0.0" conditionEdge="rising">
                <ByEntityCondition>
                  <TriggeringEntities triggeringEntitiesRule="any">
                    <EntityRef entityRef="hero"/>
                  </TriggeringEntities>
                  <EntityCondition>
                    <TraveledDistanceCondition value="100.0"/>
                  </EntityCondition>
                </ByEntityCondition>
              </Condition>
              <Condition name="Max Simulation Time" delay="0.0" conditionEdge="rising">
                <ByValueCondition>
                  <SimulationTimeCondition rule="greaterThan" value="100.0" />
                </ByValueCondition>
              </Condition>
            </ConditionGroup>
        </StopTrigger>
)";