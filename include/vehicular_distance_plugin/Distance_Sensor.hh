#ifndef VEHICULAR_DISTANCE_PLUGIN_DISTANCE_SENSOR_HH_
#define VEHICULAR_DISTANCE_PLUGIN_DISTANCE_SENSOR_HH_

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace Distance_Sensor {

/// \brief Publishes a string when a named model crosses a distance threshold.
///
/// Both the vehicle and target are named in SDF. The monitored point is an FLU
/// offset in the vehicle model's local coordinate frame.
class Distance_SensorSystem : public gz::sim::System,
                              public gz::sim::ISystemConfigure,
                              public gz::sim::ISystemPostUpdate {
public:
  Distance_SensorSystem() = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventManager) override;

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

private:
  void OnTriggerMessage(const gz::msgs::StringMsg &_message);

  gz::sim::Entity vehicleEntity{gz::sim::kNullEntity};
  gz::sim::Entity targetEntity{gz::sim::kNullEntity};
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;

  std::string vehicleName;
  std::string targetName;
  std::string belowThresholdMessage;
  std::string aboveThresholdMessage;
  std::string triggerCondition;
  gz::math::Vector3d vehicleLocalOffset{0.0, 0.0, 0.0};
  double distanceThreshold{0.0};
  double minimumTimeWithinThreshold{0.0};

  bool configured{false};
  bool warnedMissingVehicle{false};
  bool warnedMissingTarget{false};
  bool isActivated{false};
  bool belowThresholdPublished{false};
  bool aboveThresholdPublished{false};
  std::optional<std::chrono::steady_clock::duration> belowThresholdSince;
  std::mutex activationMutex;
};

} // namespace Distance_Sensor

#endif // VEHICULAR_DISTANCE_PLUGIN_DISTANCE_SENSOR_HH_
