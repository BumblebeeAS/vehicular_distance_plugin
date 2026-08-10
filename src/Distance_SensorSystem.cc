#include "vehicular_distance_plugin/Distance_Sensor.hh"

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include <sdf/Element.hh>

#include <string>

namespace Distance_Sensor {

void Distance_SensorSystem::Configure(
    const gz::sim::Entity & /* _entity */,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /* _eventManager */) {
  if (!_sdf->HasElement("vehicle_name") || !_sdf->HasElement("target_name") ||
      !_sdf->HasElement("distance_threshold") ||
      !_sdf->HasElement("gz_publish_topic")) {
    gzerr << "[Distance_SensorSystem] Required SDF elements are "
          << "<vehicle_name>, <target_name>, <distance_threshold>, and "
          << "<gz_publish_topic>.\n";
    return;
  }

  this->vehicleName = _sdf->Get<std::string>("vehicle_name");
  this->targetName = _sdf->Get<std::string>("target_name");
  this->distanceThreshold = _sdf->Get<double>("distance_threshold");
  this->minimumTimeWithinThreshold =
      _sdf->Get<double>("minimum_time_within_threshold", 0.0).first;
  const std::string publishTopic =
      _sdf->Get<std::string>("gz_publish_topic");

  if (this->vehicleName.empty() || this->targetName.empty() ||
      this->distanceThreshold < 0.0 || this->minimumTimeWithinThreshold < 0.0) {
    gzerr << "[Distance_SensorSystem] Model names must not be empty, and "
          << "distance and time thresholds must be non-negative.\n";
    return;
  }
  if (this->vehicleName == this->targetName) {
    gzerr << "[Distance_SensorSystem] <vehicle_name> and <target_name> must "
          << "identify two different models.\n";
    return;
  }

  if (publishTopic.empty()) {
    gzerr << "[Distance_SensorSystem] <gz_publish_topic> must not be empty.\n";
    return;
  }

  // FLU vehicle frame: +X forward, +Y left, +Z up.
  const auto offset = [&_sdf](const std::string &_name,
                              const std::string &_legacyName) {
    if (_sdf->HasElement(_name)) {
      return _sdf->Get<double>(_name);
    }
    return _sdf->Get<double>(_legacyName, 0.0).first;
  };
  this->vehicleLocalOffset.Set(offset("vehicle_offset_x", "magnet_positionx"),
                               offset("vehicle_offset_y", "magnet_positiony"),
                               offset("vehicle_offset_z", "magnet_positionz"));

  this->publisher = this->node.Advertise<gz::msgs::StringMsg>(publishTopic);
  if (!this->publisher) {
    gzerr << "[Distance_SensorSystem] Failed to advertise [" << publishTopic
          << "].\n";
    return;
  }

  this->vehicleEntity =
      _ecm.EntityByComponents(gz::sim::components::Name(this->vehicleName),
                              gz::sim::components::Model());
  this->targetEntity =
      _ecm.EntityByComponents(gz::sim::components::Name(this->targetName),
                              gz::sim::components::Model());
  this->configured = true;

  gzmsg << "[Distance_SensorSystem] Monitoring [" << this->vehicleName
        << "] relative to target [" << this->targetName << "] with threshold "
        << this->distanceThreshold << " m and minimum dwell "
        << this->minimumTimeWithinThreshold << " s; publishing on ["
        << publishTopic << "].\n";
}

void Distance_SensorSystem::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) {
  if (_info.paused || !this->configured) {
    return;
  }

  if (this->vehicleEntity == gz::sim::kNullEntity) {
    this->vehicleEntity =
        _ecm.EntityByComponents(gz::sim::components::Name(this->vehicleName),
                                gz::sim::components::Model());
    if (this->vehicleEntity == gz::sim::kNullEntity) {
      if (!this->warnedMissingVehicle) {
        gzwarn << "[Distance_SensorSystem] Waiting for model ["
               << this->vehicleName << "].\n";
        this->warnedMissingVehicle = true;
      }
      return;
    }
  }

  if (this->targetEntity == gz::sim::kNullEntity) {
    this->targetEntity =
        _ecm.EntityByComponents(gz::sim::components::Name(this->targetName),
                                gz::sim::components::Model());
    if (this->targetEntity == gz::sim::kNullEntity) {
      if (!this->warnedMissingTarget) {
        gzwarn << "[Distance_SensorSystem] Waiting for model ["
               << this->targetName << "].\n";
        this->warnedMissingTarget = true;
      }
      return;
    }
  }

  const gz::math::Pose3d vehiclePose =
      gz::sim::worldPose(this->vehicleEntity, _ecm);
  const gz::math::Pose3d targetPose =
      gz::sim::worldPose(this->targetEntity, _ecm);
  const gz::math::Vector3d monitoredPosition =
      vehiclePose.Pos() +
      vehiclePose.Rot().RotateVector(this->vehicleLocalOffset);
  const double distance = (targetPose.Pos() - monitoredPosition).Length();
  const bool belowThreshold = distance < this->distanceThreshold;

  if (belowThreshold) {
    if (this->detectionPublished) {
      return;
    }
    if (!this->belowThresholdSince.has_value() ||
        _info.simTime < *this->belowThresholdSince) {
      this->belowThresholdSince = _info.simTime;
    }

    const double timeWithinThreshold =
        std::chrono::duration<double>(_info.simTime -
                                      *this->belowThresholdSince)
            .count();
    if (timeWithinThreshold < this->minimumTimeWithinThreshold) {
      return;
    }
  } else {
    this->belowThresholdSince.reset();
    this->detectionPublished = false;
    return;
  }

  gz::msgs::StringMsg message;
  message.set_data("activate");
  if (!this->publisher.Publish(message)) {
    gzerr << "[Distance_SensorSystem] Failed to publish proximity detection.\n";
    return;
  }

  this->detectionPublished = true;
  gzmsg << "[Distance_SensorSystem] Distance " << distance
        << " m is below threshold; published [" << message.data() << "].\n";
}

} // namespace Distance_Sensor

GZ_ADD_PLUGIN(Distance_Sensor::Distance_SensorSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(Distance_Sensor::Distance_SensorSystem,
                    "Distance_Sensor::Distance_SensorSystem")
