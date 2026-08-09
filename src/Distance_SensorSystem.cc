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
      !_sdf->HasElement("distance_threshold")) {
    gzerr << "[Distance_SensorSystem] Required SDF elements are "
          << "<vehicle_name>, <target_name>, and <distance_threshold>.\n";
    return;
  }

  this->vehicleName = _sdf->Get<std::string>("vehicle_name");
  this->targetName = _sdf->Get<std::string>("target_name");
  this->distanceThreshold = _sdf->Get<double>("distance_threshold");
  this->minimumTimeWithinThreshold =
      _sdf->Get<double>("minimum_time_within_threshold", 0.0).first;
  const std::string outputTopic =
      _sdf->Get<std::string>("output_topic", "").first;

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

  if (outputTopic.empty()) {
    gzmsg << "[Distance_SensorSystem] <output_topic> is empty; distance "
          << "trigger is disabled.\n";
    return;
  }

  if (_sdf->HasElement("below_threshold_message")) {
    this->belowThresholdMessage =
        _sdf->Get<std::string>("below_threshold_message");
  } else if (_sdf->HasElement("gz_topic_output_if_below_threshold")) {
    this->belowThresholdMessage =
        _sdf->Get<std::string>("gz_topic_output_if_below_threshold");
  } else {
    gzerr << "[Distance_SensorSystem] Missing "
          << "<below_threshold_message>.\n";
    return;
  }
  if (_sdf->HasElement("above_threshold_message")) {
    this->aboveThresholdMessage =
        _sdf->Get<std::string>("above_threshold_message");
  } else if (_sdf->HasElement("gz_topic_output_if_above_threshold")) {
    this->aboveThresholdMessage =
        _sdf->Get<std::string>("gz_topic_output_if_above_threshold");
  } else {
    gzerr << "[Distance_SensorSystem] Missing "
          << "<above_threshold_message>.\n";
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

  this->publisher = this->node.Advertise<gz::msgs::StringMsg>(outputTopic);
  if (!this->publisher) {
    gzerr << "[Distance_SensorSystem] Failed to advertise [" << outputTopic
          << "].\n";
    return;
  }

  const std::string subscribeTopic =
      _sdf->Get<std::string>("gz_subscribe_topic", "").first;
  if (subscribeTopic.empty()) {
    this->isActivated = true;
  } else {
    if (!_sdf->HasElement("trigger_condition")) {
      gzerr << "[Distance_SensorSystem] <trigger_condition> is required when "
            << "<gz_subscribe_topic> is not empty.\n";
      return;
    }

    this->triggerCondition = _sdf->Get<std::string>("trigger_condition");
    if (this->triggerCondition.empty()) {
      gzerr << "[Distance_SensorSystem] <trigger_condition> must not be "
            << "empty.\n";
      return;
    }

    if (!this->node.Subscribe(subscribeTopic,
                              &Distance_SensorSystem::OnTriggerMessage, this)) {
      gzerr << "[Distance_SensorSystem] Failed to subscribe to ["
            << subscribeTopic << "].\n";
      return;
    }

    gzmsg << "[Distance_SensorSystem] Waiting for [" << this->triggerCondition
          << "] on [" << subscribeTopic << "] before activating.\n";
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
        << outputTopic << "].\n";
}

void Distance_SensorSystem::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) {
  if (_info.paused || !this->configured) {
    return;
  }

  const std::lock_guard<std::mutex> lock(this->activationMutex);
  if (!this->isActivated || this->belowThresholdPublished) {
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
    this->aboveThresholdPublished = false;
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
    if (this->aboveThresholdPublished) {
      return;
    }
  }

  gz::msgs::StringMsg message;
  message.set_data(belowThreshold ? this->belowThresholdMessage
                                  : this->aboveThresholdMessage);
  if (!this->publisher.Publish(message)) {
    gzerr << "[Distance_SensorSystem] Failed to publish threshold state.\n";
    return;
  }

  if (belowThreshold) {
    this->belowThresholdPublished = true;
  } else {
    this->aboveThresholdPublished = true;
  }
  gzmsg << "[Distance_SensorSystem] Distance " << distance << " m is "
        << (belowThreshold ? "below" : "at or above")
        << " threshold; published [" << message.data() << "].\n";
}

void Distance_SensorSystem::OnTriggerMessage(
    const gz::msgs::StringMsg &_message) {
  const std::lock_guard<std::mutex> lock(this->activationMutex);
  const bool activate = _message.data() == this->triggerCondition;

  if (activate && !this->isActivated) {
    this->belowThresholdPublished = false;
    this->aboveThresholdPublished = false;
    this->belowThresholdSince.reset();
  } else if (!activate) {
    this->belowThresholdPublished = false;
    this->aboveThresholdPublished = false;
    this->belowThresholdSince.reset();
  }
  this->isActivated = activate;
}

} // namespace Distance_Sensor

GZ_ADD_PLUGIN(Distance_Sensor::Distance_SensorSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(Distance_Sensor::Distance_SensorSystem,
                    "Distance_Sensor::Distance_SensorSystem")
