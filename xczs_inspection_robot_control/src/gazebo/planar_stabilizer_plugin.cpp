#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <cmath>
#include <functional>
#include <memory>
#include <utility>

namespace xczs_inspection_robot_control
{

class PlanarStabilizerPlugin final : public gazebo::ModelPlugin
{
public:
  void Load(
    gazebo::physics::ModelPtr model,
    sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    settling_duration_ =
      sdf->Get<double>("settling_duration", 0.5).first;
    schedule_height_lock();
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateEnd(
      std::bind(&PlanarStabilizerPlugin::stabilize, this));
  }

  void Reset() override
  {
    height_is_locked_ = false;
    schedule_height_lock();
  }

private:
  void schedule_height_lock()
  {
    height_lock_sim_time_ =
      model_->GetWorld()->SimTime().Double() + settling_duration_;
  }

  void stabilize()
  {
    constexpr double tilt_tolerance = 1.0e-9;
    constexpr double height_tolerance = 1.0e-9;

    const ignition::math::Pose3d world_pose = model_->WorldPose();
    const double roll = world_pose.Rot().Roll();
    const double pitch = world_pose.Rot().Pitch();
    const double sim_time = model_->GetWorld()->SimTime().Double();

    if (!height_is_locked_ && sim_time >= height_lock_sim_time_) {
      locked_height_ = world_pose.Pos().Z();
      height_is_locked_ = true;
    }

    const bool has_tilt =
      std::abs(roll) > tilt_tolerance ||
      std::abs(pitch) > tilt_tolerance;
    const bool has_height_error =
      height_is_locked_ &&
      std::abs(world_pose.Pos().Z() - locked_height_) >
      height_tolerance;

    if (has_tilt || has_height_error) {
      ignition::math::Vector3d planar_position = world_pose.Pos();
      if (height_is_locked_) {
        planar_position.Z(locked_height_);
      }
      const ignition::math::Quaterniond planar_rotation(
        0.0, 0.0, world_pose.Rot().Yaw());
      model_->SetWorldPose(
        ignition::math::Pose3d(planar_position, planar_rotation),
        true,
        false);
    }

    if (height_is_locked_) {
      ignition::math::Vector3d linear_velocity =
        model_->WorldLinearVel();
      linear_velocity.Z(0.0);
      model_->SetLinearVel(linear_velocity);
    }

    ignition::math::Vector3d angular_velocity =
      model_->WorldAngularVel();
    angular_velocity.X(0.0);
    angular_velocity.Y(0.0);
    model_->SetAngularVel(angular_velocity);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::event::ConnectionPtr update_connection_;
  double settling_duration_{0.5};
  double height_lock_sim_time_{0.0};
  double locked_height_{0.0};
  bool height_is_locked_{false};
};

GZ_REGISTER_MODEL_PLUGIN(PlanarStabilizerPlugin)

}  // namespace xczs_inspection_robot_control
