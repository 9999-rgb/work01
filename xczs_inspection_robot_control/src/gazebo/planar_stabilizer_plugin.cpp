#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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
    load_held_joints(sdf);
    held_joint_positions_.resize(held_joints_.size(), 0.0);
    schedule_height_lock();
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateEnd(
      std::bind(&PlanarStabilizerPlugin::stabilize, this));
    joint_snapshot_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(
        &PlanarStabilizerPlugin::capture_joint_positions,
        this,
        std::placeholders::_1));
  }

  void Reset() override
  {
    height_is_locked_ = false;
    has_joint_snapshot_ = false;
    schedule_height_lock();
  }

private:
  void load_held_joints(const sdf::ElementPtr & sdf)
  {
    if (!sdf->HasElement("hold_joint")) {
      return;
    }

    sdf::ElementPtr joint_element = sdf->GetElement("hold_joint");
    while (joint_element) {
      const std::string joint_name = joint_element->Get<std::string>();
      const gazebo::physics::JointPtr joint = model_->GetJoint(joint_name);
      if (joint) {
        held_joints_.push_back(joint);
      } else {
        gzerr << "Planar stabilizer could not find joint ["
              << joint_name << "].\n";
      }
      joint_element = joint_element->GetNextElement("hold_joint");
    }
  }

  void schedule_height_lock()
  {
    height_lock_sim_time_ =
      model_->GetWorld()->SimTime().Double() + settling_duration_;
  }

  void capture_joint_positions(const gazebo::common::UpdateInfo &)
  {
    for (std::size_t index = 0; index < held_joints_.size(); ++index) {
      held_joint_positions_[index] = held_joints_[index]->Position(0);
    }
    has_joint_snapshot_ = true;
  }

  void restore_joint_positions()
  {
    if (!has_joint_snapshot_) {
      return;
    }

    for (std::size_t index = 0; index < held_joints_.size(); ++index) {
      held_joints_[index]->SetPosition(
        0, held_joint_positions_[index], true);
      held_joints_[index]->SetVelocity(0, 0.0);
    }
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

    // The planar mover changes the base velocity without holding child
    // joints. Restore the positions captured after trajectory processing so
    // the arm follows the chassis while explicit joint commands still work.
    restore_joint_positions();
  }

  gazebo::physics::ModelPtr model_;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo::event::ConnectionPtr joint_snapshot_connection_;
  std::vector<gazebo::physics::JointPtr> held_joints_;
  std::vector<double> held_joint_positions_;
  double settling_duration_{0.5};
  double height_lock_sim_time_{0.0};
  double locked_height_{0.0};
  bool height_is_locked_{false};
  bool has_joint_snapshot_{false};
};

GZ_REGISTER_MODEL_PLUGIN(PlanarStabilizerPlugin)

}  // namespace xczs_inspection_robot_control
