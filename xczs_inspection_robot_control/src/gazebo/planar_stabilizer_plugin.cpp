#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace xczs_inspection_robot_control
{

class PlanarStabilizerPlugin final : public gazebo::ModelPlugin
{
public:
  ~PlanarStabilizerPlugin() override
  {
    const auto callback_lifetime = callback_lifetime_;
    if (callback_lifetime) {
      std::lock_guard<std::mutex> lock(callback_lifetime->mutex);
      callback_lifetime->shutting_down = true;
      callback_lifetime->owner = nullptr;
    }

    // Stop both world-update sources before any Gazebo object is released.
    // A callback that already holds a lease is allowed to finish below.
    joint_snapshot_connection_.reset();
    update_connection_.reset();
    grasp_active_subscription_.reset();

    if (callback_lifetime) {
      std::unique_lock<std::mutex> lock(callback_lifetime->mutex);
      callback_lifetime->condition.wait(
        lock, [&callback_lifetime]() {
          return callback_lifetime->active_callbacks == 0U;
        });
    }
  }

  void Load(
    gazebo::physics::ModelPtr model,
    sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    ros_node_ = gazebo_ros::Node::Get(sdf);
    if (!ros_node_) {
      gzerr << "Planar stabilizer could not create a ROS 2 node.\n";
      return;
    }
    const auto grasp_active_topic = sdf->Get<std::string>(
      "grasp_active_topic", "/xczs/cabinet/grasp_active").first;
    if (grasp_active_topic.empty() || grasp_active_topic.front() != '/') {
      gzerr << "Planar stabilizer grasp_active_topic must be absolute.\n";
      return;
    }
    const auto grasp_active = grasp_active_;
    grasp_active_subscription_ =
      ros_node_->create_subscription<std_msgs::msg::Bool>(
      grasp_active_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      [grasp_active](const std_msgs::msg::Bool::SharedPtr message) {
        grasp_active->store(message->data);
      });
    settling_duration_ =
      sdf->Get<double>("settling_duration", 0.5).first;
    load_held_joints(sdf);
    held_joint_positions_.resize(held_joints_.size(), 0.0);
    schedule_height_lock();

    callback_lifetime_ = std::make_shared<UpdateCallbackLifetime>();
    callback_lifetime_->owner = this;
    const auto callback_lifetime = callback_lifetime_;
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateEnd(
      [callback_lifetime]() {
        auto * owner = acquire_update_callback(callback_lifetime);
        if (!owner) {
          return;
        }
        UpdateCallbackLease lease(callback_lifetime);
        std::lock_guard<std::mutex> lock(owner->update_mutex_);
        owner->stabilize();
      });
    joint_snapshot_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin(
      [callback_lifetime](const gazebo::common::UpdateInfo & update_info) {
        auto * owner = acquire_update_callback(callback_lifetime);
        if (!owner) {
          return;
        }
        UpdateCallbackLease lease(callback_lifetime);
        std::lock_guard<std::mutex> lock(owner->update_mutex_);
        owner->capture_joint_positions(update_info);
      });
  }

  void Reset() override
  {
    std::lock_guard<std::mutex> lock(update_mutex_);
    height_is_locked_ = false;
    has_joint_snapshot_ = false;
    schedule_height_lock();
  }

private:
  struct UpdateCallbackLifetime
  {
    std::mutex mutex;
    std::condition_variable condition;
    PlanarStabilizerPlugin * owner{nullptr};
    std::size_t active_callbacks{0U};
    bool shutting_down{false};
  };

  class UpdateCallbackLease
  {
  public:
    explicit UpdateCallbackLease(
      std::shared_ptr<UpdateCallbackLifetime> lifetime)
    : lifetime_(std::move(lifetime)) {}

    UpdateCallbackLease(const UpdateCallbackLease &) = delete;
    UpdateCallbackLease & operator=(const UpdateCallbackLease &) = delete;

    ~UpdateCallbackLease()
    {
      std::lock_guard<std::mutex> lock(lifetime_->mutex);
      if (lifetime_->active_callbacks > 0U) {
        --lifetime_->active_callbacks;
      }
      lifetime_->condition.notify_all();
    }

  private:
    std::shared_ptr<UpdateCallbackLifetime> lifetime_;
  };

  static PlanarStabilizerPlugin * acquire_update_callback(
    const std::shared_ptr<UpdateCallbackLifetime> & lifetime)
  {
    std::lock_guard<std::mutex> lock(lifetime->mutex);
    if (lifetime->shutting_down || !lifetime->owner) {
      return nullptr;
    }
    ++lifetime->active_callbacks;
    return lifetime->owner;
  }

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
    if (!grasp_active_->load()) {
      restore_joint_positions();
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    grasp_active_subscription_;
  std::shared_ptr<std::atomic<bool>> grasp_active_{
    std::make_shared<std::atomic<bool>>(false)};
  gazebo::event::ConnectionPtr update_connection_;
  gazebo::event::ConnectionPtr joint_snapshot_connection_;
  std::shared_ptr<UpdateCallbackLifetime> callback_lifetime_;
  std::mutex update_mutex_;
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
