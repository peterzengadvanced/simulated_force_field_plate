#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <iostream>
#include <string>

namespace gazebo
{
  class MagnetForcePlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      // Store pointers
      this->model = _model;
      this->world = _model->GetWorld();

      // Optional SDF parameters
      if (_sdf->HasElement("magnet_link_name"))
        this->magnetLinkName = _sdf->Get<std::string>("magnet_link_name");
      if (_sdf->HasElement("magnet_range"))
        this->magnetRange = _sdf->Get<double>("magnet_range");
      if (_sdf->HasElement("magnet_strength"))
        this->magnetStrength = _sdf->Get<double>("magnet_strength");

      // Get the magnet link pointer
      this->magnetLink = this->model->GetLink(this->magnetLinkName);
      if (!this->magnetLink)
      {
        gzerr << "[MagnetForcePlugin] Could not find link '"
              << this->magnetLinkName << "' in model '"
              << this->model->GetName() << "'.\n";
        return;
      }

      // Connect to the world update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MagnetForcePlugin::OnUpdate, this));

      gzdbg << "[MagnetForcePlugin] Loaded with range=" << this->magnetRange
            << ", strength=" << this->magnetStrength << "\n";
    }

    /// Called at every simulation iteration
    void OnUpdate()
    {
      // Get the world pose of the magnet link
      ignition::math::Pose3d magnetPose = this->magnetLink->WorldPose();
      ignition::math::Vector3d magnetPos = magnetPose.Pos();

      // Iterate over all models in the world
      auto models = this->world->Models();
      for (auto &m : models)
      {
        // Skip our own magnet model
        if (m->GetName() == this->model->GetName())
          continue;

        // Get all links of this model
        auto links = m->GetLinks();
        for (auto &link : links)
        {
          // If the link is valid
          if (link)
          {
            // Compute distance from magnet
            ignition::math::Vector3d linkPos = link->WorldPose().Pos();
            ignition::math::Vector3d diff = magnetPos - linkPos;
            double dist = diff.Length();

            // If within magnet range
            if (dist > 0.0 && dist < this->magnetRange)
            {
              // method 1: inverse-square pulling force:
              // Force magnitude = magnet_strength / dist^2
              // Or might can do something simpler like magnet_strength*(range-dist)
              double forceMag = this->magnetStrength / (dist * dist);

              // Direction from link to magnet
              ignition::math::Vector3d direction = diff.Normalized();

              // Final force vector
              ignition::math::Vector3d forceVec = direction * forceMag;

              // Apply the force to the link's center of mass
              link->AddForce(forceVec);
            }
          }
        }
      }
    }

  private:
    // Pointer to the magnet model, world, link
    physics::ModelPtr model;
    physics::WorldPtr world;
    physics::LinkPtr magnetLink;

    // Parameters
    std::string magnetLinkName{"magnet_link"};
    double magnetRange{2.0};       // meters
    double magnetStrength{10.0};   // arbitrary units

    // Connection to world update
    event::ConnectionPtr updateConnection;
  };

  // Register plugin
  GZ_REGISTER_MODEL_PLUGIN(MagnetForcePlugin)
}
