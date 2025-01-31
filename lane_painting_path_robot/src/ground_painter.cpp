#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/common/common.hh>

namespace gazebo {
    class GroundPainter : public ModelPlugin {
        public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
            // Store the pointer to the model
            this->model = _model;

            // Listen to the update event
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&GroundPainter::OnUpdate, this));

            // Get the world
            this->world = this->model->GetWorld();

            // Get the visual for ground_plane
            this->visual = gazebo::rendering::get_scene()->GetVisual("ground_plane");
            if (!this->visual) {
                gzerr << "Could not find ground_plane visual\n";
                return;
            }

            // Create red material
            Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
                "red_paint", "General");
            material->getTechnique(0)->getPass(0)->setDiffuse(1.0, 0.0, 0.0, 1.0);
            material->getTechnique(0)->getPass(0)->setAmbient(1.0, 0.0, 0.0);
            material->getTechnique(0)->getPass(0)->setSelfIllumination(1.0, 0.0, 0.0);
        }

        public: void OnUpdate() {
            if (!this->visual) return;

            // Get robot position
            ignition::math::Pose3d pose = this->model->WorldPose();
            
            // Create a red circle at robot's position
            this->visual->SetMaterial("red_paint");
            
            // Update position
            ignition::math::Vector3d pos = pose.Pos();
            pos.Z() = 0.001;  // Just above ground
            this->visual->SetPosition(pos);
        }

        private:
            physics::ModelPtr model;
            physics::WorldPtr world;
            rendering::VisualPtr visual;
            event::ConnectionPtr updateConnection;
    };

    GZ_REGISTER_MODEL_PLUGIN(GroundPainter)
}
