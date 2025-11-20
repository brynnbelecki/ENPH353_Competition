#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <memory>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class SimpleBodyDroneControlPlugin : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
        this->model = _model;

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "simple_body_drone_plugin",
                      ros::init_options::NoSigintHandler);
        }

        // Namespace
        std::string ns = "";
        if (_sdf->HasElement("namespace"))
            ns = _sdf->Get<std::string>("namespace");

        this->nh.reset(new ros::NodeHandle(ns));

        // Topics
        std::string linear_topic  = "drone_linear_cmd";
        std::string angular_topic = "drone_angular_cmd";

        this->subLinear = this->nh->subscribe(linear_topic, 10,
                                              &SimpleBodyDroneControlPlugin::OnLinearCommand, this);
        this->subAngular = this->nh->subscribe(angular_topic, 10,
                                               &SimpleBodyDroneControlPlugin::OnAngularCommand, this);

        ROS_INFO_STREAM("Subscribed to " << this->nh->getNamespace() << "/" << linear_topic);
        ROS_INFO_STREAM("Subscribed to " << this->nh->getNamespace() << "/" << angular_topic);

        this->rosQueueThread = std::thread([this]() {
            ros::Rate rate(100);
            while (ros::ok())
            {
                ros::spinOnce();
                rate.sleep();
            }
        });

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&SimpleBodyDroneControlPlugin::OnUpdate, this));
    }

    ~SimpleBodyDroneControlPlugin()
    {
        if (this->rosQueueThread.joinable())
            this->rosQueueThread.join();
    }

private:
    void OnLinearCommand(const geometry_msgs::Twist::ConstPtr &msg)
    {
        this->cmdForce.Set(msg->linear.x, msg->linear.y, msg->linear.z);
    }

    void OnAngularCommand(const geometry_msgs::Twist::ConstPtr &msg)
    {
        this->cmdTorque.Set(msg->angular.x, msg->angular.y, msg->angular.z);
    }

    void OnUpdate()
    {
        if (!this->model)
            return;

        auto link = this->model->GetLink("chassis");
        if (link)
        {
            link->AddForce(this->cmdForce);
            link->SetAngularVel(this->cmdTorque);
        }
    }

    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Subscriber subLinear;
    ros::Subscriber subAngular;
    std::thread rosQueueThread;

    ignition::math::Vector3d cmdForce{0, 0, 0};
    ignition::math::Vector3d cmdTorque{0, 0, 0};
};

GZ_REGISTER_MODEL_PLUGIN(SimpleBodyDroneControlPlugin)
}  // namespace gazebo
