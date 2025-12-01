#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <thread>
#include <memory>
#include <map>
#include <sstream>
#include <mutex>

namespace gazebo
{
class ClueboardHandler : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
        std::cout << "[ClueboardHandler] Starting plugin Load()" << std::endl;
        this->model = _model;
        ROS_INFO("Initializing clueboard handler!");

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "clueboard_handler",
                      ros::init_options::NoSigintHandler);       
        }
        ROS_INFO("Initializing clueboard handler2!");

        // Namespace
        std::string ns = "";
        if (_sdf->HasElement("namespace"))
            ns = _sdf->Get<std::string>("namespace");

        this->nh.reset(new ros::NodeHandle(ns));

        // Topics
        std::string clueboard_status  = "clueboard_status";
        std::string clueboard_message = "clueboard_message";
        std::string clueboard_message_in = "clueboard_message_in";

        this->InitBoards();

        this->pubStatus = this->nh->advertise<std_msgs::String>(clueboard_status, 10);

        this->pubMessage = this->nh->advertise<std_msgs::String>(clueboard_message, 10);

        this->subMessage = this->nh->subscribe(clueboard_message_in, 10,
                                               &ClueboardHandler::OnMessageSend, this);

        ROS_INFO_STREAM("Published to " << this->nh->getNamespace() << "/" << clueboard_status);
        ROS_INFO_STREAM("Published to " << this->nh->getNamespace() << "/" << clueboard_message);
        ROS_INFO_STREAM("Published to " << this->nh->getNamespace() << "/" << clueboard_message_in);

        this->rosQueueThread = std::thread([this]() {
            ros::Rate rate(100);
            while (ros::ok())
            {
                ros::spinOnce();
                rate.sleep();
            }
        });

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ClueboardHandler::OnUpdate, this));
        
    }

    ~ClueboardHandler()
    {
        if (this->rosQueueThread.joinable())
            this->rosQueueThread.join();
    }

private:
    std::map<std::string, std::string> clues; //BUG
    std::mutex clues_mutex; //prevent race condition on clues access

    void InitBoards()
    {
        clues["SIZE"] = "";
        clues["VICTIM"] = "";
        clues["CRIME"] = "";
        clues["TIME"] = "";
        clues["PLACE"] = "";
        clues["MOTIVE"] = "";
        clues["WEAPON"] = "";
        clues["BANDIT"] = "";
    }

    void OnMessageSend(const std_msgs::String::ConstPtr &msg)
    {
        std_msgs::String out;
        out.data = "test"; 
        pubMessage.publish(out);
    }

    void OnUpdate()
    {
        ROS_INFO_THROTTLE(1, "Publishing clueboard topics...");
        std::stringstream ss_clue;
        {
            std::lock_guard<std::mutex> lock(clues_mutex);
            for (auto &pair : clues)
                ss_clue << pair.first << ":" << pair.second << ";";
        }

        std_msgs::String clue_msg;
        clue_msg.data = ss_clue.str();
        pubMessage.publish(clue_msg);

        std::map<std::string, std::string> status;
        {
            std::lock_guard<std::mutex> lock(clues_mutex);
            for (auto &pair : clues)
                status[pair.first] = pair.second.empty() ? "False" : "True";
        }

        std::stringstream ss_status;
        for (auto &pair : status)
            ss_status << pair.first << ":" << pair.second << ";";

        std_msgs::String status_msg;
        status_msg.data = ss_status.str();
        pubStatus.publish(status_msg);
    }

    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Publisher pubStatus;
    ros::Publisher pubMessage;
    ros::Subscriber subMessage;
    std::thread rosQueueThread;
};

GZ_REGISTER_MODEL_PLUGIN(ClueboardHandler)
}  // namespace gazebo
