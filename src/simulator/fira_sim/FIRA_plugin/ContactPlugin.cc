#include "ContactPlugin.hh"
#include "std_msgs/String.h"


using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{ 
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{   
    // Get the parent sensor.
    this->parentSensor =
    boost::shared_dynamic_cast<sensors::ContactSensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
        gzerr << "ContactPlugin requires a ContactSensor.\n";
        return;
    }

    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(
      boost::bind(&ContactPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);

    std::cout << "in Contact Load()" << std::endl;

    ros::NodeHandle n;
    chatter_pub = n.advertise<std_msgs::String>("Last_TouchBall", 1000);

}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
    // Get all the contacts.
    msgs::Contacts contacts;
    contacts = this->parentSensor->GetContacts();
    for (unsigned int i = 0; i < contacts.contact_size(); ++i)
    {
        if(contacts.contact(i).collision2()=="FIRA_Ground::link::collision"){
            continue;
        }

//    std::cout << "Collision between[" << contacts.contact(i).collision1()
//              << "] and [" << contacts.contact(i).collision2() << "]\n";


    if(contacts.contact(i).collision2().find("youbot0")!=std::string::npos)
        Touch_player = "youbot0";
    else if(contacts.contact(i).collision2().find("youbot1")!=std::string::npos)
        Touch_player = "youbot1";
    else if(contacts.contact(i).collision2().find("youbot2")!=std::string::npos)
        Touch_player = "youbot2";
    else if(contacts.contact(i).collision2().find("youbot3")!=std::string::npos)
        Touch_player = "youbot3";
    else if(contacts.contact(i).collision2().find("youbot4")!=std::string::npos)
        Touch_player = "youbot4";
    else if(contacts.contact(i).collision2().find("youbot5")!=std::string::npos)
        Touch_player = "youbot5";


//    if(contacts.contact(i).collision2()=="youbot0::base_footprint::base_footprint_collision_base_link")
//        Touch_player = "youbot0";
//    else if(contacts.contact(i).collision2()=="youbot1::base_footprint::base_footprint_collision_base_link")
//        Touch_player = "youbot1";
//    else if(contacts.contact(i).collision2()=="youbot2::base_footprint::base_footprint_collision_base_link")
//        Touch_player = "youbot2";
//    else if(contacts.contact(i).collision2()=="youbot3::base_footprint::base_footprint_collision_base_link")
//        Touch_player = "youbot3";
//    else if(contacts.contact(i).collision2()=="youbot4::base_footprint::base_footprint_collision_base_link")
//        Touch_player = "youbot4";
//    else if(contacts.contact(i).collision2()=="youbot5::base_footprint::base_footprint_collision_base_link")
//        Touch_player = "youbot5";

    if(Touch_player_new != Touch_player){
        Touch_player_new = Touch_player;
        sendstr();
    }

//    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
//    {
//      std::cout << j << "  Position:"
//                << contacts.contact(i).position(j).x() << " "
//                << contacts.contact(i).position(j).y() << " "
//                << contacts.contact(i).position(j).z() << "\n";
//      std::cout << "   Normal:"
//                << contacts.contact(i).normal(j).x() << " "
//                << contacts.contact(i).normal(j).y() << " "
//                << contacts.contact(i).normal(j).z() << "\n";
//      std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
//    }
  }
}

void ContactPlugin::sendstr(){

    std_msgs::String msg;

    msg.data =Touch_player_new;
    chatter_pub.publish(msg);
    ROS_INFO("%s", msg.data.c_str());

}
