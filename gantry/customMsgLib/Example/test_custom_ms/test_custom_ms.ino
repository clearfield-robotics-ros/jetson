#include <ros.h>
#include <ros/time.h>
#include <gantry/to_gantry_msg.h>
#include <gantry/gantry_status.h>

void messageCb(const gantry::to_gantry_msg& gantry_cmd)
{
  //do something here
}

ros::NodeHandle nh;
//gantry::to_gantry_msg to_gantry_msg; //this is subscribed
gantry::gantry_status gantry_status;
ros::Publisher gantryStatus("gantry_current_status", &gantry_status);
ros::Subscriber<gantry::to_gantry_msg> sub("gantry_cmd", messageCb);

void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
