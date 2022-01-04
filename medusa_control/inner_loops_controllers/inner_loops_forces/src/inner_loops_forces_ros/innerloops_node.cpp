#include <ros/ros.h>
#include "innerloops.h"
#include "thruster_allocation.h"
#include "safeties.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "innerloops");
  ros::NodeHandle nh("~");

  Innerloops con(nh);
  ThrustAllocation thr(nh);
  Safeties sft(nh);

  ros::spin();
}
