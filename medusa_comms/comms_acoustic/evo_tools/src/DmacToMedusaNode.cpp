/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
*/
// this header incorporates all the necessary #include files and defines the class "DmacToMedusaNode"
#include "DmacToMedusaNode.h"

/*
#######################################################################################################################
@.@ CONSTRUCTOR: put all dirty work of initializations here
Note the odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
#######################################################################################################################
*/
DmacToMedusaNode::DmacToMedusaNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private) : nh_(*nodehandle), nh_private_(*nodehandle_private)
{
    ROS_INFO("in class constructor of DmacToMedusaNode");
    loadParams();
    buildUSBLRotationMatrix();
    initializeSubscribers();
    initializePublishers();
}

/*
#######################################################################################################################
@.@ Destructor
#######################################################################################################################
*/
DmacToMedusaNode::~DmacToMedusaNode()
{

    // +.+ shutdown publishers
    // ---> add publishers here
    // Example: uref_pub.shutdown();
    usbl_fix_medusa_pub_.shutdown();

    // +.+ shutdown subscribers
    // ---> add subscribers here
    // Example: state_sub.shutdown();
    usbl_fix_dmac_sub_.shutdown();

    // +.+ shutdown node
    nh_.shutdown();
    nh_private_.shutdown();
}

/*
#######################################################################################################################
 @.@ Member Helper function to set up subscribers;
 note odd syntax: &DmacToMedusaNode::subscriberCallback is a pointer to a member function of DmacToMedusaNode
 "this" keyword is required, to refer to the current instance of DmacToMedusaNode
 #######################################################################################################################
 */
void DmacToMedusaNode::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers for DmacToMedusaNode");
    // ---> add subscribers here
    // Example: state_sub = nh_.subscribe("State", 10, &DmacToMedusaNode::updateCallback,this);

    usbl_fix_dmac_sub_ = nh_.subscribe(p_usbl_fix_dmac_topic_, 10, &DmacToMedusaNode::fixCallback, this);
    state_sub_ = nh_.subscribe(MedusaGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/state", "/nav/filter/state"), 1, &DmacToMedusaNode::stateCallback, this);
}

/*
#######################################################################################################################
 @.@ Member helper function to set up publishers;
 #######################################################################################################################
 */
void DmacToMedusaNode::initializePublishers()
{
    ROS_INFO("Initializing Publishers for DmacToMedusaNode"); // ---> add publishers here
                                                              //Example: uref_pub = nh_.advertise<std_medusa_usbl_fix_msgs::Float64>("URef", 10); //Surge Reference

    usbl_fix_medusa_pub_ = nh_.advertise<medusa_msgs::mUSBLFix>(p_usbl_fix_medusa_topic_, 1);
}

/*
#######################################################################################################################
 @.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate
 #######################################################################################################################
 */

/*
#######################################################################################################################
 @.@ Load the parameters
 #######################################################################################################################
 */
void DmacToMedusaNode::loadParams()
{
    ROS_INFO("Load the DmacToMedusaNode parameters");
    //---> params here, always p_paramName

    // +.+ Frame parameters
    //p_destination_frame_ = MedusaGimmicks::getParameters<std::string>(nh_private_, "destination_frame");
    // +.+ Topic in
    p_usbl_fix_dmac_topic_ = MedusaGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/dmac_fix");
    // +.+ Topic out
    p_usbl_fix_medusa_topic_ = MedusaGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/medusa_fix");

    p_real_ = MedusaGimmicks::getParameters<bool>(nh_private_, "real", false);
    
    p_fix_type_ = MedusaGimmicks::getParameters<bool>(nh_private_, "fix_type", false);

    p_installation_matrix_ = MedusaGimmicks::getParameters<std::vector<double>>(nh_private_, "installation_matrix");

}

/*
#######################################################################################################################
@.@ Helper method for rotating message from sensor frame to base_link(emo) or base_pose(real) frames
    Note: messages passed by reference
#######################################################################################################################
*/
void DmacToMedusaNode::buildUSBLRotationMatrix(){

  if (p_fix_type_ == false){
  usbl_rot_matrix_ << std::sin(p_installation_matrix_[0]), std::cos(p_installation_matrix_[0]), 0,
                      std::cos(p_installation_matrix_[0]), -std::sin(p_installation_matrix_[0]), 0,
                      0, 0, std::cos(p_installation_matrix_[2]);
}
else{
  usbl_rot_matrix_ << std::sin(p_installation_matrix_[0]), -std::cos(p_installation_matrix_[0]), 0,
                      std::cos(p_installation_matrix_[0]), std::sin(p_installation_matrix_[0]), 0,
                      0, 0, std::cos(p_installation_matrix_[2]);
}
}
/*
#######################################################################################################################
@.@ Callbacks Section / Methods
#######################################################################################################################
*/

/*
#######################################################################################################################
@.@ Callback fix usbl dmac -> convert usbl fix dmac message into usbl fix medusa dmac message
#######################################################################################################################
*/

void DmacToMedusaNode::fixCallback(const dmac::mUSBLFix &dmac_usbl_fix_msg)
{
    // +.+ Start building the medusa usbl fix message
    medusa_msgs::mUSBLFix medusa_usbl_fix_msg;

    medusa_usbl_fix_msg.header = dmac_usbl_fix_msg.header;
    medusa_usbl_fix_msg.type = dmac_usbl_fix_msg.type;

    medusa_usbl_fix_msg.source_id = dmac_usbl_fix_msg.source_id;
    medusa_usbl_fix_msg.source_name = dmac_usbl_fix_msg.source_name;
    medusa_usbl_fix_msg.source_frame_id = "vehicle" + dmac_usbl_fix_msg.source_name;

    medusa_usbl_fix_msg.bearing_raw = dmac_usbl_fix_msg.bearing_raw;
    medusa_usbl_fix_msg.elevation_raw = dmac_usbl_fix_msg.elevation_raw;

    // +.+ If message is range only or full fix grab the range value from dmac message
    if (medusa_usbl_fix_msg.type == medusa_usbl_fix_msg.RANGE_ONLY || medusa_usbl_fix_msg.type == medusa_usbl_fix_msg.FULL_FIX)
    {
        medusa_usbl_fix_msg.range = dmac_usbl_fix_msg.range;
    }

    // If message is
    if (medusa_usbl_fix_msg.type == medusa_usbl_fix_msg.AZIMUTH_ONLY || medusa_usbl_fix_msg.type == medusa_usbl_fix_msg.FULL_FIX)
    {   
        
      if (p_real_){
        //std::cout << "Here" << "\n"; 
        //// sphere to cartesian usbl, assuming a range
        double x_rel, y_rel, z_rel, range_r=10.0;
        x_rel = cos(dmac_usbl_fix_msg.bearing_raw)*range_r*cos(dmac_usbl_fix_msg.elevation_raw);
        y_rel = sin(dmac_usbl_fix_msg.bearing_raw)*range_r*cos(dmac_usbl_fix_msg.elevation_raw);
        z_rel = range_r*sin(dmac_usbl_fix_msg.elevation_raw);

        // Rotate Vector to body
        Eigen::MatrixXd point(3,1);
        point << x_rel, y_rel, z_rel;
        Eigen::MatrixXd pt_rot;
        pt_rot = usbl_rot_matrix_*point;
      
      
        // Rotate Vector to interial
        body_rot_matrix_ << cos(yaw_state_), -sin(yaw_state_), 0,
                           sin(yaw_state_), cos(yaw_state_), 0,
                           0,0,1;
      
        pt_rot = body_rot_matrix_ * pt_rot;

        // cartesian to sphere
       medusa_usbl_fix_msg.bearing  = std::atan2(pt_rot(1),pt_rot(0));
       medusa_usbl_fix_msg.elevation = std::atan2(pt_rot(2),std::sqrt(std::pow(pt_rot(0),2) + std::pow(pt_rot(1),2)));

      }
      else{
        medusa_usbl_fix_msg.bearing = dmac_usbl_fix_msg.bearing_raw;
        medusa_usbl_fix_msg.elevation = dmac_usbl_fix_msg.elevation_raw;
      }
    }
    // +.+ Publishing the new message medusa usbl fix message
    usbl_fix_medusa_pub_.publish(medusa_usbl_fix_msg);
}


void DmacToMedusaNode::stateCallback(const auv_msgs::NavigationStatus &msg){
    yaw_state_ = (msg.orientation.z/180) * MedusaGimmicks::PI;
}
/*
#######################################################################################################################
 @.@ Main
 #######################################################################################################################
 */
int main(int argc, char **argv)
{
    // +.+ ROS set-ups:
    ros::init(argc, argv, "acoustic_converters_node"); //node name
    // +.+ create a node handle; need to pass this to the class constructor
    ros::NodeHandle nh, nh_p("~");

    ROS_INFO("main: instantiating an object of type DmacToMedusaNode");

    // +.+ instantiate an DmacToMedusaNode class object and pass in pointer to nodehandle for constructor to use
    DmacToMedusaNode DmacToMedusa(&nh, &nh_p);

    // +.+ Added to work with timer -> going into spin; let the callbacks do all the work
    ros::spin();

    return 0;
}
