#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <medusa_msgs/mUSBLFix.h>
#include <nav_msgs/Odometry.h>
#include <medusa_msgs/mState.h>

#include <signal.h>
/*INCLUDE SOCKET HEADERS*/
#include <sys/types.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

double RATE, watchdog_time, wifi_timeout, lastDepth;
std::string surface_vehicle, vehicle_name;
std_msgs::String lastRoles;
ros::Time lastping, lastRoles_time;

bool acoustic_abort;

int id_sck, sckClient, SAFETY_PORT;
bool Client, RELAY_WIFI;

// Non blocking accept
fd_set master;
fd_set read_fds;
int fdmax;/* maximum file descriptor number */

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

/**
 * @brief replacement SIGINT handler
 * 
 * @param sig 
 */
void mySigIntHandler(int sig)
{
  ROS_INFO("Shutdown Signal received!");
  g_request_shutdown = 1;
  close(sckClient);
  exit(0);
}

/**
 * @brief Creates a TCP server with a specified port
 * 
 * @param port TCP port 
 * @return int >0 open port with success. Otherwise failed to open port 
 */
int MakeServer(int port){
  int sck;

  /* clear the master and temp sets */
  FD_ZERO(&master);
  FD_ZERO(&read_fds);

  struct sockaddr_in si_me;

  if ((sck=socket(AF_INET,SOCK_STREAM,0))<0){
    ROS_ERROR("Creating socket");
    return -1;
  }

  int optval = 1;
  // To reuse the socket event if it is binded
  setsockopt(sck, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);

  memset((char *) &si_me, 0, sizeof(si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(sck,(struct sockaddr *) &si_me, sizeof(si_me))==-1){
    ROS_ERROR("Port already in use");
    return -1;
  }
  /* socket waits for a connection*/
  listen(sck,1);
  /* add the listener to the master set */
  FD_SET(sck, &master);
  /* keep track of the biggest file descriptor */
  fdmax = sck;

  return sck;
}

/**
 * @brief Accepts any client that trys to connect
 * 
 * @param sockfd 
 * @return int 
 */
int AcceptClient(int sockfd){
  struct sockaddr_in cli_addr;
  int  newsockfd, clilen;
  clilen = sizeof(cli_addr);

  read_fds = master;

  struct timeval tv = {2, 0};   // sleep for 2.0s
  if(select(fdmax+1, &read_fds, NULL, NULL, &tv) == -1)
  {
    ROS_ERROR("Server-select() error!");
    return -1;
  }
  if(FD_ISSET(fdmax, &read_fds)){
    newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, (socklen_t *) &clilen);
    if (newsockfd <= 0){
      if(errno == 4){
        ROS_INFO("SIGINT recieved in accept\n");
        ros::shutdown();
        exit(0);
      }else{
        ROS_ERROR("Accepting a Client");
        return -1;
      }
    }
  }else{
    return -1;
  }
  return newsockfd;
}

/*
 * Reads the message and verifies if it comes correctly or not
 * Waits for 1 second to receive this message
 */

/**
 * @brief Reads the message and verifies if it comes correctly or not
 *        Waits for 1 second to receive this message
 * @param sck 
 * @return true 
 * @return false 
 */
bool ReadACK(int sck){
  static fd_set readfds;
  int rv,n, dim;
  struct timeval tvselect;
  char ACK[3];

  // Waits for 1 second to receive the ACK
  tvselect.tv_sec=1;
  tvselect.tv_usec=0;
  FD_ZERO(&readfds);
  FD_SET(sck,  &readfds);
  n = sck + 1;
  rv = select(n, &readfds, NULL, NULL, &tvselect);

  if (rv == -1){
    ROS_ERROR("select reading data port"); /*error occurred in select()*/
    return false;
  }else if (rv == 0){
    ROS_WARN("Timeout occurred! No ACK in 1 second.\n");
    return false;
  }else
    if (!FD_ISSET(sck, &readfds))
      return false;

  dim=read(sck, ACK, sizeof(unsigned char));
  if(dim==0){
    ROS_ERROR("Connection with shore closed");
    Client=false;
    return false;
  }
  if(ACK[0]!=1){
    ROS_WARN("Wrong ACK received");
    return false;
  }
  return true;
}

/**
 * @brief Range acoustic callback 
 * 
 * @param aux 
 */
void range_Callback(medusa_msgs::mUSBLFix const aux){

  // Not a range between me and the surface
  if(aux.type==aux.AZIMUTH_ONLY || aux.source_name.compare(surface_vehicle)!=0)
    return;
  // The header stamp could correspond to old stuff
  lastping = ros::Time::now();
}

/**
 * @brief Save the depth to validate if the vehicle is underwater
 * 
 * @param ptr 
 */
void ownOdomCallback(const medusa_msgs::mState& ptr){
  //lastDepth=ptr.pose.pose.position.z;
  lastDepth=ptr.Depth;
}

/**
 * @brief Check the acoustic abort signal
 * 
 * @param ptr 
 */
void safetyabort_Callback(const std_msgs::UInt8& ptr){
  if (ptr.data!=0)
    acoustic_abort = true;
  else
    acoustic_abort = false;
}

/**
 * @brief Defines which vehicle is at surface
 * 
 * @param event 
 */
void surfacename_Callback(const ros::MessageEvent<std_msgs::String const>& event){
  lastRoles=*(event.getMessage());

  lastRoles_time = event.getReceiptTime();

  // Parse roles
  /*std::stringstream ss(lastRoles.data);
    std::getline(ss,ownrole,',');
    if(ownrole.empty()) ROS_ERROR("No parameter ownrole was published");
    std::getline(ss,surfacerole,',');
    if(surfacerole.empty()) ROS_ERROR("No parameter surfacerole was published");*/

  // Medusa Architecture
  if(lastRoles.data.empty()) ROS_ERROR("No parameter surface vehicle was published");
  surface_vehicle = lastRoles.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Acoustic_Safety_Feature", ros::init_options::NoSigintHandler);
  signal(SIGINT, mySigIntHandler);

  ros::NodeHandle nh, nh_l("~");


  if(!nh_l.getParamCached("SAFETY_PORT", SAFETY_PORT)) { ROS_ERROR("No Parameter: SAFETY_PORT"); ros::shutdown(); exit(-1);}

  if(!nh_l.getParamCached("RATE", RATE)) { ROS_ERROR("No Parameter: RATE"); ros::shutdown(); exit(-1);}
  if(!nh_l.getParamCached("name", vehicle_name)) { ROS_ERROR("No Parameter: name"); ros::shutdown(); exit(-1);}

  // Surface Vehicle
  if(!nh_l.getParamCached("surface_vehicle", surface_vehicle))
    surface_vehicle.clear();// clear surface_name wait for topic surfacerole

  if(!nh_l.getParamCached("/vehicle/safety_features/acoustic_timeout", watchdog_time)) { ROS_ERROR("No Parameter: acoustic_timeout"); ros::shutdown(); exit(-1);}
  if(!nh_l.getParamCached("/vehicle/safety_features/wifi_timeout", wifi_timeout)) { ROS_ERROR("No Parameter: wifi_timeout"); ros::shutdown(); exit(-1);}
  ros::Publisher safety_pub = nh.advertise<std_msgs::Int8>("/Safety_Feature", 10);

  ros::Subscriber sub_ownstate = nh.subscribe("/State", 10,ownOdomCallback);
  ros::Subscriber sub_range = nh.subscribe("/sensors/usbl_fix", 10, range_Callback);
  ros::Subscriber sub_roles = nh_l.subscribe("surface_vehicle", 10,surfacename_Callback);
  ros::Publisher  stop_pub = nh.advertise<std_msgs::Int8>("/Thruster_Stop", 1000, true); // Stop Thrusters

  ros::Subscriber sub_safety_abort = nh.subscribe("comms/safety_abort", 10,safetyabort_Callback);
  ros::Publisher  safety_abort = nh.advertise<std_msgs::UInt8>("/safety_abort", 10);

  ros::Publisher flag_pub = nh.advertise<std_msgs::Int8>("/Flag", 10);


  if((id_sck=MakeServer(SAFETY_PORT))<=0){
    ROS_WARN("Creating Server. Trying one more time");
    usleep(100000);// Waiting 100ms to try again
    if((id_sck=MakeServer(SAFETY_PORT))<=0){
      ROS_ERROR("Creating Server after two tries. Shutting down the process.");
      ros::shutdown();
      exit(-1);
    }
  }

  RELAY_WIFI = false;

  ros::Rate loop_rate(2.0);
  std_msgs::Int8 aux;
  aux.data=1; safety_pub.publish(aux); // Start by Activating the Safety_Feature
  acoustic_abort = false;
  ros::Time t_old =ros::Time::now() - ros::Duration(1/RATE+1);
  ros::Time t_last_wifi_ping = ros::Time::now();
  lastRoles_time = ros::Time::now();

  bool nosafety_pub=true;

  ROS_INFO("Starting Safety Feature Loop");
  while (!g_request_shutdown && ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
    ros::Time tnow =ros::Time::now();

    // Check if it is a surface vehicle to relay acoustic abort
    if(!surface_vehicle.empty()  && surface_vehicle.compare(vehicle_name)==0)
      RELAY_WIFI = true;

    // Check if the vehicle is underwater to start publishing safety feature
    // Check if it should beleive on acoustics (roles have been published)
    if(lastDepth>=0.3 && (tnow-lastRoles_time).toSec()<5.0){
      // Abort signal
      if(acoustic_abort){
        ROS_ERROR_THROTTLE(10.0,"Acoustic abort signal received");
      }

      // If it did not receive a ping for a long time -> abort
      ROS_INFO("Time diff from the last ping %.2f",round((tnow-lastping).toSec()*10)/10);
      if((tnow-lastping).toSec()>watchdog_time || acoustic_abort){
        if(!acoustic_abort)
          ROS_ERROR_THROTTLE(10,"No ranges between me and %s for a period greater than %.1f. Aborting vehicle!",surface_vehicle.c_str(), watchdog_time);
        aux.data=1; safety_pub.publish(aux); // Activate Safety_Feature
        //stop_pub.publish(aux); // Stop Thrusters, it needs reactivation

        // Abort Mission -> Flag to Idle
        std_msgs::Int8 flag_idle;
        flag_idle.data = 0;
        flag_pub.publish(flag_idle);

        // safety abort
        std_msgs::UInt8 acoustic_msg;
        acoustic_msg.data = 1;
        safety_abort.publish(acoustic_msg);
      }else{
        aux.data=0; safety_pub.publish(aux); // Ranges are comming well
        nosafety_pub = false;
      }
    }else{
      // Updating the (virtual) last ping at the surface
      lastping = tnow;

      if((round((tnow-t_old).toSec()*10)/10>=(1.0/RATE)) && Client){
        //ROS_INFO("Time since last ping %.1f",(tnow-t_old).toSec());
        unsigned char Ack[2];
        t_old=tnow;
        Ack[0]=1;
        // Writing an ACK in the port
        if((write(sckClient, Ack, sizeof(unsigned char)))<=0){
          Client=false;
          ROS_ERROR("Connection with shore closed");
          continue;
        }
        usleep(100000);
        // Reading the ack from the client
        if(!ReadACK(sckClient)){
          // Connection was closed by the other peer
          if(!Client){
            continue;
          }

          // Wifi Ping Timeout has passed
          if((tnow-t_last_wifi_ping).toSec()>=wifi_timeout){
            ROS_ERROR("%.1fs without pings from shore. Shutting down the Thrusters", (tnow-t_last_wifi_ping).toSec());
            close(sckClient);
            Client = false;
            aux.data=1; safety_pub.publish(aux);
            continue;
          }
          //                    ROS_WARN("Error receiving ACK. Let's try again");
          //                    // One more try to send an receive
          //                    if(!Client || (write(sckClient, Ack, sizeof(unsigned char)))<=0){
          //                        Client=false;
          //                        ROS_WARN("Connection closed");
          //                        continue;
          //                    }
          //                    usleep(100000);
          //                    if(!Client || !ReadACK(sckClient)){
          //                        ROS_ERROR("Error in the second try. Shutting down the Thrusters\n");
          //                        close(sckClient);
          //                        Client = false;
          //                        aux.data=1; safety_pub.publish(aux);
          //                        continue;
          //                    }
        }else{
          t_last_wifi_ping = tnow;
        }

        aux.data=0; safety_pub.publish(aux);
        nosafety_pub = false;
      }else if(Client){
        // In between pings
        aux.data=0; safety_pub.publish(aux);
        nosafety_pub = false;
      }else{
        ROS_ERROR_DELAYED_THROTTLE(45.0,"No Safety Client, setting Thrusters to zero");
        if(!nosafety_pub){
          aux.data=1; safety_pub.publish(aux);
          nosafety_pub = true;
        }

        // If the vehicle is the communications relay abort all the underwater vehicles
        if(RELAY_WIFI){
          ROS_ERROR_DELAYED_THROTTLE(5.0,"Sending an acoustic abort and abort Mission!");
          std_msgs::UInt8 acoustic_msg;
          acoustic_msg.data = 1;
          safety_abort.publish(acoustic_msg);

          // Abort Mission -> Flag to Idle
          /*                    std_msgs::Int8 flag_idle;
                    flag_idle.data = 0;
                    flag_pub.publish(flag_idle);*/
        }
        // Send everything to ROS
        ros::spinOnce();

        //ROS_INFO_THROTTLE(15.0,"Waiting for Client to connect on port %d",SAFETY_PORT);
        if((sckClient=AcceptClient(id_sck))>0){
          ROS_INFO("New Client Accepted");
          Client = true;
          acoustic_abort = false;

          if(RELAY_WIFI){
            // Set Acoustic Safety Abort to normal
            std_msgs::UInt8 acoustic_msg;
            acoustic_msg.data = 0;
            safety_abort.publish(acoustic_msg);
          }
        }
      }
    }
  }
  return 0;
}
