#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include <signal.h>
/*INCLUDE SOCKET HEADERS*/
#include <sys/types.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
    ROS_INFO("Shutdown Signal received!");
    g_request_shutdown = 1;
    exit(0);
}

int id_sck, sckClient, SAFETY_PORT;
double RATE;
bool Client, RELAY_WIFI;

/* 
 * Creates a TCP server with a specified port
 */
int MakeServer(int port){
	int sck;
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
	listen(sck,5);
	return sck;
}

/*
 * Accepts any client that trys to connect
 */
int AcceptClient(int sockfd){
struct sockaddr_in cli_addr;
int  newsockfd, clilen;
	clilen = sizeof(cli_addr);
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

return newsockfd;
}

/*
 * Reads the message and verifies if it comes correctly or not
 * Waits for 1 second to receive this message
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
		Client=false;
		return false;
	}
	if(ACK[0]!=1){
        ROS_WARN("Wrong ACK received\n");
		return false;
	}
	return true;
}

int main(int argc, char **argv)
{
    // Override SIGINT handler
    ros::init(argc, argv, "Safety_Feature", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigIntHandler);

	ros::NodeHandle nh, nh_l("~");


	if(!nh_l.getParamCached("SAFETY_PORT", SAFETY_PORT)) { ROS_ERROR("No Parameter: SAFETY_PORT"); ros::shutdown(); exit(-1);}
    if(!nh_l.getParamCached("RATE", RATE)) { ROS_ERROR("No Parameter: RATE"); ros::shutdown(); exit(-1);}
    if(!nh_l.getParamCached("RELAY", RELAY_WIFI)) { ROS_ERROR("No Parameter: RELAY"); ros::shutdown(); exit(-1);}

	if((id_sck=MakeServer(SAFETY_PORT))<=0){
        ROS_WARN("Creating Server. Trying one more time");
		usleep(100000);// Waiting 100ms to try again
		if((id_sck=MakeServer(SAFETY_PORT))<=0){
			ROS_ERROR("Creating Server after two tries. Shuting down the process.");
			ros::shutdown(); 
			exit(-1);
		}
	}

    ros::Publisher safety_pub = nh.advertise<std_msgs::Int8>("Safety_Feature", 10);
    ros::Publisher acoustic_pub = nh.advertise<std_msgs::Bool>("acoustic_abort", 10);

	ros::Rate loop_rate(1);
	ROS_INFO("Starting Safety Feature Loop");
	std_msgs::Int8 aux;
	aux.data=1; safety_pub.publish(aux); // Start by Activating the Safety_Feature
	ros::Time t_old =ros::Time::now() - ros::Duration(1/RATE+1);

    while (!g_request_shutdown && ros::ok()) {
		loop_rate.sleep();
		ros::spinOnce();
		ros::Time t_now =ros::Time::now();

		if(((t_now-t_old).toSec()>=(1/RATE)) && Client){
			unsigned char Ack[2];
			t_old=t_now;
			Ack[0]=1;
			// Writing an ACK in the port
			if((write(sckClient, Ack, sizeof(unsigned char)))<=0){
				Client=false;
				ROS_ERROR("Connection closed");
				continue;
			}
			usleep(100000);
			// Reading the ack from the client
			if(!ReadACK(sckClient)){
                ROS_WARN("Error receiving ACK. Let's try again");
				// One more try to send an receive
				if(!Client || (write(sckClient, Ack, sizeof(unsigned char)))<=0){
					Client=false;
					ROS_WARN("Connection closed");
					continue;
				}
				usleep(100000);
				if(!Client || !ReadACK(sckClient)){
					ROS_ERROR("Error in the second try. Shutting down the Thrusters\n");
					close(sckClient);
					Client = false;
					aux.data=1; safety_pub.publish(aux);
					continue;
                }
			}
			aux.data=0; safety_pub.publish(aux); 
			//ROS_INFO("PINGING");
        }else if(Client){
            // In between pings
            aux.data=0; safety_pub.publish(aux);
		}else{
			// Send 5 times the message for shuting down the thrusters
			ROS_ERROR("Set Thrusters to zero");
			for(int i=0;i<5;i++){
				aux.data=1; safety_pub.publish(aux);
				usleep(200000);
			}

            // If the vehicle is the communications relay abort all the underwater vehicles
            if(RELAY_WIFI){
                std_msgs::Bool acoustic_msg;
                acoustic_msg.data = true;
                acoustic_pub.publish(acoustic_msg);
            }
			ROS_INFO("Waiting for Client to connect on port %d",SAFETY_PORT);
			if((sckClient=AcceptClient(id_sck))>0){
				ROS_INFO("New Client Accepted");
				Client = true;
			}
		}
	}
	return 0;
}
