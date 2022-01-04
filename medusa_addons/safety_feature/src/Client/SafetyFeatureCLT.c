#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>


#ifdef _WIN32
#include "windows.h"
#include <sys/stat.h>
#include <fcntl.h>
#include <Ws2tcpip.h>
#include <winsock.h>
#include <winsock2.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#endif
void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int Connect2Server(char *ip, int portno){
    int sockfd, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    
    #ifdef _WIN32
    int iResult;
    WSADATA wsaData;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed: %d\n", iResult);
        return 1;
    }
    #endif
    
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0){
      perror("ERROR opening socket");
	    return 0;
    }
    server = gethostbyname(ip);
    if (server == NULL) {
      perror("ERROR no such host");
      return 0;
    }

    #ifdef _WIN32
    serv_addr.sin_family = AF_INET;
    CopyMemory(&serv_addr.sin_addr, server->h_addr_list[0],server->h_length);
    #else
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    #endif
    serv_addr.sin_port = htons(portno);
    
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0){
      perror("ERROR connecting");
      return 0;
    }

    return sockfd;

} 
int main(int argc, char **argv)
{
    int sock, i=0;
    unsigned char Ack[2];
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    if(argc<2){
      printf("SafetyFeatureCLT [IP]\n");
      return 0;
    }else if(argc==2){
      printf("SafetyFeatureCLT\nConnecting to [%s]....\n", argv[1]);
    }
    
    sock = 0;
    while((sock=Connect2Server(argv[1],10111))<=0)
      sleep(1);
    
    printf("Connected! [%d]\n", sock);
    while(1){
      if(read(sock, Ack, sizeof(unsigned char))<=0){
        printf("Connection with [%s] closed\nTrying to connect again...\n",argv[1]);
        close(sock);sock = 0;
        while((sock=Connect2Server(argv[1],10111))<=0)
          sleep(1);
        printf("Connected!\n");
      }else{
        time(&t);
        tm=*localtime(&t);
        printf("[%d:%d.%d] IP=[%s]\n", tm.tm_hour, tm.tm_min, tm.tm_sec, argv[1]);
        write(sock, Ack, sizeof(unsigned char));
      }
    }
    close(sock);
    #ifdef _WIN32
      WSACleanup();
    #endif
   return 0;
}
