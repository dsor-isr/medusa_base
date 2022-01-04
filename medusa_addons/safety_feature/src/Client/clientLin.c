#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int Connect2Server(char *ip, int portno){
    int sockfd, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0){
        perror("ERROR opening socket");
	return 0;
    }
    server = gethostbyname(ip);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        return 0;
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0){
        perror("ERROR connecting");
	return 0;
    }

    return sockfd;

} 
int main(void)
{
   int sock, i=0;
   unsigned char Ack[2];
   sock =Connect2Server("127.0.0.1",10111);
   if (sock==0)
	return 0;
   while(1){
	if(read(sock, Ack, sizeof(unsigned char))<=0){
		close(sock);
		while((sock=Connect2Server("127.0.0.1",10111))<=0)
			usleep(500000);
	}else{
		printf("TEXT=%d\n",Ack[0]);
		write(sock, Ack, sizeof(unsigned char));
	}
   }
   return 0;
}
