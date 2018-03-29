#include "socket.hpp"

#include <netdb.h>

Socket::Socket(std::string address, int port) : address(address), port(port) {
    this->sock = socket(AF_INET, SOCK_STREAM, 0);
    if(this->sock == -1) {
        std::fprintf(stderr, "Error: Couldn't open socket");
    }
    if(inet_addr(address.c_str()) == -1) {
        struct hostent *he;
        struct in_addr **addr_list;
        if((he = gethostbyname(address.c_str())) == NULL) {
            herror("gethostbyname");
            std::fprintf(stderr, "Couldn't resolve hostname of non-IP address \"%s\"", address.c_str());
            // exit(-2);
        }
        addr_list = ((struct in_addr **) he->h_addr_list);
        for(int i = 0; addr_list[i] != NULL; i++) {
            server.sin_addr = *addr_list[i];
            break;
        }
    } else {
        server.sin_addr.s_addr = inet_addr(address.c_str());
    }
    server.sin_family = AF_INET;
    server.sin_port = htons(port);
};

void Socket::open() {
    if(connect(sock, (struct sockaddr *) &server, sizeof(server)) > 0) {
        std::fprintf(stderr, "Error: Socket connect failed");
        // exit(-1);
    }
}

//bool Socket::send() {
//
//}
//


