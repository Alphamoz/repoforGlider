#include <iostream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

const char* SERVER_IP = "192.168.2.124";  // BeagleBone's IP address
const int SERVER_PORT = 5001;             // BeagleBone's port

int main() {
    int clientSocket;
    struct sockaddr_in serverAddr;

    // Create socket
    clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket < 0) {
        perror("Error creating socket");
        exit(EXIT_FAILURE);
    }

    // Configure server address
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);
    if (inet_pton(AF_INET, SERVER_IP, &(serverAddr.sin_addr)) <= 0) {
        perror("Invalid server IP address");
        exit(EXIT_FAILURE);
    }

    // Connect to the server
    if (connect(clientSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("Error connecting to server");
        exit(EXIT_FAILURE);
    }

    std::cout << "Connected to server at " << SERVER_IP << ":" << SERVER_PORT << std::endl;
    while (true) {
        const char* message = "Hello, BeagleBone!";
        int bytesSent = send(clientSocket, message, strlen(message), 0);
        if (bytesSent < 0) {
            perror("Error sending data");
            break;
        }

        // Receive a response from the server
        char buffer[3000];
        memset(buffer, 0, sizeof(buffer));
        int bytesRead = read(clientSocket, buffer, sizeof(buffer));
        if (bytesRead <= 0) {
            break;
        }
        std::cout << "Received: " << buffer << std::endl;

        usleep(200000); // Sleep for 1 second before sending the next message (adjust as needed)
    }

    // Close the socket
    close(clientSocket);

    return 0;
}
