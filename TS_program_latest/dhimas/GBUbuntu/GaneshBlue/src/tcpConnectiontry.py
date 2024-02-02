import socket

beaglebone_ip = "192.168.2.124"  # BeagleBone's IP address
beaglebone_port = 5000  # The port you chose

# Create a socket object for IPv4 and TCP
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    # Connect to the BeagleBone
    client_socket.connect((beaglebone_ip, beaglebone_port))

    # Send data to the BeagleBone
    message = "Hello, BeagleBone!"
    client_socket.sendall(message.encode())

    # Receive data from the BeagleBone
    data = client_socket.recv(1024)
    print("Received from BeagleBone:", data.decode())

except ConnectionRefusedError:
    print("Connection was refused. Ensure the BeagleBone is listening on the specified port.")

# finally:
#     # Close the client socket
#     client_socket.close()
