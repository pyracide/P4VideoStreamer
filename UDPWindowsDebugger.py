import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 8282))
print("Listening for UDP packets on port 8282...")
while True:
    data, addr = sock.recvfrom(1024)
    print(f"Received from {addr}: {data.decode('utf-8')}")