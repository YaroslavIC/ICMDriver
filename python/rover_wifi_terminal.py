# rover_pc_server.py

import socket
import threading

HOST = "0.0.0.0"
PORT = 3333

def rx_thread(conn):
    while True:
        data = conn.recv(4096)
        if not data:
            print("\n[ESP32 disconnected]")
            break
        print(data.decode(errors="replace"), end="")

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)

print(f"Waiting for ESP32 on port {PORT}...")

conn, addr = server.accept()
print("ESP32 connected from:", addr)

threading.Thread(target=rx_thread, args=(conn,), daemon=True).start()

while True:
    line = input()
    if line.lower() in ("exit", "quit"):
        break
    conn.sendall((line + "\r\n").encode())

conn.close()
server.close()