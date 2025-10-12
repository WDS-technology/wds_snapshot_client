# client_base.py - Improved version with configurable parameters

import socket
import logging
import time
import os
import errno


class UnixSocketClient:
    def __init__(self, socket_path="/tmp/snapshot_comm_socket/snapshot.sock", buffer_size=4096):
        self.path = socket_path
        self.buffer_size = buffer_size

    def send_sync(self, message):
        """Send a synchronous message and return the response"""
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
                # Set socket timeout to prevent hanging
                s.settimeout(10.0)  # 10 second timeout
                
                # Connect to server
                s.connect(self.path)
                
                # Send message
                s.sendall(message.encode())
                
                # Receive response with proper buffer size
                response = s.recv(self.buffer_size).decode()
                
                logging.debug(f"[UnixSocketClient] Sent: {message}")
                logging.debug(f"[UnixSocketClient] Received: {response}")
                time.sleep(0.1)  # 100ms delay

                return response
                
        except socket.timeout:
            error_msg = f"Socket timeout after 10 seconds for message: {message}"
            logging.error(f"[UnixSocketClient] {error_msg}")
            return f"Error: {error_msg}"
            
        except socket.error as e:
            error_msg = f"Socket error: {e}"
            logging.error(f"[UnixSocketClient] {error_msg}")
            return f"Error: {error_msg}"
            
        except Exception as e:
            error_msg = f"Unexpected error: {e}"
            logging.error(f"[UnixSocketClient] {error_msg}")
            return f"Error: {error_msg}"

    def is_server_available(self):
        """Check if the server socket is available"""
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
                s.settimeout(1.0)  # Short timeout for availability check
                s.connect(self.path)
                return True
        except:
            return False