# client_base.py - Improved version with configurable parameters

import socket
import logging
import time
import os
import errno


class UnixSocketClient:
    def __init__(self, socket_path="/tmp/snapshot_comm_socket/snapshot.sock", buffer_size=32768):
        """
        Initialize Unix socket client
        
        Args:
            socket_path: Path to Unix domain socket
            buffer_size: Buffer size for receiving data (default 32KB)
        """
        self.path = socket_path
        self.buffer_size = buffer_size

    def send_sync(self, message):
        """Send a synchronous message and return the response"""
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
                # Set socket timeout to prevent hanging
                s.settimeout(30.0)
                
                # Connect to server
                s.connect(self.path)
                
                # Send message
                s.sendall(message.encode())
                
                # Receive response with proper buffer size
                response_parts = []
                total_received = 0

                
                while True:
                    try:
                        # Receive chunk
                        chunk = s.recv(self.buffer_size)
                        
                        if not chunk:
                            # Connection closed by server (normal end of response)
                            break
                            
                        response_parts.append(chunk)
                        total_received += len(chunk)
                        
                        # If we received less than buffer size, likely got everything
                        if len(chunk) < self.buffer_size:
                            break
                            
                    except socket.timeout:
                        # Timeout waiting for more data - assume response is complete
                        if response_parts:
                            logging.debug(f"[UnixSocketClient] Receive timeout after {total_received} bytes - assuming complete")
                            break
                        else:
                            raise  # Re-raise if no data received at all
                
                # Decode the complete response
                response = b''.join(response_parts).decode()
                
                logging.debug(f"[UnixSocketClient] Sent: {message[:100]}...")  # Truncate log
                logging.debug(f"[UnixSocketClient] Received {total_received} bytes")
                time.sleep(0.1)  # 100ms delay

                return response
                
        except socket.timeout:
            error_msg = f"Socket timeout after 30 seconds for message: {message[:50]}..."
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