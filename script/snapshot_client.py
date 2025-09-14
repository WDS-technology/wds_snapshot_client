# Snapshot-specific wrapper# snapshot_client.py

from client_base import UnixSocketClient
from datetime import datetime
import time
import logging
import os
from response_handler import ResponseHandler


class SnapshotClient:
    def __init__(self, socket_path="/tmp/snapshot_comm_socket/snapshot.sock", buffer_size=4096):
        self.client = UnixSocketClient(socket_path, buffer_size)
        self.socket_path = socket_path
        self.buffer_size = buffer_size
        self.response_handler = ResponseHandler()



    def send_snapshot(self, camera_name: str, destination: str, idx = None, timestamp = None):
        
        # Use provided timestamp or generate new one (UTC+3)
        if timestamp is None:
            from datetime import timedelta
            utc_plus_3 = datetime.now() + timedelta(hours=3)
            timestamp = utc_plus_3.strftime("%d%m%Y_%H%M%S")
        
        suffix = f"_{idx}" if idx is not None else ""
        if not destination.endswith('/'):
            destination += '/'
        filename = f"{camera_name}_{timestamp}{suffix}.jpeg"
        path = os.path.join(destination, filename)

         # Command format that matches your server expectation
        cmd = f"voxl-send-command {camera_name}_snapshot snapshot {path}"
        logging.info(f"[SnapshotClient] Sending command: {cmd}")

        try:
            response = self.client.send_sync(cmd)
            success, message, file_path = self.response_handler.parse_response(response)
            
            result = {
                'success': success,
                'message': message,
                'file_path': file_path,
                'command': cmd,
                'response': response
            }
            
            if success:
                logging.info(f"[SnapshotClient] Success: {message}")
            else:
                logging.error(f"[SnapshotClient] Failed: {message}")
                
            return result
        except Exception as e:
            error_result = {
                'success': False,
                'message': f"Client error: {e}",
                'file_path': None,
                'command': cmd,
                'response': None
            }
            logging.error(f"[SnapshotClient] Exception: {e}")
            return error_result
    def send_multiple_snapshots(self, camera_name: str, destination: str, num_images: int, delay_s=0.0, base_idx=0):
        """Send multiple snapshot commands and collect all results"""
        results = []
        success_count = 0
        
        # Generate timestamp once for the entire batch (UTC+3)
        from datetime import timedelta
        utc_plus_3 = datetime.now() + timedelta(hours=3)
        batch_timestamp = utc_plus_3.strftime("%d%m%Y_%H%M%S")
        logging.info(f"[SnapshotClient] Batch timestamp: {batch_timestamp}")
        
        for i in range(num_images):
            current_idx = base_idx * num_images + i if base_idx else i
            logging.info(f"[SnapshotClient] Taking snapshot {i+1}/{num_images} (idx: {current_idx})")
            
            # Pass the same timestamp to all snapshots in this batch
            result = self.send_snapshot(camera_name, destination, idx=current_idx, timestamp=batch_timestamp)
            results.append(result)
            
            if result['success']:
                success_count += 1
                
            # Configurable delay between snapshots, accounting for processing time
            if delay_s > 0 and i < num_images - 1:
                import time
                start_time = time.time()
                time.sleep(max(0, delay_s - (time.time() - start_time)))
        
        overall_success = success_count == num_images
        logging.info(f"[SnapshotClient] Completed: {success_count}/{num_images} successful")
        
        return {
            'overall_success': overall_success,
            'success_count': success_count,
            'total_count': num_images,
            'results': results
        }

                

    def shutdown(self):
        try:
            self.client.shutdown()
        except Exception:
            pass