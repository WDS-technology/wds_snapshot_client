# Snapshot-specific wrapper# snapshot_client.py

from client_base import UnixSocketClient
from response_handler import ResponseHandler
from datetime import datetime
import time
import logging
import os

# Import the CSV logger from pybind11 module
try:
    import py_csv_logger
except ImportError:
    print("Warning: py_csv_logger module not found. CSV logging will be disabled.")
    py_csv_logger = None



class SnapshotClient:
    def __init__(self, socket_path="/tmp/snapshot_comm_socket/snapshot.sock", buffer_size=4096, csv_logger=None):
        self.client = UnixSocketClient(socket_path, buffer_size)
        self.csv_logger = csv_logger  # Use external logger instead of creating own
        self.socket_path = socket_path
        self.buffer_size = buffer_size
        self.response_handler = ResponseHandler()
        # Sequential batch counter starting from 0
        self.batch_counter = 0

    def set_csv_logger(self, csv_logger):
        """Set the CSV logger from external source"""
        self.csv_logger = csv_logger

    def _get_next_batch_id(self):
            """Get the next sequential batch ID and increment counter"""
            current_batch = self.batch_counter
            self.batch_counter += 1
            return current_batch


    def _log_image_metadata(
        self,
        camera_name: str,
        batch_id: int,
        frame_idx: int,
        destination: str,
        qvio_pose=None,
    ):
        """Log single image metadata to CSV - called once per frame"""
        logging.info(f"[DEBUG] About to log CSV for camera_name='{camera_name}', batch={batch_id}, frame={frame_idx}")

        if not self.csv_logger:
            logging.warning("[SnapshotClient] No CSV logger available, skipping metadata logging")
            return False
        
        try:
            # Determine camera folder and encoding based on camera name
            if camera_name.startswith("hires2"):
                camera_folder = "hires2"
            else:  # hires, hires_grey, hires_color all go to hires
                camera_folder = "hires"
        
            encoding = "bgr8" 
            extension = ".jpeg"  # Snapshots are always JPEG

            # Create metadata for single frame
            metadata = py_csv_logger.ImageMetadata()
            metadata.batch_id = batch_id
            metadata.frame_number = frame_idx
            metadata.height = 3040  # Standard snapshot resolution
            metadata.width = 4056
            metadata.encoding = encoding


            actual_filename = f"{batch_id}_{frame_idx}_{camera_name}_snapshot.jpeg"

            # Full path already includes camera name in destination
            metadata.image_file_path = os.path.join(destination, actual_filename)
            metadata.image_title = actual_filename

            # Set QVIO position data (use provided pose or zeros as default)
            if qvio_pose:
                metadata.position_x = qvio_pose.pose.position.x
                metadata.position_y = qvio_pose.pose.position.y
                metadata.position_z = qvio_pose.pose.position.z
                metadata.orientation_w = qvio_pose.pose.orientation.w
                metadata.orientation_x = qvio_pose.pose.orientation.x
                metadata.orientation_y = qvio_pose.pose.orientation.y
                metadata.orientation_z = qvio_pose.pose.orientation.z
            else:
                # Default values when no QVIO data available
                metadata.position_x = 0.0
                metadata.position_y = 0.0
                metadata.position_z = 0.0
                metadata.orientation_w = 1.0
                metadata.orientation_x = 0.0
                metadata.orientation_y = 0.0
                metadata.orientation_z = 0.0

            # Write to appropriate CSV (hires.csv or hires2.csv)
            self.csv_logger.writeImageMetadata(camera_folder, metadata)
            logging.debug(f"[SnapshotClient] CSV metadata logged for {camera_name}, batch {batch_id}, frame {frame_idx}")
            return True
    
        except Exception as e:
            logging.error(f"[SnapshotClient] Failed to log CSV metadata for {camera_name}: {e}")
            return False

    def send_snapshot(self,
        camera_name: str,
        destination: str,
        num_of_frames: int,
        base_idx: int,
        batch_id: int = None,
        qvio_pose=None):

        """Send snapshot command for single camera"""
        # Ensure destination ends with /
        if not destination.endswith("/"):
            destination += "/"

         # Generate batch_id if not provided
        if batch_id is None:
            batch_id = self._get_next_batch_id()

        # Single camera handling
        if camera_name.lower().startswith("hires2"):
            single_cam_destination = destination + "hires2/"
        else:  # hires goes to hires folder
            single_cam_destination = destination + "hires/"    
        
        filename = f"{batch_id}_{base_idx}_{camera_name}_snapshot.jpeg"
        path = os.path.join(single_cam_destination, filename)


        # Command format for voxl-send-command
        cmd = f"voxl-send-command {camera_name}_snapshot snapshot {path}"
        print(f"[SnapshotClient] Sending command: {cmd}")

        # Initialize status variables
        image_success = False
        csv_success = False
        result = None

        try:
            result = self.client.send_sync(cmd)
            print(f"[SnapshotClient] Result: {result}")
            
            # Check if result contains error
            if result and "Error:" in result:
                image_success = False
                logging.error(f"[SnapshotClient] Command failed: {result}")
            else:
                image_success = True
                # Only log to CSV if image was successful
                csv_success = self._log_image_metadata(
                    camera_name,
                    batch_id,
                    base_idx,
                    single_cam_destination,
                    qvio_pose,
                )
        except Exception as e:
            error_msg = f"Error sending snapshot command: {e}"
            logging.error(error_msg)
        
        # Return status
        return {
            "result": result,
            "image_success": image_success,
            "csv_success": csv_success,
            "overall_success": image_success
        }

    def send_multiple_snapshots( self,
        camera_name: str,
        destination: str,
        num_of_frames: int,
        base_idx=0,
        delay_s=0.25,
        color: int = 1,
        qvio_pose=None):

        """Send multiple snapshot commands"""
        # Generate batch_id once for the entire batch
        batch_id = self._get_next_batch_id()
        print(f"[SnapshotClient] Batch ID: {batch_id}")

        if camera_name.lower() == "both":
            # Handle "both" cameras
            cameras = ["hires", "hires2"]
            results = []

            if delay_s == 0:
                # Fast mode - process all frames for each camera
                for cam in cameras:
                    for frame_idx in range(num_of_frames):
                        result = self.send_snapshot(cam, destination, 1, frame_idx, batch_id, qvio_pose)
                        results.append(result)
                    if cam == "hires":
                        time.sleep(0.1)  # Brief delay between cameras
                return results
            else:
                # Synchronized frame mode - each frame processes both cameras
                for i in range(num_of_frames):
                    for cam in cameras:
                        result = self.send_snapshot(
                            cam,
                            destination,
                            1,
                            i,  # Frame index
                            batch_id,
                            qvio_pose
                        )
                        results.append(result)
                        
                        # Small delay between cameras within same frame
                        if cam == "hires":
                            time.sleep(0.5)  # Delay between hires and hires2
                    
                    # Frame delay (except after last frame)
                    if i < num_of_frames - 1:
                        time.sleep(delay_s)
                
                return results
        else:
            # Single camera mode
            if delay_s == 0:
                # Fast mode - all frames in sequence
                results = []
                for i in range(num_of_frames):
                    result = self.send_snapshot(
                        camera_name,
                        destination,
                        1,
                        i,
                        batch_id,
                        qvio_pose,
                    )
                    results.append(result)
                return results
            else:
                # Delayed mode
                logging.debug(f"[SnapshotClient] Delay {delay_s}s - {num_of_frames} separate commands")
                results = []

                for i in range(num_of_frames):
                    logging.debug(f"[SnapshotClient] Taking snapshot {i+1}/{num_of_frames} (idx: {i})")

                    try:
                        result = self.send_snapshot(
                            camera_name,
                            destination,
                            1,
                            i,
                            batch_id,
                            qvio_pose,
                        )
                        results.append(result)

                        # Check if the result indicates failure
                        if isinstance(result, dict) and not result.get("overall_success", False):
                            logging.warning(f"[SnapshotClient] Image {i+1} failed but continuing")

                    except Exception as e:
                        error_msg = f"Error taking snapshot {i+1}: {e}"
                        logging.debug(f" → {error_msg}")
                        results.append({
                            "result": error_msg, 
                            "image_success": False, 
                            "csv_success": False, 
                            "overall_success": False
                        })

                    # Add delay between commands (except after the last one)
                    if i < num_of_frames - 1:
                        time.sleep(delay_s)

                return results

                

    def shutdown(self):
        """Shutdown the client connection"""
        try:
            if hasattr(self.client, "shutdown"):
                self.client.shutdown()
        except Exception as e:
            print(f"Error during snapshot client shutdown: {e}")
            pass