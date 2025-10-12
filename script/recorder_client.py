# Record-raw wrapper

# snapshot_client.py

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


class RawImageClient:
    def __init__(self, socket_path="/tmp/snapshot_comm_socket/snapshot.sock", buffer_size=4096, csv_logger=None, file_logger=None):

        self.client = UnixSocketClient(socket_path, buffer_size)
        self.csv_logger = csv_logger  # Use external logger instead of creating own
        self.file_logger = file_logger  # File-only logger for voxl command output

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
        logging.debug(f"About to log CSV for camera_name='{camera_name}', batch={batch_id}, frame={frame_idx}")

        if not self.csv_logger:
            logging.warning("[RawImageClient] No CSV logger available, skipping metadata logging")
            return False
        
        try:

            # Determine camera folder and encoding based on camera name
            if camera_name.startswith("hires2"):
                camera_folder = "hires2"
            else:  # hires, hires_grey, hires_color all go to hires
                camera_folder = "hires"
        
            encoding = "mono8" if "_grey" in camera_name else "bgr8"
            extension = ".gray" if "_grey" in camera_name else ".bin"

            # Create metadata for single frame
            metadata = py_csv_logger.ImageMetadata()
            metadata.batch_id = batch_id
            metadata.frame_number = frame_idx  # Use frame_idx directly as frame_number
            metadata.height = 3040
            metadata.width = 4056
            metadata.encoding = encoding

            # Actual filename format: {batch_id}_{frame_idx}_{camera}_{width}x{height}.ext
            # Example: "12345_0_hires_grey_4056x3040.gray"
            actual_filename = f"{batch_id}_{frame_idx}_{camera_name}_{metadata.width}x{metadata.height}{extension}"

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
            logging.debug(f"[RawImageClient] CSV metadata logged for {camera_name}, batch {batch_id}, frame {frame_idx}")
            return True
    
        except Exception as e:
            logging.error(f"[RawImageClient] Failed to log CSV metadata for {camera_name}: {e}")
            return False

    def _validate_path_length(self, destination: str, camera_name: str, batch_id: int, base_idx: int):
        """Validate that the resulting filename won't exceed filesystem limits"""
        # Calculate the maximum possible filename that voxl-record-raw-image will create
        # Format: {batch_id}_{base_idx}_{camera_name}_{resolution}.{extension}

        # Estimate maximum component sizes
        batch_str = str(batch_id)
        base_idx_str = str(base_idx)
        max_camera_suffix = "_color" if "_" not in camera_name else ""
        max_resolution = "4056x3040"
        max_extension = ".gray"

        # Build the maximum possible filename
        max_filename = f"{batch_str}_{base_idx_str}_{camera_name}{max_camera_suffix}_{max_resolution}{max_extension}"

        # Calculate total path length
        full_path = destination + max_filename

        # Check against actual observed limits (based on user test results)
        # From testing: paths over ~100 chars cause filename truncation
        # FOLDER name limit: 37 characters confirmed working (current test folder)
        # Total path limit: 100 characters observed safe limit
        MAX_FILENAME_LENGTH = 255  # Standard Linux limit
        MAX_SAFE_PATH_LENGTH = 100  # Observed safe limit from user testing
        WARNING_THRESHOLD = 95  # Start warning at 95 chars (closer to limit)

        if len(max_filename) > MAX_FILENAME_LENGTH:
            raise ValueError(f"Filename too long ({len(max_filename)} chars): {max_filename[:50]}... (max: {MAX_FILENAME_LENGTH})")

        if len(full_path) > MAX_SAFE_PATH_LENGTH:
            raise ValueError(f"Path too long ({len(full_path)} chars): {full_path} (max safe: {MAX_SAFE_PATH_LENGTH})")

        # Log warning if getting very close to limits (95+ chars = danger zone)
        if len(full_path) >= WARNING_THRESHOLD:
            logging.warning(f"[RawImageClient] Path near truncation limit: {len(full_path)}/{MAX_SAFE_PATH_LENGTH} chars - filename truncation likely!")

        return True

    def send_raw_image(
        self,
        camera_name: str,
        destination: str,
        num_of_frames: int,
        base_idx: int,
        batch_id: int = None,
        color: int = 0,
        qvio_pose=None,
    ):
        # if camera name is hires make destinantion = destination + /hires
        # Ensure destination ends with /
        if not destination.endswith("/"):
            destination += "/"

          # Generate batch_id if not provided (use sequential counter)
        if batch_id is None:
            batch_id = self._get_next_batch_id()

        # Single camera handling (existing logic)
        if camera_name.lower().startswith("hires2"):
            single_cam_destination = destination + "hires2/"
        else:  # hires goes to hires folder
            single_cam_destination = destination + "hires/"

        color_suffix = "_color" if color == 1 else "_grey"
        cam_with_suffix = f"{camera_name}{color_suffix}"
        print(f"[RawImageClient] Using {camera_name}{color_suffix} (color={color})")
        camera_name = f"{camera_name}{color_suffix}"

        # Validate path length before proceeding
        try:
            self._validate_path_length(single_cam_destination, camera_name, batch_id, base_idx)
        except ValueError as e:
            error_msg = f"Path validation failed: {e}"
            logging.error(f"[RawImageClient] {error_msg}")
            return {
                "result": f"Error: {error_msg}",
                "image_success": False,
                "csv_success": False,
                "overall_success": False
            }

        # Command format: use -n flag if capturing multiple frames, omit for single frame
        if num_of_frames > 1:
            filename_base = f"{batch_id}_"
            path = os.path.join(single_cam_destination, filename_base)
            cmd = f"voxl-record-raw-image {camera_name} -d {path} -n {num_of_frames}"
        else:
            filename_base = f"{batch_id}_{base_idx}_"
            path = os.path.join(single_cam_destination, filename_base)
            cmd = f"voxl-record-raw-image {camera_name} -d {path}"

        logging.info(f"[RawImageClient] Sending command: {cmd}")
        # Initialize status variables
        image_success = False
        csv_success = False
        result = None

        try:
            result = self.client.send_sync(cmd)
            # Log result to file only to prevent terminal clearing
            if self.file_logger:
                self.file_logger.info(f"[RawImageClient] Result: {result}")
            else:
                # Fallback to regular logging if no file logger
                logging.debug(f"[RawImageClient] Result: {result}")
            # Check if result contains error
            if result and "Error:" in result:
                image_success = False
                logging.error(f"[RawImageClient] Command failed: {result}")
            else:
                image_success = True
                # Only log to CSV if image was successful
                csv_success = self._log_image_metadata(
                    cam_with_suffix,
                    batch_id,
                    base_idx,
                    single_cam_destination,
                    qvio_pose,
                )
        except Exception as e:
            error_msg = f"Error sending raw image command: {e}"
            logging.error(error_msg)
            

        # Return Outside the try-except to ensure it's always returned   
        return {
        "result": result,
        "image_success": image_success,
        "csv_success": csv_success,
        "overall_success": image_success
        }
    def send_multiple_raw_images(
        self,
        camera_name: str,
        destination: str,
        num_of_frames: int,
        base_idx=0,
        delay_s=0.25,
        color: int = 0,
        qvio_pose=None,
    ):
        """Send raw images - single command if no delay, multiple commands with delay"""
        # Generate batch_id once for the entire batch
        batch_id = self._get_next_batch_id()  # Gets batch 0, 1, 2, etc.
        print(f"[RawImageClient] Batch ID: {batch_id}")
        
        if camera_name.lower() == "both":
            # Handle "both" at this level, not in send_raw_image
            cameras = ["hires", "hires2"]
            results = []

             
            if delay_s == 0:
                # Fast mode - use -n flag for multiple frames per camera
                for cam in cameras:
                    result = self.send_raw_image(cam, destination, num_of_frames, 0, batch_id, color, qvio_pose)
                    results.append(result)
                    if cam == "hires":
                        time.sleep(0.1)  # Brief delay between cameras
                return results
            else:
                
                for i in range(num_of_frames):  # For each frame
                    for cam in cameras:  # Process each camera for this frame
                        result = self.send_raw_image(
                            cam,  # Pass individual camera name
                            destination,
                            1,  # Single frame
                            i,  # Frame index
                            batch_id,
                            color,
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
            # If no delay, use single command with -n flag
            if delay_s == 0:
                if num_of_frames > 1:
                    print(
                        f"[RawImageClient] WARNING: Taking {num_of_frames} frames with 0 time delta - frames will be captured as fast as possible without delay"
                    )
                    print(
                        f"[RawImageClient] This may not provide the intended time spacing between frames"
                    )
                print(
                    f"[RawImageClient] No delay - single command for {num_of_frames} frames"
                )
                result = self.send_raw_image(
                    camera_name,
                    destination,
                    num_of_frames,
                    base_idx,
                    batch_id,
                    color,
                    qvio_pose,
                )
                return [result]

            # If delay > 0, loop and call send_raw_image multiple times with single frames
            logging.debug(f"[RawImageClient] Delay {delay_s}s - {num_of_frames} separate commands")
            results = []

            for i in range(num_of_frames):
                current_idx = i  # Always start from 0 for each frame
                logging.debug(
                    f"[RawImageClient] Taking raw image {i+1}/{num_of_frames} (idx: {current_idx})"
                )

                try:
                    # Call send_raw_image with single frame (1) and same batch timestamp/id
                    result = self.send_raw_image(
                        camera_name,
                        destination,
                        1,
                        current_idx,
                        batch_id,
                        color,
                        qvio_pose,
                    )
                    results.append(result)

                    # Check if the result indicates failure
                    if isinstance(result, dict) and not result.get("overall_success", False):
                        logging.warning(f"[RawImageClient] Image {i+1} failed but continuing with remaining images")
                    elif isinstance(result, str) and "Error" in result:
                        logging.warning(f"[RawImageClient] Image {i+1} failed: {result}")

                except Exception as e:
                    error_msg = f"Error taking raw image {i+1}: {e}"
                    logging.debug(f" → {error_msg}")
                    results.append({"result": error_msg, "image_success": False, "csv_success": False, "overall_success": False})


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
            print(f"Error during raw image client shutdown: {e}")
            pass
