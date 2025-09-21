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
    def __init__(self, socket_path="/tmp/snapshot_comm_socket/snapshot.sock", buffer_size=4096, csv_logger=None):

        self.client = UnixSocketClient(socket_path, buffer_size)
        self.csv_logger = csv_logger  # Use external logger instead of creating own

        self.socket_path = socket_path
        self.buffer_size = buffer_size
        self.response_handler = ResponseHandler()

       
    def set_csv_logger(self, csv_logger):
            """Set the CSV logger from external source"""
            self.csv_logger = csv_logger
    def _log_image_metadata(
        self,
        camera_name: str,
        batch_id: int,
        frame_idx: int,
        destination: str,
        qvio_pose=None,
    ):
        """Log single image metadata to CSV - called once per frame"""
        if not self.csv_logger:
            return

        # Determine camera folder and encoding based on camera name
        camera_folder = (
            "hires1"
            if camera_name.startswith("hires_")
            and not camera_name.startswith("hires2_")
            else "hires2"
        )
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

        # Write to appropriate CSV (hires1.csv or hires2.csv)
        self.csv_logger.writeImageMetadata(camera_folder, metadata)

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

        destination += camera_name.lower() + "/"

        # Generate batch_id if not provided (use current epoch seconds)
        if batch_id is None:
            import time

            batch_id = int(time.time()) % 100000  # Use last 5 digits of epoch

        # Handle "both" cameras - send commands to hires and hires2
        if camera_name.lower() == "both":
            results = []
            cameras = ["hires", "hires2"]

            for cam in cameras:
                # Apply color/grey suffix
                color_suffix = "_color" if color == 1 else "_grey"
                cam_with_suffix = f"{cam}{color_suffix}"
                print(f"[RawImageClient] Using {cam_with_suffix} (color={color})")

                # Command format: use -n flag if capturing multiple frames, omit for single frame
                if num_of_frames > 1:
                    # For multiple frames with both cameras, use batch_id and timestamp
                    filename_base = f"{batch_id}_"
                    path = os.path.join(destination, filename_base)
                    cmd = f"voxl-record-raw-image {cam_with_suffix} -d {path} -n {num_of_frames}"
                else:
                    # For single frame, include frame number
                    filename_base = f"{batch_id}_{base_idx}_"
                    path = os.path.join(destination, filename_base)
                    cmd = f"voxl-record-raw-image {cam_with_suffix} -d {path} "

                print(f"[RawImageClient] Sending command for {cam}: {cmd}")

                try:
                    result = self.client.send_sync(cmd)
                    print(f"[RawImageClient] {cam} Result: {result}")
                    results.append({"camera": cam, "result": result, "success": True})

                    # Log to CSV after successful capture
                    self._log_image_metadata(
                        cam_with_suffix,
                        batch_id,
                        base_idx,
                        destination,
                        qvio_pose,
                    )

                except Exception as e:
                    error_msg = f"Error sending raw image command to {cam}: {e}"
                    print(f"[RawImageClient] {error_msg}")
                    results.append(
                        {"camera": cam, "result": error_msg, "success": False}
                    )

            # Return combined results
            return {
                "cameras": cameras,
                "results": results,
                "all_success": all(r["success"] for r in results),
            }

        # Single camera handling (existing logic)
        color_suffix = "_color" if color == 1 else "_grey"
        print(f"[RawImageClient] Using {camera_name}{color_suffix} (color={color})")
        camera_name = f"{camera_name}{color_suffix}"

        # Command format: use -n flag if capturing multiple frames, omit for single frame
        if num_of_frames > 1:
            filename_base = f"{batch_id}_"
            path = os.path.join(destination, filename_base)
            cmd = f"voxl-record-raw-image {camera_name} -d {path} -n {num_of_frames}"
        else:
            filename_base = f"{batch_id}_{base_idx}_"
            path = os.path.join(destination, filename_base)
            cmd = f"voxl-record-raw-image {camera_name} -d {path}"

        print(f"[RawImageClient] Sending command: {cmd}")

        try:
            result = self.client.send_sync(cmd)
            print(f"[RawImageClient] Result: {result}")

            # Log to CSV after successful capture
            self._log_image_metadata(
                camera_name,
                batch_id,
                base_idx,
                destination,
                qvio_pose,
            )

            return result
        except Exception as e:
            error_msg = f"Error sending raw image command: {e}"
            print(f"[RawImageClient] {error_msg}")
            return error_msg

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
        import time

        # Generate batch_id once for the entire batch
        batch_id = int(time.time()) % 100000  # Use last 5 digits of epoch
        print(f"[RawImageClient] Batch ID: {batch_id}")

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
        print(f"[RawImageClient] Delay {delay_s}s - {num_of_frames} separate commands")
        results = []

        for i in range(num_of_frames):
            current_idx = i  # Always start from 0 for each frame
            print(
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

            except Exception as e:
                error_msg = f"Error taking raw image {i+1}: {e}"
                print(f" → {error_msg}")
                results.append(error_msg)

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
