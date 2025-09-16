# Record-raw wrapper

# snapshot_client.py

from client_base import UnixSocketClient
from response_handler import ResponseHandler
from datetime import datetime
import time
import logging
import os


class RawImageClient:
    def __init__(
        self, socket_path="/tmp/snapshot_comm_socket/snapshot.sock", buffer_size=4096
    ):
        self.client = UnixSocketClient(socket_path, buffer_size)
        self.socket_path = socket_path
        self.buffer_size = buffer_size
        self.response_handler = ResponseHandler()

    def send_raw_image(
        self,
        camera_name: str,
        destination: str,
        num_of_frames: int,
        base_idx: int,
        timestamp: str = None,
        color: int = 0,
    ):
        # Use provided timestamp or generate new one (UTC+3)
        if timestamp is None:
            from datetime import timedelta

            utc_plus_3 = datetime.now() + timedelta(hours=3)
            timestamp = utc_plus_3.strftime("%d%m%Y_%H%M%S")

        # Ensure destination ends with /
        if not destination.endswith("/"):
            destination += "/"

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
                    # For multiple frames with both cameras, use camera-specific prefix
                    filename_base = f"{timestamp}_"
                    path = os.path.join(destination, filename_base)
                    cmd = f"voxl-record-raw-image {cam_with_suffix} -d {path} -n {num_of_frames}"
                else:
                    # For single frame, include camera name in filename
                    filename_base = f"{timestamp}_{base_idx}_"
                    path = os.path.join(destination, filename_base)
                    cmd = f"voxl-record-raw-image {cam_with_suffix} -d {path}"

                print(f"[RawImageClient] Sending command for {cam}: {cmd}")

                try:
                    result = self.client.send_sync(cmd)
                    print(f"[RawImageClient] {cam} Result: {result}")
                    results.append({"camera": cam, "result": result, "success": True})
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
            filename_base = f"{timestamp}_"
            path = os.path.join(destination, filename_base)
            cmd = f"voxl-record-raw-image {camera_name} -d {path} -n {num_of_frames}"
        else:
            filename_base = f"{timestamp}_{base_idx}_"
            path = os.path.join(destination, filename_base)
            cmd = f"voxl-record-raw-image {camera_name} -d {path}"

        print(f"[RawImageClient] Sending command: {cmd}")

        try:
            result = self.client.send_sync(cmd)
            print(f"[RawImageClient] Result: {result}")
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
    ):
        """Send raw images - single command if no delay, multiple commands with delay"""
        import time

        # Generate timestamp once for the entire batch (UTC+3)
        from datetime import timedelta

        utc_plus_3 = datetime.now() + timedelta(hours=3)
        batch_timestamp = utc_plus_3.strftime("%d%m%Y_%H%M%S")
        print(f"[RawImageClient] Batch timestamp: {batch_timestamp}")

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
                batch_timestamp,
                color,
            )
            return [result]

        # If delay > 0, loop and call send_raw_image multiple times with single frames
        print(f"[RawImageClient] Delay {delay_s}s - {num_of_frames} separate commands")
        results = []

        for i in range(num_of_frames):
            current_idx = i  # Always start from 0 for each batch
            print(
                f"[RawImageClient] Taking raw image {i+1}/{num_of_frames} (idx: {current_idx})"
            )

            try:
                # Call send_raw_image with single frame (1) and same batch timestamp
                result = self.send_raw_image(
                    camera_name, destination, 1, current_idx, batch_timestamp, color
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
