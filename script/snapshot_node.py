# ROS node logic (subscribes to topic)# snapshot_node.py

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import logging
import os
import sys
from rospy import loginfo, logwarn, logerr

# Add current script directory to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from commands import CommandType, normalize_command_type
from datetime import datetime, timedelta, timezone
from snapshot_client import SnapshotClient
from recorder_client import RawImageClient

# TODO: load the temp directory where iamges are saved for report


class UTCPlus3Formatter(logging.Formatter):
    COLOR_CODES = {
        logging.DEBUG: "\033[94m",  # Blue
        logging.INFO: "\033[92m",  # Green
        logging.WARNING: "\033[93m",  # Yellow
        logging.ERROR: "\033[91m",  # Red
        logging.CRITICAL: "\033[95m",  # Magenta
    }
    RESET_CODE = "\033[0m"

    def __init__(self, node_name="wds_snapshot_client_node", use_color=False):
        super().__init__()
        self.node_name = node_name
        self.use_color = use_color

    def formatTime(self, record, datefmt=None):
        dt = datetime.fromtimestamp(record.created) + timedelta(hours=3)
        return dt.strftime("%H:%M:%S") + f":{int(record.msecs):03d}"

    # def formatTime(self, record, datefmt=None):
    #     dt = datetime.datetime.fromtimestamp(record.created) + datetime.timedelta(hours=3)
    #     if datefmt:
    #         return dt.strftime(datefmt)
    #     else:
    #         return dt.strftime("%Y-%m-%d %H:%M:%S")

    def format(self, record):
        time_str = self.formatTime(record)
        level_str = f"[{record.levelname:>5}]"
        line = f"[{time_str}] {level_str} [{self.node_name}] {record.getMessage()}"
        if self.use_color:
            color = self.COLOR_CODES.get(record.levelno, "")
            return f"{color}{line}{self.RESET_CODE}"
        return line


try:
    from py_csv_logger import CsvLogger, ImageMetadata
except ImportError as e:
    rospy.logerr(f"Failed to import C++ CsvLogger module: {e}")
    # Handle the error, maybe exit or run in a mode without logging
    CsvLogger, ImageMetadata = None, None


class SnapshotNode:
    def __init__(self):
        rospy.init_node("wds_snapshot_client_node", anonymous=False)

        # Store node name for logging
        self.node_name = rospy.get_name()

        # ---- Params (with defaults) ----

        self.base_destination = rospy.get_param(
            "~base_destination", "/data/wds/barcode_scan/"
        )
        self.folder = os.getenv("FOLDER", "")
        self.frame_delta = float(os.getenv("FRAME_DELTA", "0"))
        self.num_images = int(os.getenv("NUM_IMAGES_PER_COMMAND", "3"))
        self.cmd_type = os.getenv("COMMAND_TYPE", "RAW_IMAGE")
        self.color = int(os.getenv("COLOR", "0"))
        self.destination = (
            os.path.join(self.base_destination, self.folder)
            if self.folder
            else self.base_destination
        )
        self.log_file_path = rospy.get_param("log_file_path", "/data/wds/logs")
        # TODO: we need to fidn latest fodlr to save to since we are suppsoed to save in the mission flow logs
             #  --- Set up logging ---
        self.setup_logging()

        # Instantiate the C++ CsvLogger
        self.csv_logger = None
        if CsvLogger is not None:
            try:
                self.csv_logger = CsvLogger(self.destination)
                if not self.csv_logger.initialize():
                    logging.error("Failed to initialize C++ CsvLogger!")
                    self.csv_logger = None
                else:
                    logging.info("C++ CsvLogger initialized successfully.")
            except Exception as e:
                logging.error(f"Error instantiating C++ CsvLogger: {e}")
                self.csv_logger = None

        socket_path = rospy.get_param(
            "~socket_path", "/tmp/snapshot_comm_socket/snapshot.sock"
        )
        buffer_size = int(rospy.get_param("~buffer_size", 4096))
        self.idx = 0
        # ---- Clients ----
        self.snapshot_client = SnapshotClient(
            socket_path=socket_path, buffer_size=buffer_size
        )
        self.raw_client = RawImageClient(
            socket_path=socket_path, buffer_size=buffer_size
        )

        # ---- Subscriptions ----
        self.sub = rospy.Subscriber(
            "/corvus/payload/take_picture", String, self.take_picture_callback
        )

        # QVIO pose subscriber for position logging
        self.qvio_sub = rospy.Subscriber(
            "/corvus/pose/fixed", PoseStamped, self.qvio_pose_callback
        )
        self.latest_qvio_pose = None

   
        logging.info(
            f"[SnapshotNode] Started. dest={self.destination} count={self.num_images} "
            f"type={self.cmd_type} socket={socket_path}"
        )

    def setup_logging(self):
        """
        Configure logging to save  Python logs,
        with timestamps shifted 3 hours forward.
        """
        try:
            print(f"[DEBUG] Setting up logging...")
            print(f"[DEBUG] log_file_path parameter: {self.log_file_path}")
            print(f"[DEBUG] destination path: {self.destination}")
            print(f"[DEBUG] FOLDER env var: {self.folder}")
            # Get the path from parameter
            log_path_param = self.log_file_path

            # Determine log directory (if param is a dir or a dummy filepath)
            if os.path.isdir(log_path_param):
                log_dir = log_path_param
            else:
                log_dir = os.path.dirname(log_path_param)

            os.makedirs(log_dir, exist_ok=True)
            print(f"[DEBUG] Created log directory: {log_dir}")
            print(f"[DEBUG] Directory exists: {os.path.exists(log_dir)}")
            print(f"[DEBUG] Directory writable: {os.access(log_dir, os.W_OK)}")
            logging.debug(f"Log directory ensured: {log_dir}")

            # Generate timestamped filename
            prefix = "wds_snapshot_client"
            timestamp = (datetime.now(timezone.utc) + timedelta(hours=3)).strftime(
                "%d%m%Y_T%H%M%S_%f"
            )[:-3]
            filename = f"{prefix}_{timestamp}.log"
            full_log_path = os.path.join(log_dir, filename)
            self.log_file_path = full_log_path
            print(f"[DEBUG] Full log path: {full_log_path}")
            print(f"[DEBUG] Attempting to create log file...")

            # Formatters
            plain_formatter = UTCPlus3Formatter(
                node_name="wds_snapshot_client_node", use_color=False
            )
            color_formatter = UTCPlus3Formatter(
                node_name="wds_snapshot_client_node", use_color=True
            )

            # Handlers
            file_handler = logging.FileHandler(full_log_path, mode="a")
            file_handler.setFormatter(plain_formatter)

            stream_handler = logging.StreamHandler()
            stream_handler.setFormatter(color_formatter)

            # Logger
            root_logger = logging.getLogger()
            root_logger.setLevel(logging.DEBUG)
            root_logger.handlers = []
            root_logger.addHandler(file_handler)
            root_logger.addHandler(stream_handler)

            # === File-only logger for SSH script output ===
            self.fileonly_logger = logging.getLogger("ssh_script_logger")
            self.fileonly_logger.setLevel(logging.DEBUG)
            self.fileonly_logger.propagate = False  # prevent bubbling up to console

            if not self.fileonly_logger.handlers:
                fh = logging.FileHandler(full_log_path, mode="a")
                fh.setFormatter(plain_formatter)
                self.fileonly_logger.addHandler(fh)

        except Exception as e:
            print(f"[DEBUG] LOGGING SETUP FAILED: {str(e)}")
            import traceback

            traceback.print_exc()
            logging.error(f"Failed to set up logging: {str(e)}")

    def take_picture_callback(self, msg: String):

        camera_pipeline = msg.data.strip()
        if not camera_pipeline:
            logging.warning("[SnapshotNode] Empty camera pipeline received; ignoring.")
            return

        # Use the cmd_type from environment variable (loaded in __init__)
        logging.info(
            f"[SnapshotClient] Received command: {self.cmd_type} for camera pipeline: {camera_pipeline}"
        )
        try:
            # Convert string to enum for comparison
            if self.cmd_type not in ["SNAPSHOT", "RAW_IMAGE"]:
                logging.warning(f"Unsupported command type: {self.cmd_type}")
                return

            logging.info(f"[DEBUG] Processing command type: {self.cmd_type}")

            if self.cmd_type == "SNAPSHOT":
                results = self.take_snapshots(camera_pipeline)
            elif self.cmd_type == "RAW_IMAGE":
                results = self.take_raw_image(camera_pipeline)

        except Exception as e:
            logging.error(f"[SnapshotNode] Error while processing request: {e}")

    def qvio_pose_callback(self, msg: PoseStamped):
        """Callback for QVIO pose data from /corvus/pose/fixed - just store latest pose"""
        try:
            self.latest_qvio_pose = msg
            # logging.debug(f"[SnapshotNode] QVIO pose updated: pos=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})")
        except Exception as e:
            logging.error(f"[SnapshotNode] Error processing QVIO pose: {e}")


    def shutdown(self):
        logging.info("[SnapshotNode] Shutting down...")
        try:
            self.snapshot_client.shutdown()
        except Exception:
            pass

    def take_snapshots(self, camera_pipeline):
        """Take snapshot images using the snapshot client"""
        try:

            client = SnapshotClient()
            client.send_multiple_snapshots(
                camera_pipeline, self.destination, self.num_images, 0, self.color
            )
        except Exception as e:
            logging.error(f"[SnapshotNode] Snapshot error: {e}")
            return "ERROR"

    def take_raw_image(self, camera_pipeline):
        """Take raw images using the raw image client"""
        try:
            logging.debug(f"[DEBUG] take_raw_image called with camera: {camera_pipeline}")
            logging.debug(f"[DEBUG] Destination directory: {self.destination}")
            logging.debug(f"[DEBUG] Number of images: {self.num_images}")
            logging.debug(f"[DEBUG] Starting index: 0")
            color_mode = "color" if self.color == 1 else "grey"
            logging.debug(f"[DEBUG] Color mode: {color_mode} (COLOR={self.color})")

            # Create destination directory if it doesn't exist
            os.makedirs(self.destination, exist_ok=True)
            logging.debug(f"[DEBUG] Destination directory created/verified")

            client = RawImageClient(csv_logger=self.csv_logger)


            logging.info(
                f"[SnapshotClient] Taking {self.num_images} raw images for camera pipeline: {camera_pipeline} with {self.frame_delta}s delay"
            )
            result = client.send_multiple_raw_images(
                camera_pipeline,
                self.destination,
                self.num_images,
                0,
                self.frame_delta,
                self.color,
                self.latest_qvio_pose,
            )
            logging.info(f"[DEBUG] Raw image result: {result}")

            # Implement the raw image logic here if needed
            return "SUCCESS"
        except Exception as e:
            logging.error(f"[SnapshotNode] Raw image error: {e}")
            return "ERROR"


def main():
    node = SnapshotNode()
    rospy.on_shutdown(node.shutdown)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received")
    except Exception as e:
        logging.error(f"Unexpected error: {e}")


if __name__ == "__main__":
    main()
