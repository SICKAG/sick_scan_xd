# Subscribe to sick_scan_xd pointcloud and laserscan messages and save to json-file

import json
import numpy as np
from collections import namedtuple
from sick_scan_xd_simu_report import SickScanXdMsgStatus

Jitter = namedtuple("Jitter", "angle_deg, angle_rad, range, velocity, acceleration, intensity") # declaration of a jitter for comparison for pointcloud, laserscan and imu data (const values)
jitter = Jitter(angle_deg = 0.1, angle_rad = np.deg2rad(0.1), range = 0.001, velocity = 0.1, acceleration = 0.1, intensity = 0.1) # tolerated jitter for comparison for pointcloud, laserscan and imu data

ros1_found = False
try:
    import rospy
    from sensor_msgs.msg import PointCloud2, PointField, LaserScan, Imu
    ros1_found = True
except ModuleNotFoundError as exc:
    ros1_found = False
class Node:
    pass



def ros_init(os_name = "linux", ros_version = "noetic", node_name = "sick_scan_xd_subscriber"):
    """
    ros initialization, delegates to ros initialization function depending on system and ros version
    """
    if ros1_found and ros_version == "noetic":
        return rospy.init_node(node_name)
    else:
        print(f"## ERROR sick_scan_xd_subscriber.ros_init(): ros version {ros_version} not found or not supported")

class RefPointcloudMsg:
    """
    Subset of a PointCloud2 message. Used to verify received PointCloud2 messages against reference (groundtruth) values
    """
    def __init__(self, msg = None):
        """
        Initializing constructor
        """
        self.seq = msg.header.seq if msg is not None else 0
        self.frame_id = msg.header.frame_id if msg is not None else ""
        self.width = msg.width if msg is not None else 0
        self.height = msg.height if msg is not None else 0
        self.point_step = msg.point_step if msg is not None else 0
        self.row_step = msg.row_step if msg is not None else 0
        self.fields = msg.fields if msg is not None else []
        self.data = msg.data if msg is not None else []
    def to_dictionary(self):
        """
        Converts and returns all member variables to a serializable dictionary
        """
        fields = []
        for field in self.fields:
            fields.append({"name": field.name, "offset": field.offset, "datatype": field.datatype, "count": field.count})
        hex_data_str = "".join("{:02x}".format(x) for x in self.data)
        return {
            "seq": self.seq,
            "frame_id": self.frame_id,
            "width": self.width,
            "height": self.height,
            "point_step": self.point_step,
            "row_step": self.row_step,
            "fields": fields,
            "data": hex_data_str,
        }
    def identical_messages(self, reference_msg, received_msg):
        """
        Returns True, if reference_msg and received_msg are identical except for the sequence counter and small floating point differences.
        """
        return (reference_msg["frame_id"] == received_msg["frame_id"]) and \
            (reference_msg["width"] == received_msg["width"]) and \
            (reference_msg["height"] == received_msg["height"]) and \
            (reference_msg["point_step"] == received_msg["point_step"]) and \
            (reference_msg["row_step"] == received_msg["row_step"]) and \
            (self.identical_fields(reference_msg["fields"], received_msg["fields"])) and \
            (self.identical_pointclouds(reference_msg["data"], received_msg["data"], reference_msg["width"], reference_msg["height"], reference_msg["point_step"], reference_msg["row_step"], reference_msg["fields"]))
    def identical_fields(self, reference_fields, received_fields):
        """
        Returns True, if reference_fields and received_fields are identical
        """
        if (len(reference_fields) != len(received_fields)):
            return False
        for field_idx in range(len(reference_fields)):
            if reference_fields[field_idx]["name"] != received_fields[field_idx]["name"] or \
                reference_fields[field_idx]["offset"] != received_fields[field_idx]["offset"] or \
                reference_fields[field_idx]["datatype"] != received_fields[field_idx]["datatype"] or \
                reference_fields[field_idx]["count"] != received_fields[field_idx]["count"]:
                return False
        return True
    def convert_pointcloud_field_datatype(self, field_datatype):
            field_element_size = 0
            field_element_dtype = 0
            if field_datatype == 1: # compare INT8
                field_element_size = 1
                field_element_dtype = np.int8
            elif field_datatype == 2: # compare UINT8
                field_element_size = 1
                field_element_dtype = np.uint8
            elif field_datatype == 3: # compare INT16
                field_element_size = 2
                field_element_dtype = np.int16
            elif field_datatype == 4: # compare UINT16
                field_element_size = 2
                field_element_dtype = np.uint16
            elif field_datatype == 5: # compare INT32
                field_element_size = 4
                field_element_dtype = np.int32
            elif field_datatype == 6: # compare UINT32
                field_element_size = 4
                field_element_dtype = np.uint32
            elif field_datatype == 7: # compare FLOAT32
                field_element_size = 4
                field_element_dtype = np.float32
            elif field_datatype == 8: # compare FLOAT64
                field_element_size = 8
                field_element_dtype = np.float64
            return field_element_dtype, field_element_size
    def identical_pointclouds(self, reference_pointcloud_hex, received_pointcloud_hex, pointcloud_width, pointcloud_height, pointcloud_point_step, pointcloud_row_step, field_description):
        if len(reference_pointcloud_hex) != len(received_pointcloud_hex):
            return False
        # Cache field element datatypes
        fields_element_size = []
        fields_element_dtype = []
        fields_element_jitter = []
        for field_idx in range(len(field_description)):
            field_element_dtype, field_element_size = self.convert_pointcloud_field_datatype(field_description[field_idx]["datatype"])
            field_element_jitter = jitter.range
            if field_element_size <= 0:
                print(f"## ERROR RefPointcloudMsg.identical_pointclouds: unexpected field: count={field_description[field_idx]['count']}, datatype={field_description[field_idx]['datatype']}")
                return False
            fields_element_size.append(field_element_size)
            fields_element_dtype.append(field_element_dtype)
            fields_element_jitter.append(field_element_jitter)
        # Convert messages to bytes
        ref_pc_bytes = bytes.fromhex(reference_pointcloud_hex)
        msg_pc_bytes = bytes.fromhex(received_pointcloud_hex)
        for row in range(pointcloud_height):
            for col in range(pointcloud_width):
                for field_idx in range(len(field_description)):
                    field_start_idx = row * pointcloud_row_step + col * pointcloud_point_step + field_description[field_idx]["offset"]
                    field_end_idx = field_start_idx + fields_element_size[field_idx]
                    ref_field = np.frombuffer(ref_pc_bytes[field_start_idx:field_end_idx], dtype=fields_element_dtype[field_idx])
                    msg_field = np.frombuffer(msg_pc_bytes[field_start_idx:field_end_idx], dtype=fields_element_dtype[field_idx])
                    element_delta = np.abs(ref_field-msg_field)
                    if np.max(element_delta) > fields_element_jitter[field_idx]:
                        return False
        return True

class RefLaserscanMsg:
    """
    Subset of a LaserScan message. Used to verify received LaserScan messages against reference (groundtruth) values
    """
    def __init__(self, msg = None):
        """
        Initializing constructor
        """
        self.seq = msg.header.seq if msg is not None else 0
        self.frame_id = msg.header.frame_id if msg is not None else ""
        self.angle_min = msg.angle_min if msg is not None else 0
        self.angle_max = msg.angle_max if msg is not None else 0
        self.angle_increment = msg.angle_increment if msg is not None else 0
        self.ranges = msg.ranges if msg is not None else []
        self.intensities = msg.intensities if msg is not None else []
    def to_dictionary(self):
        """
        Converts and returns all member variables to a serializable dictionary
        """
        ranges_bytes = np.float32(self.ranges).tobytes()
        ranges_hex_str = "".join("{:02x}".format(x) for x in ranges_bytes)
        intensities_bytes = np.float32(self.intensities).tobytes()
        intensities_hex_str = "".join("{:02x}".format(x) for x in intensities_bytes)
        return {
            "seq": self.seq,
            "frame_id": self.frame_id,
            "angle_min": self.angle_min,
            "angle_max": self.angle_max,
            "angle_increment": self.angle_increment,
            "ranges": ranges_hex_str,
            "intensities": intensities_hex_str,
        }
    def identical_floats_from_hex(self, floats_hex_ref, floats_hex_msg, epsilon):
        floats_ref_bytes = bytes.fromhex(floats_hex_ref)
        floats_msg_bytes = bytes.fromhex(floats_hex_msg)
        floats_ref = np.frombuffer(floats_ref_bytes, dtype=np.float32)
        floats_msg = np.frombuffer(floats_msg_bytes, dtype=np.float32)
        if len(floats_ref) != len(floats_msg):
            return False
        floats_diff = np.max(np.abs(floats_ref - floats_msg))
        return floats_diff <= epsilon
    def identical_messages(self, reference_msg, received_msg):
        """
        Returns True, if reference_msg and received_msg are identical except for the sequence counter and small floating point differences.
        """
        return (reference_msg["frame_id"] == received_msg["frame_id"]) and \
            (np.abs(reference_msg["angle_min"] - received_msg["angle_min"]) < jitter.angle_rad) and \
            (np.abs(reference_msg["angle_max"] - received_msg["angle_max"]) < jitter.angle_rad) and \
            (np.abs(reference_msg["angle_increment"] - received_msg["angle_increment"]) < jitter.angle_rad) and \
            (self.identical_floats_from_hex(reference_msg["ranges"], received_msg["ranges"], jitter.range)) and \
            (self.identical_floats_from_hex(reference_msg["intensities"], received_msg["intensities"], jitter.intensity))

class RefImuMsg:
    """
    Subset of a IMU message. Used to verify received IMU messages against reference (groundtruth) values
    """
    def __init__(self, msg = None):
        """
        Initializing constructor
        """
        self.seq = msg.header.seq if msg is not None else 0
        self.frame_id = msg.header.frame_id if msg is not None else ""
        self.orientation = msg.orientation if msg is not None else []
        self.orientation_covariance = msg.orientation_covariance if msg is not None else []
        self.angular_velocity = msg.angular_velocity if msg is not None else []
        self.angular_velocity_covariance = msg.angular_velocity_covariance if msg is not None else []
        self.linear_acceleration = msg.linear_acceleration if msg is not None else []
        self.linear_acceleration_covariance = msg.linear_acceleration_covariance if msg is not None else []
    def to_dictionary(self):
        """
        Converts and returns all member variables to a serializable dictionary
        """
        return {
            "seq": self.seq,
            "frame_id": self.frame_id,
            "orientation": [ self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w ],
            "orientation_covariance": self.orientation_covariance,
            "angular_velocity": [ self.angular_velocity.x, self.angular_velocity.y, self.angular_velocity.z ],
            "angular_velocity_covariance": self.angular_velocity_covariance,
            "linear_acceleration": [ self.linear_acceleration.x, self.linear_acceleration.y, self.linear_acceleration.z ],
            "linear_acceleration_covariance": self.linear_acceleration_covariance,
        }
    def identical_messages(self, reference_msg, received_msg):
        """
        Returns True, if reference_msg and received_msg are identical except for the sequence counter and small floating point differences.
        """
        return (reference_msg["frame_id"] == received_msg["frame_id"]) and \
            (np.linalg.norm(np.float32(reference_msg["orientation"]) - np.float32(received_msg["orientation"])) < jitter.angle_rad) and \
            (np.linalg.norm(np.float32(reference_msg["orientation_covariance"]) - np.float32(received_msg["orientation_covariance"])) < jitter.angle_rad**2) and \
            (np.linalg.norm(np.float32(reference_msg["angular_velocity"]) - np.float32(received_msg["angular_velocity"])) < jitter.velocity) and \
            (np.linalg.norm(np.float32(reference_msg["angular_velocity_covariance"]) - np.float32(received_msg["angular_velocity_covariance"])) < jitter.velocity**2) and \
            (np.linalg.norm(np.float32(reference_msg["linear_acceleration"]) - np.float32(received_msg["linear_acceleration"])) < jitter.acceleration) and \
            (np.linalg.norm(np.float32(reference_msg["linear_acceleration_covariance"]) - np.float32(received_msg["linear_acceleration_covariance"])) < jitter.acceleration**2)

class SickScanXdSubscriber(Node):

    def __init__(self, pointcloud_subscriber_topic = "", laserscan_subscriber_topic = "", imu_subscriber_topic = ""):
        self.subscriber_topic = ""
        self.subscriber = None
        self.messages_received = []
        if ros1_found and len(pointcloud_subscriber_topic) > 0:
            self.subscriber_topic = pointcloud_subscriber_topic
            self.subscriber = rospy.Subscriber(pointcloud_subscriber_topic, PointCloud2, self.pointcloud_listener_callback, queue_size=16*12*3) # max. queue size for multiScan: 16 layer, 12 segments, 3 echos
            print(f"SickScanXdSubscriber: subscribed to PointCloud2 messages on topic \"{pointcloud_subscriber_topic}\"")
        elif ros1_found and len(laserscan_subscriber_topic) > 0:
            self.subscriber_topic = laserscan_subscriber_topic
            self.subscriber = rospy.Subscriber(laserscan_subscriber_topic, LaserScan, self.laserscan_listener_callback, queue_size=16*12*3) # max. queue size for multiScan: 16 layer, 12 segments, 3 echos
            print(f"SickScanXdSubscriber: subscribed to LaserScan messages on topic \"{laserscan_subscriber_topic}\"")
        elif ros1_found and len(imu_subscriber_topic) > 0:
            self.subscriber_topic = imu_subscriber_topic
            self.subscriber = rospy.Subscriber(imu_subscriber_topic, Imu, self.imu_listener_callback, queue_size=1024)
            print(f"SickScanXdSubscriber: subscribed to IMU messages on topic \"{imu_subscriber_topic}\"")
        else:
            print("## ERROR SickScanXdSubscriber: ros version and/or topics not supported or invalid")
    
    def pointcloud_listener_callback(self, msg):
        self.messages_received.append(RefPointcloudMsg(msg))

    def laserscan_listener_callback(self, msg):
        self.messages_received.append(RefLaserscanMsg(msg))

    def imu_listener_callback(self, msg):
        self.messages_received.append(RefImuMsg(msg))

    def export_dictionary(self, dict):
        if len(self.messages_received) > 0:
            dict[self.subscriber_topic] = []
            for msg in self.messages_received:
                dict[self.subscriber_topic].append(msg.to_dictionary())

class SickScanXdMonitor():

    def __init__(self, config, run_ros_init):
        self.ros_node = None
        self.laserscan_subscriber = []
        self.pointcloud_subscriber = []
        self.imu_subscriber = []
        self.reference_messages_jsonfile = f"{config.log_folder}/{config.reference_messages_jsonfile}"

        if run_ros_init:
            self.ros_node = ros_init(os_name = config.os_name, ros_version = config.ros_version, node_name = "sick_scan_xd_simu")
        for topic in config.sick_scan_xd_pointcloud_topics:
            self.pointcloud_subscriber.append(SickScanXdSubscriber(pointcloud_subscriber_topic = topic))
        for topic in config.sick_scan_xd_laserscan_topics:
            self.laserscan_subscriber.append(SickScanXdSubscriber(laserscan_subscriber_topic = topic))
        for topic in config.sick_scan_xd_imu_topics:
            self.imu_subscriber.append(SickScanXdSubscriber(imu_subscriber_topic = topic))

    def export_received_messages(self):
        messages = {}
        messages["RefLaserscanMsg"] = {}
        messages["RefPointcloudMsg"] = {}
        messages["RefImuMsg"] = {}
        num_messages = 0
        for subscriber in self.laserscan_subscriber:
            subscriber.export_dictionary(messages["RefLaserscanMsg"])
            num_messages += len(subscriber.messages_received)
        for subscriber in self.pointcloud_subscriber:
            subscriber.export_dictionary(messages["RefPointcloudMsg"])
            num_messages += len(subscriber.messages_received)
        for subscriber in self.imu_subscriber:
            subscriber.export_dictionary(messages["RefImuMsg"])
            num_messages += len(subscriber.messages_received)
        return num_messages, messages

    def export_received_messages_to_jsonfile(self, jsonfile):
        num_messages, messages = self.export_received_messages()
        if num_messages > 0:
            with open(jsonfile, "w") as file_stream:
                json.dump(messages, file_stream, indent=2)
                print(f"SickScanXdMonitor: {num_messages} messages exported to file \"{jsonfile}\"")
        else:
            print(f"## ERROR SickScanXdMonitor.export_received_messages(): no messages received, file \"{jsonfile}\" not written")

    def verify_messages(self, report):
        try:
            _, received_messages = self.export_received_messages()
            reference_messages = {}
            with open(self.reference_messages_jsonfile, "r") as file_stream:
                reference_messages = json.load(file_stream)
            converter = { "RefLaserscanMsg": RefLaserscanMsg(), "RefPointcloudMsg": RefPointcloudMsg(), "RefImuMsg": RefImuMsg() }
            num_messages_verified = 0
            for msg_type in reference_messages.keys():
                for topic in reference_messages[msg_type].keys():
                    for ref_msg in reference_messages[msg_type][topic]:
                        if topic not in received_messages[msg_type] or not self.find_message(ref_msg, received_messages[msg_type][topic], converter[msg_type]):
                            ref_msg_seq = ref_msg["seq"]
                            ref_msg_frame_id = ref_msg["frame_id"]
                            self.print_message(report, SickScanXdMsgStatus.ERROR, f"## ERROR in SickScanXdMonitor.verify_messages(): reference message not found in received messages (msg type: {msg_type}, topic: {topic}, seq: {ref_msg_seq} frame_id: {ref_msg_frame_id}), test failed")
                            return False
                        num_messages_verified = num_messages_verified + 1
            self.print_message(report, SickScanXdMsgStatus.INFO, f"SickScanXdMonitor.verify_messages(): {num_messages_verified} reference messages verified.")
            # print(f"SickScanXdMonitor.verify_messages(): received_messages = {received_messages}")
            # print(f"SickScanXdMonitor.verify_messages(): reference_messages = {reference_messages}")
            # with open("verify_messages_received.json", "w") as file_stream:
            #     json.dump(received_messages, file_stream, indent=2)
            # with open("verify_messages_reference.json", "w") as file_stream:
            #     json.dump(reference_messages, file_stream, indent=2)
        except Exception as exc:
            self.print_message(report, SickScanXdMsgStatus.ERROR, f"## ERROR in SickScanXdMonitor.verify_messages(): exception {exc}")
            return False
        return True
    
    def print_message(self, report, msg_status, message):
        report.append_message(msg_status, message)
        print(message)

    def find_message(self, reference_msg, received_messages, converter):
        for received_message in received_messages:
            if converter.identical_messages(reference_msg, received_message):
                return True
        return False

if __name__ == '__main__':
    ros_node = ros_init()
    sick_scan_xd_subscriber = SickScanXdSubscriber(pointcloud_subscriber_topic = "cloud_all_fields_fullframe")
    if ros1_found:
        # rospy.spin()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
    else:
        print("## ERROR sick_scan_xd_subscriber: expected ros version not found or not supported")
