"""
ImageDatasetReader.py — ROS2 port of the original Kalibr ROS1 bag reader.

Changes from original:
  - Replaced 'import rosbag' with rosbag2_py SequentialReader
  - Replaced internal _get_connections/_get_indexes/_read_message with
    a one-pass load into self._messages list
  - Updated timestamp fields: .secs/.nsecs → .sec/.nanosec
  - Updated stamp.to_sec() → sec + nanosec/1e9
  - Message type detection uses type_map string instead of data._type
  - 'from cv_bridge import CvBridge' (ROS2 cv_bridge unchanged API)
"""

import cv2
import os
import numpy as np
import aslam_cv as acv
import sm

from cv_bridge import CvBridge

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def _open_bag(bagfile):
    """Open a ROS2 bag file, auto-detecting storage format (mcap or sqlite3)."""
    storage_options = rosbag2_py.StorageOptions(uri=bagfile, storage_id='')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


class BagImageDatasetReaderIterator(object):
    def __init__(self, dataset, indices=None):
        self.dataset = dataset
        if indices is None:
            self.indices = np.arange(dataset.numImages())
        else:
            self.indices = indices
        self.iter = self.indices.__iter__()

    def __iter__(self):
        return self

    def next(self):
        # required for python 2.x compatibility
        idx = next(self.iter)
        return self.dataset.getImage(idx)

    def __next__(self):
        idx = next(self.iter)
        return self.dataset.getImage(idx)


class BagImageDatasetReader(object):
    def __init__(self, bagfile, imagetopic, bag_from_to=None, perform_synchronization=False, bag_freq=None):
        self.bagfile = bagfile
        self.topic = imagetopic
        self.perform_synchronization = perform_synchronization
        self.CVB = CvBridge()

        if imagetopic is None:
            raise RuntimeError(
                "Please pass in a topic name referring to the image stream in the bag file: {}".format(bagfile))

        # --- Load all messages from the bag for this topic ---
        reader = _open_bag(bagfile)
        type_map = {info.name: info.type for info in reader.get_all_topics_and_types()}

        if imagetopic not in type_map:
            raise RuntimeError("Could not find topic '{}' in '{}'.".format(imagetopic, bagfile))

        msg_type_str = type_map[imagetopic]
        # Store the ROS2 type string for later dispatch (replaces data._type)
        self._msg_type_str = msg_type_str
        msg_type = get_message(msg_type_str)

        # _messages[i] = (data_msg, bag_stamp_nanosec)
        self._messages = []
        while reader.has_next():
            topic, raw_data, t_ns = reader.read_next()
            if topic == imagetopic:
                msg = deserialize_message(raw_data, msg_type)
                self._messages.append((msg, t_ns))

        if not self._messages:
            raise RuntimeError("No messages found on topic '{}' in '{}'.".format(imagetopic, bagfile))

        self.indices = np.arange(len(self._messages))

        # Sort by header stamp
        self.indices = self.sortByTime(self.indices)

        if bag_from_to:
            self.indices = self.truncateIndicesFromTime(self.indices, bag_from_to)

        if bag_freq:
            self.indices = self.truncateIndicesFromFreq(self.indices, bag_freq)

    def _header_stamp_sec(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec / 1.0e9

    def _header_stamp_ns(self, msg):
        return int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)

    def sortByTime(self, indices):
        self.timestamp_corrector = sm.DoubleTimestampCorrector()
        timestamps = []
        for idx in self.indices:
            msg, t_ns = self._messages[idx]
            timestamp = self._header_stamp_ns(msg)
            timestamps.append(timestamp)
            if self.perform_synchronization:
                self.timestamp_corrector.correctTimestamp(
                    self._header_stamp_sec(msg),
                    t_ns / 1.0e9)

        sorted_tuples = sorted(zip(timestamps, indices))
        sorted_indices = [t[1] for t in sorted_tuples]
        return sorted_indices

    def truncateIndicesFromTime(self, indices, bag_from_to):
        timestamps = []
        for idx in self.indices:
            msg, _ = self._messages[idx]
            timestamps.append(self._header_stamp_sec(msg))

        bagstart = min(timestamps)
        baglength = max(timestamps) - bagstart

        if bag_from_to[0] >= bag_from_to[1]:
            raise RuntimeError("Bag start time must be smaller than end time.")
        if bag_from_to[0] < 0.0:
            sm.logWarn("Bag start time of {0} s is smaller 0".format(bag_from_to[0]))
        if bag_from_to[1] > baglength:
            sm.logWarn("Bag end time of {0} s is bigger than the total length of {1} s".format(
                bag_from_to[1], baglength))

        valid_indices = []
        for idx, timestamp in enumerate(timestamps):
            if (bagstart + bag_from_to[0]) <= timestamp <= (bagstart + bag_from_to[1]):
                valid_indices.append(idx)
        sm.logWarn("BagImageDatasetReader: truncated {0} / {1} images (from-to).".format(
            len(indices) - len(valid_indices), len(indices)))
        return valid_indices

    def truncateIndicesFromFreq(self, indices, freq):
        if freq < 0.0:
            raise RuntimeError("Frequency {0} Hz is smaller 0".format(freq))

        timestamp_last = -1
        valid_indices = []
        for idx in self.indices:
            msg, _ = self._messages[idx]
            timestamp = self._header_stamp_sec(msg)
            if timestamp_last < 0.0:
                timestamp_last = timestamp
                valid_indices.append(idx)
                continue
            if (timestamp - timestamp_last) >= 1.0 / freq:
                timestamp_last = timestamp
                valid_indices.append(idx)
        sm.logWarn("BagImageDatasetReader: truncated {0} / {1} images (frequency)".format(
            len(indices) - len(valid_indices), len(indices)))
        return valid_indices

    def __iter__(self):
        return self.readDataset()

    def readDataset(self):
        return BagImageDatasetReaderIterator(self, self.indices)

    def readDatasetShuffle(self):
        indices = self.indices.copy()
        np.random.shuffle(indices)
        return BagImageDatasetReaderIterator(self, indices)

    @property
    def index(self):
        """Alias for self.indices — used by IccSensors to count images."""
        return self.indices

    def numImages(self):
        return len(self.indices)

    def getImage(self, idx):
        msg, t_ns = self._messages[idx]
        if self.perform_synchronization:
            timestamp = acv.Time(self.timestamp_corrector.getLocalTime(
                self._header_stamp_sec(msg)))
        else:
            timestamp = acv.Time(msg.header.stamp.sec, msg.header.stamp.nanosec)

        # Dispatch on ROS2 message type string (replaces data._type attribute)
        msg_type = self._msg_type_str

        if msg_type == 'sensor_msgs/msg/CompressedImage':
            img_data = np.array(self.CVB.compressed_imgmsg_to_cv2(msg))
            if len(img_data.shape) > 2 and img_data.shape[2] == 3:
                img_data = cv2.cvtColor(img_data, cv2.COLOR_BGR2GRAY)

        elif msg_type == 'sensor_msgs/msg/Image':
            enc = msg.encoding
            if enc in ('16UC1', 'mono16'):
                image_16u = np.array(self.CVB.imgmsg_to_cv2(msg))
                img_data = (image_16u / 256).astype('uint8')
            elif enc in ('8UC1', 'mono8'):
                img_data = np.array(self.CVB.imgmsg_to_cv2(msg))
            elif enc in ('8UC3', 'bgr8'):
                img_data = np.array(self.CVB.imgmsg_to_cv2(msg))
                img_data = cv2.cvtColor(img_data, cv2.COLOR_BGR2GRAY)
            elif enc == 'rgb8':
                img_data = np.array(self.CVB.imgmsg_to_cv2(msg))
                img_data = cv2.cvtColor(img_data, cv2.COLOR_RGB2GRAY)
            elif enc in ('8UC4', 'bgra8'):
                img_data = np.array(self.CVB.imgmsg_to_cv2(msg))
                img_data = cv2.cvtColor(img_data, cv2.COLOR_BGRA2GRAY)
            elif enc == 'bayer_rggb8':
                img_data = np.array(self.CVB.imgmsg_to_cv2(msg))
                img_data = cv2.cvtColor(img_data, cv2.COLOR_BAYER_BG2GRAY)
            elif enc == 'bayer_bggr8':
                img_data = np.array(self.CVB.imgmsg_to_cv2(msg))
                img_data = cv2.cvtColor(img_data, cv2.COLOR_BAYER_RG2GRAY)
            elif enc == 'bayer_gbrg8':
                img_data = np.array(self.CVB.imgmsg_to_cv2(msg))
                img_data = cv2.cvtColor(img_data, cv2.COLOR_BAYER_GR2GRAY)
            elif enc == 'bayer_grbg8':
                img_data = np.array(self.CVB.imgmsg_to_cv2(msg))
                img_data = cv2.cvtColor(img_data, cv2.COLOR_BAYER_GB2GRAY)
            else:
                raise RuntimeError(
                    "Unsupported Image Encoding: '{}'\nSupported: "
                    "16UC1/mono16, 8UC1/mono8, 8UC3/bgr8, rgb8, 8UC4/bgra8, "
                    "bayer_rggb8, bayer_bggr8, bayer_gbrg8, bayer_grbg8".format(enc))
        else:
            raise RuntimeError(
                "Unsupported Image Type: '{}'\nSupported: "
                "sensor_msgs/msg/CompressedImage, sensor_msgs/msg/Image".format(msg_type))

        return (timestamp, img_data)
