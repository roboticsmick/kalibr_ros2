"""
ImuDatasetReader.py — ROS2 port of the original Kalibr ROS1 bag reader.

Changes from original:
  - Replaced 'import rosbag' with rosbag2_py SequentialReader
  - Replaced internal _get_connections/_get_indexes/_read_message with
    a one-pass load into self._messages list
  - Updated timestamp fields: .secs/.nsecs → .sec/.nanosec
  - Updated stamp.to_sec() → sec + nanosec/1e9
"""

import os
import sm
import numpy as np
import aslam_cv as acv

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


class BagImuDatasetReaderIterator(object):
    def __init__(self, dataset, indices=None):
        self.dataset = dataset
        if indices is None:
            self.indices = np.arange(dataset.numMessages())
        else:
            self.indices = indices
        self.iter = self.indices.__iter__()

    def __iter__(self):
        return self

    def next(self):
        # required for python 2.x compatibility
        idx = next(self.iter)
        return self.dataset.getMessage(idx)

    def __next__(self):
        idx = next(self.iter)
        return self.dataset.getMessage(idx)


class BagImuDatasetReader(object):
    def __init__(self, bagfile, imutopic, bag_from_to=None, perform_synchronization=False):
        self.bagfile = bagfile
        self.topic = imutopic
        self.perform_synchronization = perform_synchronization

        if imutopic is None:
            raise RuntimeError(
                "Please pass in a topic name referring to the IMU stream in the bag file: {}".format(bagfile))

        # --- Load all messages from the bag for this topic ---
        reader = _open_bag(bagfile)
        type_map = {info.name: info.type for info in reader.get_all_topics_and_types()}

        if imutopic not in type_map:
            raise RuntimeError("Could not find topic '{}' in '{}'.".format(imutopic, bagfile))

        msg_type_str = type_map[imutopic]
        msg_type = get_message(msg_type_str)

        # _messages[i] = (data_msg, bag_stamp_nanosec)
        self._messages = []
        while reader.has_next():
            topic, raw_data, t_ns = reader.read_next()
            if topic == imutopic:
                msg = deserialize_message(raw_data, msg_type)
                self._messages.append((msg, t_ns))

        if not self._messages:
            raise RuntimeError("No messages found on topic '{}' in '{}'.".format(imutopic, bagfile))

        self.indices = np.arange(len(self._messages))

        # Sort by header stamp
        self.indices = self.sortByTime(self.indices)

        # Optionally restrict to a time window [bag_from_to[0], bag_from_to[1]] seconds
        if bag_from_to:
            self.indices = self.truncateIndicesFromTime(self.indices, bag_from_to)

    def _header_stamp_sec(self, msg):
        """Return header timestamp as float seconds (ROS2: .sec + .nanosec/1e9)."""
        return msg.header.stamp.sec + msg.header.stamp.nanosec / 1.0e9

    def _header_stamp_ns(self, msg):
        """Return header timestamp as integer nanoseconds."""
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
        sm.logWarn("BagImuDatasetReader: truncated {0} / {1} messages.".format(
            len(indices) - len(valid_indices), len(indices)))
        return valid_indices

    def __iter__(self):
        return self.readDataset()

    def readDataset(self):
        return BagImuDatasetReaderIterator(self, self.indices)

    def readDatasetShuffle(self):
        indices = self.indices.copy()
        np.random.shuffle(indices)
        return BagImuDatasetReaderIterator(self, indices)

    @property
    def index(self):
        """Alias for self.indices — used by IccSensors to count messages."""
        return self.indices

    def numMessages(self):
        return len(self.indices)

    def getMessage(self, idx):
        msg, t_ns = self._messages[idx]
        if self.perform_synchronization:
            timestamp = acv.Time(self.timestamp_corrector.getLocalTime(
                self._header_stamp_sec(msg)))
        else:
            timestamp = acv.Time(msg.header.stamp.sec, msg.header.stamp.nanosec)

        omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        alpha = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        return (timestamp, omega, alpha)
