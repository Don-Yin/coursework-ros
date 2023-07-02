#!/usr/bin/env python3

import os
from pathlib import Path
import sys
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import geometry_msgs.msg
import numpy as np

sys.path.insert(0, "src/modules")
from fcsv import FcsvParser


def convert_slicer_to_ros(point):
    scale_factor = 0.1
    point = point * scale_factor
    position_correction = np.array([-28.0, -12.5, 8])
    point = point + position_correction
    return np.array([-point[0], point[1], point[2]])


def create_marker(color, namespace, frame_id, points):
    size_scale = 0.4
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.scale.x = size_scale  # Adjust as necessary
    marker.scale.y = size_scale  # Adjust as necessary
    marker.scale.z = size_scale  # Adjust as necessary
    marker.color = color
    marker.pose.orientation.w = 1.0
    for point in points:
        p = geometry_msgs.msg.Point()
        p.x, p.y, p.z = point
        marker.points.append(p)
    return marker


def main():
    rospy.init_node("marker_publisher")
    publisher = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rospy.sleep(1)

    entires_fcsv = FcsvParser(Path("data", "entries.fcsv"))
    targets_fcsv = FcsvParser(Path("data", "targets.fcsv"))

    entries_coords = entires_fcsv.content_dataframe[["x", "y", "z"]].values
    targets_coords = targets_fcsv.content_dataframe[["x", "y", "z"]].values

    entries_coords = [convert_slicer_to_ros(p) for p in entries_coords]
    targets_coords = [convert_slicer_to_ros(p) for p in targets_coords]

    frame_id = "base_link"

    # Create and publish markers
    entries_marker = create_marker(
        ColorRGBA(1.0, 0, 0, 1.0), "entries", frame_id, entries_coords
    )
    targets_marker = create_marker(
        ColorRGBA(0, 1.0, 0, 1.0), "targets", frame_id, targets_coords
    )

    publisher.publish(entries_marker)
    publisher.publish(targets_marker)

    rospy.spin()


def test():
    rospy.init_node("marker_publisher")
    publisher = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rospy.sleep(1)

    coords = np.array([[10, 10, 10]])
    frame_id = "base_link"

    marker = create_marker(ColorRGBA(1.0, 0, 0, 1.0), "entries", frame_id, coords)

    publisher.publish(marker)

    rospy.spin()


if __name__ == "__main__":
    main()
    # test()
