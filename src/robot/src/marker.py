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


def create_marker(color, namespace, frame_id, points):
    scale = 1
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.scale.x = scale  # Adjust as necessary
    marker.scale.y = scale  # Adjust as necessary
    marker.scale.z = scale  # Adjust as necessary
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

    # Assume that all points are defined in the 'world' frame
    frame_id = "world"

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

    # Assume that all points are defined in the 'world' frame
    frame_id = "world"

    # Create and publish markers
    marker = create_marker(ColorRGBA(1.0, 0, 0, 1.0), "entries", frame_id, coords)

    publisher.publish(marker)

    rospy.spin()


if __name__ == "__main__":
    # main()
    test()
