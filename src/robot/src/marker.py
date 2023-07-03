#!/usr/bin/env python3

from pathlib import Path
import sys
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import geometry_msgs.msg
import numpy as np
import json

sys.path.insert(0, "src/modules")
from fcsv import FcsvParser


def convert_slicer_to_ros(point):
    scale_factor = 0.08
    point = point * scale_factor
    position_correction = np.array([-27.0, -10, 2])
    point = point + position_correction
    return np.array([-point[0], point[1], point[2]])


with open(Path("assets", "entry_target_real.json"), "r") as loader:
    data = json.load(loader)

entry, target = data["original_coordinates"][0], data["original_coordinates"][1]
entry, target = np.array(entry), np.array(target)

entry = convert_slicer_to_ros(entry)
target = convert_slicer_to_ros(target)


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

    entries_coords = [i for i in entries_coords if not np.array_equal(i, entry)]
    targets_coords = [i for i in targets_coords if not np.array_equal(i, target)]

    frame_id = "base_link"

    # Create and publish markers
    entries_markers = create_marker(
        ColorRGBA(1.0, 0, 0, 1.0), "entries", frame_id, entries_coords
    )
    targets_markers = create_marker(
        ColorRGBA(0, 1.0, 0, 1.0), "targets", frame_id, targets_coords
    )

    # the chosen entry and target

    chosen_entry_marker = create_marker(
        ColorRGBA(1.0, 1.0, 0, 1.0), "chosen_entry", frame_id, [entry]
    )
    chosen_target_marker = create_marker(
        ColorRGBA(1.0, 1.0, 0, 1.0), "chosen_target", frame_id, [target]
    )

    publisher.publish(entries_markers)
    publisher.publish(targets_markers)
    publisher.publish(chosen_entry_marker)
    publisher.publish(chosen_target_marker)

    rospy.spin()


if __name__ == "__main__":
    main()
