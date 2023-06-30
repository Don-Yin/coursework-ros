"""
Unittest
"""

import unittest
import torch
import numpy as np
from src.utils.intersect import check_intersect, check_intersect_batch
from src.variables import device
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


class TestCheckIntersectBatch(unittest.TestCase):
    def setUp(self):
        self.origins = torch.tensor([[1.0, 1.0, 1.0]]).double().to(device)
        self.targets = torch.tensor([[0.2, 0.2, 0.2]]).double().to(device)

        # Define the vertices
        self.verts = torch.tensor([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        self.verts = self.verts.double().to(device)

        # Define the faces of the triangle surface
        self.faces = torch.tensor([[0, 1, 2], [0, 1, 3], [1, 2, 3], [0, 2, 3]]).to(device)

    def test_batch_intersect_runs(self):
        check_intersect_batch(self.origins, self.targets, self.verts, self.faces)

    def test_batch_non_intersect_runs(self):
        targets = torch.tensor([[2, 2, 2]]).double().to(device)
        check_intersect_batch(self.origins, targets, self.verts, self.faces)

    # intersecting scenario ---------------------------------------------------
    def test_batch_intersection_true(self):
        intersects, lengths, distances, angles = check_intersect_batch(self.origins, self.targets, self.verts, self.faces)
        self.assertTrue(intersects)

    def test_batch_intersect_length(self):
        intersects, lengths, distances, angles = check_intersect_batch(self.origins, self.targets, self.verts, self.faces)

        length = lengths.item()
        self.assertAlmostEqual(length, 1.15470039, places=3)

    def test_batch_intersect_distance(self):
        intersects, lengths, distances, angles = check_intersect_batch(self.origins, self.targets, self.verts, self.faces)
        self.assertEqual(distances, 0)

    def test_batch_intersect_angle(self):
        intersects, lengths, distances, angles = check_intersect_batch(self.origins, self.targets, self.verts, self.faces)
        self.assertEqual(angles, 0)

    # non-intersecting scenario -----------------------------------------------
    def test_batch_non_intersection_false(self):
        targets = torch.tensor([[2, 2, 2]]).double().to(device)
        intersects, lengths, distances, angles = check_intersect_batch(self.origins, targets, self.verts, self.faces)
        self.assertFalse(intersects)

    def test_batch_non_intersect_length(self):
        targets = torch.tensor([[2, 2, 2]]).double().to(device)
        intersects, lengths, distances, angles = check_intersect_batch(self.origins, targets, self.verts, self.faces)
        self.assertAlmostEqual(lengths, np.sqrt(3), places=5)  # maybe ditch this test

    def test_batch_non_intersect_distance(self):
        origins = torch.tensor([[1, 1, 2]]).double().to(device)
        targets = torch.tensor([[-0.1, -0.1, 2]]).double().to(device)
        intersects, lengths, distances, angles = check_intersect_batch(origins, targets, self.verts, self.faces)
        self.assertEqual(distances, 1)

    def test_batch_non_intersect_angle(self):
        origins = torch.tensor([[1, 1, 2]]).double().to(device)
        targets = torch.tensor([[0, 0, 2]]).double().to(device)
        intersects, lengths, distances, angles = check_intersect_batch(origins, targets, self.verts, self.faces)
        self.assertTrue(np.isnan(angles.item()))


class TestCheckIntersect(unittest.TestCase):
    def setUp(self):
        # Define the origin and direction of the ray
        self.points = (torch.tensor([1.0, 1.0, 1.0]), torch.tensor([0.2, 0.2, 0.2]))
        # self.points = (torch.tensor([1.0, 1.0, 2.0]), torch.tensor([-0.1, -0.1, 2.0]))
        self.points = (self.points[0].double().to(device), self.points[1].double().to(device))

        # Define the vertices
        self.verts = torch.tensor([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]).to(device)
        self.verts = self.verts.double().to(device)

        # Define the faces of the triangle surface
        self.faces = torch.tensor([[0, 1, 2], [0, 1, 3], [1, 2, 3], [0, 2, 3]]).to(device)

    def test_intersect_runs(self):
        check_intersect(self.points, self.verts, self.faces)

    def test_non_intersect_runs(self):
        points = (torch.tensor([1.0, 1.0, 2.0]), torch.tensor([0.0, 0.0, 2.0]))
        points = (points[0].double().to(device), points[1].double().to(device))
        check_intersect(points, self.verts, self.faces)

    # intersecting scenario ---------------------------------------------------
    def test_intersection_true(self):
        intersect, length, distance, angle = check_intersect(self.points, self.verts, self.faces)
        self.assertTrue(intersect)

    def test_intersect_length(self):
        intersect, length, distance, angle = check_intersect(self.points, self.verts, self.faces)

        length = length.item()
        self.assertAlmostEqual(length, 1.15470039, places=3)

    def test_intersection_distance(self):
        intersect, length, distance, angle = check_intersect(self.points, self.verts, self.faces)
        self.assertEqual(distance, 0)

    def test_intersect_angle(self):
        intersect, length, distance, angle = check_intersect(self.points, self.verts, self.faces)
        self.assertIsNotNone(angle)
        self.assertAlmostEqual(angle, 0, places=5)

    # non-intersecting scenario ----------------------------------------------
    def test_non_intersection_false(self):
        points = (self.points[0], torch.tensor([2, 2, 2]).to(device))
        intersect, length, distance, angle = check_intersect(points, self.verts, self.faces)
        self.assertFalse(intersect)

    def test_non_intersect_length(self):
        points = (self.points[0], torch.tensor([2, 2, 2]).to(device))
        intersect, length, distance, angle = check_intersect(points, self.verts, self.faces)
        self.assertAlmostEqual(length, np.sqrt(3), places=5)  # maybe ditch this test

    def test_non_intersect_distance(self):
        points = (torch.tensor([1.0, 1.0, 2.0]), torch.tensor([-0.1, -0.1, 2.0]))
        points = (points[0].double().to(device), points[1].double().to(device))

        intersect, length, distance, angle = check_intersect(points, self.verts, self.faces)

        distance = distance.item()
        self.assertAlmostEqual(distance, 1, places=5)

    def test_non_intersect_angle(self):
        points = (self.points[0], torch.tensor([2, 2, 2]).to(device))
        intersect, length, distance, angle = check_intersect(points, self.verts, self.faces)
        self.assertIsNone(angle)

    # visualize scenario -------------------------------------------------------

    def visualize(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        # Plot vertices
        ax.scatter(self.verts[:, 0].cpu(), self.verts[:, 1].cpu(), self.verts[:, 2].cpu(), color="b", label="Vertices")

        # Plot points
        ax.scatter(*self.points[0].cpu(), color="r", label="Point 1")
        ax.scatter(*self.points[1].cpu(), color="g", label="Point 2")

        # Plot faces
        for face in self.faces:
            polygon = Poly3DCollection([self.verts[face].cpu()], alpha=0.5)
            ax.add_collection3d(polygon)

        # Add labels
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        plt.legend()
        plt.show()


if __name__ == "__main__":
    unittest.main()

    # Visual check
    # test = TestCheckIntersect()
    # test.setUp()
    # test.visualize()
