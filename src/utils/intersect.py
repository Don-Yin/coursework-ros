from rich import print

from src.variables import entries_targets_combs, images_meshes, device
import torch
from torch import tensor
from tqdm import tqdm
import numpy as np


def check_intersect(points: tuple[tensor, tensor], verts: tensor, faces: tensor):
    """
    Args:
        points (Tuple[torch.Tensor, torch.Tensor]): A tuple of two tensors representing the origin and destination of the ray, respectively. Each tensor has three elements representing the x, y, and z coordinates of the point. Each point is represented by a tensor of shape (3,) as a 1D vector (example: torch.tensor([14.9295, 31.4212, 7.8232], dtype=torch.float64)).
        verts (torch.Tensor): The vertices of the triangle surface. It is a tensor of shape (N, 3), where N is the number of vertices. Each vertex is represented by a tensor of shape (3,) as a 1D vector (example: torch.tensor([[x1, y1, z1], [x2, y2, z2], ...], dtype=torch.float64)).
        faces (torch.Tensor): The faces of the triangle surface as the indices of the vertices. It is a tensor of shape (M, 3), where M is the number of faces. Each face is represented by a tensor of shape (3,) as a 1D vector containing the vertex indices (example: torch.tensor([[v1, v2, v3], [v4, v5, v6], ...], dtype=torch.int64)).

    Returns:
        intersect (bool): A boolean indicating whether the ray intersects with the triangle surface. Returns True if the ray intersects.

        length (float): The length between the origin of the ray and the *first* intersection point. If the ray does not intersect with the triangle surface, returns the length of the ray.

        distance (float): The shortest distance between any point on the ray and the triangle surface if the ray does not intersect. If the ray intersects with the triangle surface, returns 0. The distance is the minimal distance between the ray and any point on the any vertices.

        angle (float): The minimum angle between the ray and the normal vector of the triangle surface. If the ray does not intersect with the triangle surface returns None. If the ray intersects, returns the angle between the ray and the normal vector of the triangle surface at the intersection point.

    Example usage:
        # Define the ray origin and direction
        ray_origin = torch.tensor([14.9295, 31.4212, 7.8232], dtype=torch.float64)
        ray_destination = torch.tensor([23.5000, 29.5000, 18.5000], dtype=torch.float64)

        # Define the vertices of the triangle surface
        vertices = torch.tensor([[10.0, 20.0, 5.0], [20.0, 20.0, 5.0], [15.0, 30.0, 5.0]], dtype=torch.float64)

        # Define the faces of the triangle surface
        faces = torch.tensor([[0, 1, 2]], dtype=torch.int64)

        # Check intersection
        intersect, length, distance, angle = check_intersect((ray_origin, ray_destination), vertices, faces)

    Procedure of coding:
        1. Identify the faces that intersect with the given ray. If there is at least one intersecting face:
            - Set `intersect` to True.
            - Determine the first face that the ray intersects with and get its index.
            - Calculate the length of the ray between the origin and the intersection point on the first face, and assign it to `length`.
            - Set `distance` to 0 since the ray intersects with the triangle surface.
            - Calculate the angle between the ray and the normal vector of the first face at the intersection point, and assign it to `angle`.

        2. If no face intersects with the ray:
            - Set `intersect` to False.
            - Calculate the length of the ray, which is the distance between the origin and destination points, and assign it to `length`.
            - Create a line between the origin and destination points (i.e., treating the ray as a line).
            - make a generic function that takes a line and all the vertices and returns the distance between the line and the closest vertex.
            - Calculate the distance between the line and the closest point on the mesh vertices, and assign it to `distance`.
            - Set `angle` to None since the ray does not intersect with the triangle surface.
    """
    ray_origin, ray_destination = points

    ray_direction = ray_destination - ray_origin
    ray_direction = ray_direction.unsqueeze(0)

    # Edge calculation
    vert1 = verts[faces[:, 0].long()]
    vert2 = verts[faces[:, 1].long()]
    vert3 = verts[faces[:, 2].long()]

    edge1 = vert2 - vert1
    edge2 = vert3 - vert1

    # Normal calculation
    normal = torch.cross(edge1, edge2, dim=1)
    normal = normal / (torch.norm(normal, dim=1, keepdim=True) + 1e-8)

    # Find intersection
    diff = ray_origin - vert1
    pvec = torch.cross(ray_direction, edge2, dim=1)
    qvec = torch.cross(diff, edge1, dim=1)

    det = torch.einsum("ij,ij->i", edge1, pvec)
    inv_det = 1.0 / det

    u = torch.einsum("ij,ij->i", diff, pvec) * inv_det
    v = torch.einsum("ij,ij->i", ray_direction, qvec) * inv_det
    t = torch.einsum("ij,ij->i", edge2, qvec) * inv_det

    # Intersection condition
    eps = 1e-8
    intersection_condition: tensor[bool] = (
        (det.abs() > eps) & (u >= 0.0) & (v >= 0.0) & (u + v <= 1.0) & (t >= 0.0) & (t <= 1.0)
    )

    if sum(intersection_condition) > 0:
        intersect = True
        distance = 0.0

        # find the first intersecting face
        intersect_indices = torch.nonzero(intersection_condition, as_tuple=True)[0]
        t_intersect = t[intersect_indices]
        first_intersect_face_index: int = intersect_indices[torch.argmin(t_intersect)].item()

        # Calculate the length of the ray between the origin and the intersection point on the first face, and assign it to `length`.
        intersect_point = ray_origin + t[first_intersect_face_index] * ray_direction.squeeze(0)
        length = torch.norm(intersect_point - ray_origin)

        # Calculate the angle between the ray and the normal vector of the first face at the intersection point, and assign it to `angle`.
        normal_intersect = normal[first_intersect_face_index]
        ray_direction_norm = ray_direction / torch.norm(ray_direction)
        normal_intersect_norm = normal_intersect / torch.norm(normal_intersect)
        dot_product = torch.einsum("i,i->", ray_direction_norm.squeeze(0), normal_intersect_norm)
        angle = torch.acos(dot_product) * (180.0 / np.pi)

    else:
        intersect = False
        length = torch.norm(ray_destination - ray_origin)
        angle = None

        # Calculate the distance between each vertex and the line
        vert_line_distances = torch.norm(torch.cross(verts - ray_origin, verts - ray_destination), dim=1) / length
        distance = torch.min(vert_line_distances)

    return intersect, length, distance, angle


def check_intersect_batch(ray_origins, ray_destinations, verts, faces):
    ray_directions = ray_destinations - ray_origins  # shape: (batch_size, 3) instead of (3,)

    batch_size = ray_origins.size(0)

    # Edge calculation
    vert1 = verts[faces[:, 0].long()]
    vert2 = verts[faces[:, 1].long()]
    vert3 = verts[faces[:, 2].long()]

    edge1 = vert2 - vert1
    edge2 = vert3 - vert1

    edge1 = edge1.unsqueeze(0)
    edge2 = edge2.unsqueeze(0)

    edge1 = edge1.expand(batch_size, -1, -1)
    edge2 = edge2.expand(batch_size, -1, -1)

    # after expanding edge1 and edge2:
    normal = torch.cross(edge1, edge2, dim=2)
    normal = normal / (torch.norm(normal, dim=2, keepdim=True) + 1e-8)  # shape: (batch_size, num_faces, 3)

    diff = ray_origins.unsqueeze(1) - vert1  # shape: (batch_size, num_faces, 3)
    pvec = torch.cross(ray_directions.unsqueeze(1), edge2, dim=2)  # shape: (batch_size, num_faces, 3)
    qvec = torch.cross(diff, edge1, dim=2)  # shape: (batch_size, num_faces, 3)

    det = torch.einsum("bij,bij->bi", edge1, pvec)
    inv_det = 1.0 / det

    u = torch.einsum("bij,bij->bi", diff, pvec) * inv_det
    v = torch.einsum("bij,bij->bi", ray_directions.unsqueeze(1), qvec) * inv_det
    t = torch.einsum("bij,bij->bi", edge2, qvec) * inv_det

    # Intersection condition for batch processing
    eps = 1e-8
    intersection_condition: torch.Tensor = (
        (det.abs() > eps) & (u >= 0.0) & (v >= 0.0) & (u + v <= 1.0) & (t >= 0.0) & (t <= 1.0)
    )

    # here comes the important part ---------------------------------------------
    intersection_count = torch.sum(intersection_condition, dim=1)

    # for each ray, check if it intersects with any faces
    intersect = intersection_count > 0

    # initialize tensors to hold results
    distance = torch.zeros(ray_origins.shape[0]).to(device).double()
    length = torch.zeros(ray_origins.shape[0]).to(device).double()
    angle = torch.zeros(ray_origins.shape[0]).to(device).double()

    if intersect.sum() > 0:
        t_intersect = torch.full_like(t, float("inf"))
        t_intersect[intersection_condition] = t[intersection_condition]
        first_intersect_face_indices = torch.argmin(t_intersect, dim=1)
        t_values = t[torch.arange(t.shape[0]), first_intersect_face_indices].unsqueeze(1)

        intersect_point = ray_origins + t_values * ray_directions
        intersect_point_intersects = intersect_point[intersect]
        ray_origins_intersects = ray_origins[intersect]
        length[intersect] = torch.norm(intersect_point_intersects - ray_origins_intersects, dim=1)

        normal_intersect = normal[torch.arange(normal.shape[0]), first_intersect_face_indices]
        ray_direction_norm = ray_directions / torch.norm(ray_directions, dim=1, keepdim=True)
        normal_intersect_norm = normal_intersect / torch.norm(normal_intersect, dim=1, keepdim=True)
        dot_product = torch.einsum("bi,bi->b", ray_direction_norm, normal_intersect_norm)

        all_angles = torch.acos(dot_product) * (180.0 / np.pi)
        angle[intersect] = all_angles[intersect]

    # for batches where no intersection was found, calculate the distance to the closest line
    no_intersect = ~intersect
    if no_intersect.sum() > 0:
        length[no_intersect] = torch.norm(ray_destinations[no_intersect] - ray_origins[no_intersect], dim=1)
        angle[no_intersect] = np.nan

        # # Calculate the distance between each vertex and the line / without excluding the vertices outside the line segment
        # vert_line_distances = torch.norm(
        #     torch.cross(verts.unsqueeze(0) - ray_origins.unsqueeze(1), verts.unsqueeze(0) - ray_destinations.unsqueeze(1)),
        #     dim=2,
        # ) / length.unsqueeze(1)

        # all_distance = torch.min(vert_line_distances, dim=1)[0]
        # distance[no_intersect] = all_distance[no_intersect]

        # Calculate the vector from the ray origin to each vertex
        vert_origin_vectors = verts.unsqueeze(0) - ray_origins.unsqueeze(1)

        # Calculate the vector from the ray origin to the ray destination
        ray_vector = ray_destinations - ray_origins

        # Compute the dot product between each vector and the ray direction
        dot_products = torch.einsum("bij,bj->bi", vert_origin_vectors, ray_vector)

        # Compute square lengths for both the ray and the vector from ray origin to each vertex
        ray_sq_length = torch.sum(ray_vector**2, dim=1)

        # Vertices are within the ray segment if dot product is positive and less than or equal to the square of the length of the ray
        within_ray_segment = (dot_products >= 0) & (dot_products <= ray_sq_length.unsqueeze(1))

        # Calculate the distance between each vertex and the line, but only for vertices within the ray segment
        vert_line_distances = torch.where(
            within_ray_segment,
            torch.norm(
                torch.cross(vert_origin_vectors, verts.unsqueeze(0) - ray_destinations.unsqueeze(1)),
                dim=2,
            )
            / length.unsqueeze(1),
            float("inf"),
        )

        # The minimum distance for each ray (ignoring vertices outside the ray segment)
        all_distance = torch.min(vert_line_distances, dim=1)[0]
        distance[no_intersect] = all_distance[no_intersect]

    return intersect, length, distance, angle