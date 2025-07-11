import numpy as np
from .constants import *
from sklearn.cluster import DBSCAN


def is_cone_like(cluster_points: np.ndarray) -> bool:
 
    if cluster_points.shape[0] < 5: 
        return False

    xs = cluster_points[:, 0]
    ys = cluster_points[:, 1]
    zs = cluster_points[:, 2]
    
    height = np.max(zs) - np.min(zs)
    xy_extent = np.max(np.sqrt((xs - np.mean(xs))**2 + (ys - np.mean(ys))**2)) * 2

    if xy_extent == 0:
        return False

    ratio = height / xy_extent
    std_z = np.std(zs)

    return (
        0.1 <= height <= 0.4 and          # antes era 0.2–0.5
        0.08 <= xy_extent <= 0.3 and      # antes era 0.15–0.4
        0.8 <= ratio <= 3.0 and           # ampliado para tolerância maior
        std_z < 0.12                      # um pouco mais permissivo
    )


def is_symmetric_cluster(cluster_points: np.ndarray, threshold: float = 0.2) -> bool:

    centroid = np.mean(cluster_points[:, :2], axis=0)
    distances = np.linalg.norm(cluster_points[:, :2] - centroid, axis=1)

    std_dist = np.std(distances)
    return std_dist < threshold


def DBSCAN_cluster(filtered_pCloud: np.ndarray) -> np.ndarray:

    cluster = DBSCAN(eps=MAX_DISTANCE, min_samples=MIN_SAMPLES)
    labels = cluster.fit(filtered_pCloud).labels_

    centroides = []
    unique_labels = set(labels)

    for label in unique_labels:
        if label == -1:
            continue
        
        cluster_points = filtered_pCloud[labels == label]
        
        if is_cone_like(cluster_points) and is_symmetric_cluster(cluster_points):
            centroide = np.mean(cluster_points, axis=0)
            centroides.append(centroide)

    return np.asarray(centroides)
