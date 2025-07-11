import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


'''
A funcao apply_ROI() recebe a PointCloud2 advinda do LiDAR e passa um filtro de regiao de interesse
nela, diminuindo a quantidade de pontos conforme o input colocado.
'''
def apply_ROI(msg: np.ndarray,
                xmin: float, xmax: float,
                ymin: float, ymax: float,
                zmin: float, zmax: float)-> np.ndarray:
    
    points = msg

    mask = (
        (points[:,0]>=xmin) & (points[:,0]<=xmax)&
        (points[:,1]>=ymin) & (points[:,1]<=ymax)&
        (points[:,2]>=zmin) & (points[:,2]<=zmax)
    )

    ROI_points = points[mask]
    return ROI_points

    