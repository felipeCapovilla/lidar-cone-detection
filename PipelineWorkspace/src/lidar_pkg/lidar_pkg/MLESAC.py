import numpy as np

# Supondo que constants.py exista e contenha as constantes necessárias
# Você precisará adicionar INLIER_SIGMA ao seu constants.py
from .MLESAC_constants import *
import std_msgs.msg
import sensor_msgs_py.point_cloud2 as pc2

def crop_rectangle(point_cloud: np.ndarray) -> np.ndarray:
    """
    Filter initially the point cloud by cropping just a rectangle.

    Args:
        point_cloud (np.ndarray): 3D coordinates in LiDAR frame.

    Returns:
        np.ndarray: filtered point cloud.
    """

    mask_x = (MINIMUM_X < point_cloud[:, 0]) & (point_cloud[:, 0] < MAXIMUM_X)
    mask_y = (MINIMUM_Y < point_cloud[:, 1]) & (point_cloud[:, 1] < MAXIMUM_Y)

    return point_cloud[mask_x & mask_y]


def compute_planes(random_points: np.ndarray) -> np.ndarray:
    """
    At every three points, compute the respective plane. The plane equation
    is ax + by + cz + d = 0 and the coefficients are a, b, c, and d.

    Args:
        random_points (np.ndarray): of downsample with 3D coordinates in rows.

    Returns:
        np.ndarray: normalized normals and respective d values of
            each plane with coefficients a, b, c, and d in rows.
    """

    reshaped = np.reshape(random_points, (NUM_PLANES, 3, 3))

    vectors = reshaped[:, 1:] - reshaped[:, :1]
    normals = np.cross(vectors[:, 0], vectors[:, 1])

    norms = np.sqrt(
        np.einsum("ij,ij->i", normals, normals)
    )

    mask = norms > 0   # Remove collinear points
    normals = normals[mask] / norms[mask, np.newaxis]

    d_values = -np.einsum("ij,ij->i", normals, reshaped[mask, 0])

    return np.concatenate((normals, d_values[:, np.newaxis]), axis=1)


def find_best_plane_mlesac(downsample: np.ndarray, planes: np.ndarray) -> np.ndarray:
    """
    Find the plane between the computed with the highest log-likelihood (MLESAC).

    Args:
        downsample (np.ndarray): just the first rows of the point cloud.
        planes (np.ndarray): normalized normals and respective d values
            of each plane with coefficients a, b, c, and d in rows.

    Returns:
        np.ndarray: normalized normal and respective d value of best plane.
    """
    best_log_likelihood = -np.inf
    best_plane = None

    # Constants for MLESAC likelihood calculation
    # INLIER_PROBABILITY: A estimativa da proporção de inliers. Um bom chute inicial é 0.5 a 0.8
    # OUTLIER_PROBABILITY: Probabilidade uniforme de um outlier. Pode ser 1.0 / (um valor grande)
    # ou 1.0 / volume_do_espaco_de_busca. Para simplificar, podemos definir um valor pequeno.
    # INLIER_SIGMA: Desvio padrão para a distribuição Gaussiana dos inliers. Crucial!
    #               Este valor deve ser sintonizado com base no ruído esperado dos seus dados.
    #               Pode ser aproximado por DIST2PLANE_THRESHOLD / 3 (regra do 3-sigma) ou similar.

    # Certifique-se de que INLIER_PROBABILITY, OUTLIER_PROBABILITY, e INLIER_SIGMA
    # estejam definidos no seu constants.py
    
    # Exemplo de como você pode definir essas constantes:
    # INLIER_PROBABILITY = 0.8 # Proporção esperada de inliers
    # OUTLIER_PROBABILITY = 1e-3 # Pequena probabilidade para outliers
    # INLIER_SIGMA = 0.05 # Exemplo: 5 cm de desvio padrão para inliers (ajuste conforme seu sensor/dados)

    # Verifica se as constantes estão definidas
    if not all(k in globals() for k in ['INLIER_PROBABILITY', 'OUTLIER_PROBABILITY', 'INLIER_SIGMA']):
        raise ValueError("INLIER_PROBABILITY, OUTLIER_PROBABILITY, and INLIER_SIGMA must be defined in constants.py")


    for i, plane in enumerate(planes):
        # Distâncias absolutas dos pontos ao plano
        distances = np.abs(np.matmul(plane[:3], downsample.T) + plane[3])

        # Calcular a probabilidade para cada ponto (usando a PDF Gaussiana para inliers)
        # np.fmax para evitar log(0) em caso de probabilidade muito pequena
        
        # Probabilidade de ser inlier (assumindo distribuição Gaussiana centrada em 0 distância)
        prob_inlier = (1 / (np.sqrt(2 * np.pi) * INLIER_SIGMA)) * np.exp(-distances**2 / (2 * INLIER_SIGMA**2))
        
        # Probabilidade combinada (inlier ou outlier)
        # Multiplicamos por INLIER_PROBABILITY para inliers e OUTLIER_PROBABILITY para outliers
        # A constante OUTLIER_PROBABILITY representa 1/V da formula original
        combined_prob = (INLIER_PROBABILITY * prob_inlier) + ((1 - INLIER_PROBABILITY) * OUTLIER_PROBABILITY)
        
        # Evitar log de zero, garantindo um valor mínimo
        combined_prob = np.fmax(combined_prob, np.finfo(float).eps)

        # Calcular a verossimilhança logarítmica para o plano atual
        current_log_likelihood = np.sum(np.log(combined_prob))

        # Atualizar o melhor plano se a verossimilhança for maior
        if current_log_likelihood > best_log_likelihood:
            best_log_likelihood = current_log_likelihood
            best_plane = plane

    if best_plane is None:
        # Se nenhum plano foi encontrado (pode acontecer se NUM_PLANES for muito pequeno ou dados ruins)
        # Você pode retornar um plano "vazio" ou levantar um erro, dependendo do seu caso de uso.
        # Por enquanto, retornaremos um array de zeros, mas você pode querer tratar isso melhor.
        print("Aviso: Nenhum plano adequado foi encontrado com MLESAC. Retornando plano zero.")
        return np.zeros(4)
        
    return best_plane


def remove_floor(point_cloud: np.ndarray) -> np.ndarray:
    """
    Find the best plane and remove the inliers which represent the floor.

    Args:
        point_cloud (np.ndarray): 3D coordinates in LiDAR frame.

    Returns:
        np.ndarray: filtered (no floor) point cloud.
    """

    point_cloud = crop_rectangle(point_cloud)
    downsample = point_cloud[:DOWNSAMPLE_SIZE]

    # Garante que downsample não seja vazio para evitar erros no np.random.randint
    if downsample.shape[0] == 0:
        return np.array([]) # Retorna uma nuvem de pontos vazia se não houver pontos após o crop.

    # Ajuste para garantir que DOWNSAMPLE_SIZE seja <= ao número de pontos disponíveis
    effective_downsample_size = min(DOWNSAMPLE_SIZE, downsample.shape[0])
    
    # O número de pontos aleatórios deve ser menor ou igual ao tamanho do downsample
    num_random_points_needed = 3 * NUM_PLANES
    if effective_downsample_size < num_random_points_needed:
        # Não há pontos suficientes para gerar todos os planos necessários.
        # Você pode ajustar NUM_PLANES ou retornar uma nuvem vazia, ou tentar com menos planos.
        print(f"Aviso: Não há pontos suficientes ({effective_downsample_size}) para gerar {num_random_points_needed} pontos aleatórios. Reduzindo NUM_PLANES.")
        # Ajusta NUM_PLANES para o máximo possível
        # Garante que NUM_PLANES seja pelo menos 1 para evitar divisão por zero se effective_downsample_size < 3
        actual_num_planes = max(1, effective_downsample_size // 3) 
        random_indices = np.random.randint(effective_downsample_size, size=3 * actual_num_planes)
    else:
        random_indices = np.random.randint(effective_downsample_size, size=num_random_points_needed)
    
    random_points = downsample[random_indices]

    planes = compute_planes(random_points)
    
    if planes.shape[0] == 0:
        print("Aviso: Nenhum plano válido foi computado. Retornando nuvem de pontos original sem filtro de chão.")
        return point_cloud # Ou retornar um array vazio, dependendo da sua estratégia de erro.

    # Agora usamos a nova função MLESAC
    best_plane = find_best_plane_mlesac(downsample, planes)

    # A remoção final do chão ainda usa o DIST2PLANE_THRESHOLD,
    # que é o comportamento esperado para "remover" os pontos do chão.
    distances = np.abs(
        np.matmul(best_plane[:3], point_cloud.T) + best_plane[3]
    )

    return point_cloud[distances > DIST2PLANE_THRESHOLD]