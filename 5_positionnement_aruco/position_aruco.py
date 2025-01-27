import math

def calculate_aruco_positions_triangular(room_width, room_length, room_height, fov_horizontal, fov_vertical, min_distance, safety_coefficient):
    # Conversion des angles FOV en radians
    fov_horizontal_rad = math.radians(fov_horizontal)
    fov_vertical_rad = math.radians(fov_vertical)

    # Calcul des espacements horizontaux et verticaux
    horizontal_spacing = 2 * min_distance * math.tan(fov_horizontal_rad / 2) * safety_coefficient
    vertical_spacing = 2 * min_distance * math.tan(fov_vertical_rad / 2) * safety_coefficient

    # Liste pour stocker les positions des ArUco
    aruco_positions = []

    # Positionnement des ArUco sur chaque mur (disposition triangulaire)
    # Mur 1 (largeur x hauteur)
    x = 0  # Fixé sur le mur
    for z in range(0, math.ceil(room_height / vertical_spacing)):
        for y in range(0, math.ceil(room_width / horizontal_spacing)):
            # Décalage horizontal pour les rangées impaires (effet triangulaire)
            y_offset = (horizontal_spacing / 2) if z % 2 == 1 else 0
            aruco_positions.append((x, y * horizontal_spacing + y_offset, z * vertical_spacing))

    # Mur 2 (longueur x hauteur)
    y = room_width  # Fixé sur le mur
    for z in range(0, math.ceil(room_height / vertical_spacing)):
        for x in range(0, math.ceil(room_length / horizontal_spacing)):
            x_offset = (horizontal_spacing / 2) if z % 2 == 1 else 0
            aruco_positions.append((x * horizontal_spacing + x_offset, y, z * vertical_spacing))

    # Mur 3 (largeur x hauteur)
    x = room_length  # Fixé sur le mur
    for z in range(0, math.ceil(room_height / vertical_spacing)):
        for y in range(0, math.ceil(room_width / horizontal_spacing)):
            y_offset = (horizontal_spacing / 2) if z % 2 == 1 else 0
            aruco_positions.append((x, y * horizontal_spacing + y_offset, z * vertical_spacing))

    return aruco_positions


# Exemple d'utilisation
if __name__ == "__main__":
    # Dimensions de la pièce en mètres
    room_width = 9  # Largeur (X)
    room_length = 10.70  # Longueur (Y)
    room_height = 3  # Hauteur (Z)

    # Paramètres d'entrée
    fov_horizontal = 40  # En degrés
    fov_vertical = 30  # En degrés
    min_distance = 2  # Distance minimale entre le drone et les murs
    safety_coefficient = 0.8  # Coefficient de sécurité

    # Calcul des positions optimales
    aruco_positions = calculate_aruco_positions_triangular(
        room_width, room_length, room_height, fov_horizontal, fov_vertical, min_distance, safety_coefficient
    )

    # Affichage des positions
    for pos in aruco_positions:
        print(f"Position ArUco : x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}")
    print(f"Nombre total d'ArUco : {len(aruco_positions)}")