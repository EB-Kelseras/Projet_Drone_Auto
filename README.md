# Projet 5A:
(documentation du répertoire)

## Example 
* Calibrage_camera : Contient les fichiers et programmes permettant de calibrer une caméra
* Tello_Code : Contient des programmes du simulateur
* 0_Ressources : Contient des documents utiles au projet (articles/codes/...)

## Plan Projet Drone ARUCO TCI

## I.	Acquérir les données
1.	<span style="color:green">Lecture de la caméra</span> 
2.	Récupération des données inertielles de l’IMU
## II.	Prétraitement des données
1.	<span style="color:green">Calibration Caméra</span>
2.	Synchronisation données caméra/IMU
## III.	Extraction de caractéristiques
1.	<span style="color:green">Détection code Aruco</span>
2.	<span style="color:green">Extraction ID, Position dans l’image (coordonnées des coins), orientations</span>
3.	Détermination de la position 3D de la caméra (Equation de projection)
## IV.	Optimisation
1.	Facteur Graphe
## V.	Création de la carte de trajectoire
1.	Mise à jour des données collectées en temps réel
