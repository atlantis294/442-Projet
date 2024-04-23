# Projet de 442 : Carte embarquée
## Objectif : 
Mettre en place un système permettant de faire bouger une carte sur l'écran d'un STM32.

On utilise une carte IGN au format BMP RGB565 de taille 1707x700.


<img 
    style="display: block; 
           margin-left: auto;
           margin-right: auto;
           width: 60%;"
    src="mapENS.bmp" 
    alt="Carte utilisé">
<!-- </img> -->

On affichera sur un écran LCD de taille 480x272.
Pour cela on aura un travail de redimensionnement. 


[comment]: <> (Explique les différentes fonctions)

## Taches :
* Déplacement : calcule le déplacement avec l'appui
* Affichage : s'occupe d'afficher l'image s'il y a un mouvement qui est demandé.

## Queues :
* déplacement : transmet la structure mouvement (dx,dy) de la mesure du déplacement à l'affichage qui se charge de remplir l'image à afficher. 

## Fonctions :
* Remplir image : rempli les pixels en reprenant l'image avec les bonnes coordonnées
* Fabriquer entête : fabrique l'entête du fichier bmp




