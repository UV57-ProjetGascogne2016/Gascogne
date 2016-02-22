###### README du Dossier Control Commande ######

## 6 fichiers ##

- main.py
- README.txt
- regulateur.py
- simulateur.py
- simulationPotentielsGascogne.py
- simulationPotentielsTerrainFoot.py

## Developpeurs ##

Thomas Boulier
Sylvain Hunault

## Descriptifs des fichiers : ##

# main.py #

*Fichier coeur qui assemble les fichiers regulateur.py et simulateur.py pour mettre en évidence l'architecture voulue avec un régulateur bien séparé du simulateur.

# README.txt #

*LE README.

# regulateur.py #

*Ce fichier contient le régulateur par champs de potentiels.
*Il contient une classe "Robot" qui sert à contenir les variables d'état du robot (position, vitesse, cap), à la quelle est associé les fonctions de régulation. Le régulateur est fait pour recalculer le cap des robots (fonction calculCap()) par différence finie.
*La fonction de plus haut niveau à appeler est la fonction regulate(t) pour un robot, il faut en entrée le temps écoulé, elle renvoit les commandes pour le robot (l'une pour avancer tout droit, l'autre pour tourner).
Cette fonction détermine dans un premier temps le point cible pour le robot par la fonction traj(t,delta) puis calcule la commande associé à cette cible avec la fonction controlCommande().
*La fonction traj(t,delta) sert à gérer l'ellipse que doivent suivre les robots. La structure de gestion de l'ellipse est celle développée par Sylvain Hunault et Thomas Boulier, moins performante que celle developpée par Alice Danckaers.

# simulateur.py #

*Ce fichier implémente une simulation par Euler d'un robot.
*Il prend en entrée un robot et une commande moteur (pwm) puis retourne le nouvel état du robot.

# simulationPotentielsGascogne.py #

Fichier source originale : simulationControl.py réalisé par Alice Danckaers et Elouan Autret

*Réalise la simulation sur le Golfe de Gascogne avec la régulation par potentiels, le régulateur dans regulateur.py a été adapté puis inclus dans ce fichier.
*L'évolution de l'ellipse à suivre par les robots est plus élaborée. Ce fichier a donc été repris pour réaliser une meilleure simulation avec la régulation par potentiels.

# simulationPotentielsTerrainFoot.py #

Fichier source originale : simulationControl.py réalisé par Alice Danckaers et Elouan Autret

*Réalise la simulation sur le terrain de l'ENSTA Bretagne avec la régulation par potentiels, le régulateur dans regulateur.py a été adapté puis inclus dans ce fichier.
*L'évolution de l'ellipse à suivre par les robots est plus élaborée. Ce fichier a donc été repris pour réaliser une meilleure simulation avec la régulation par potentiels.
