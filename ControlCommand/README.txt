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

*Fichier coeur qui assemble les fichiers regulateur.py et simulateur.py pour mettre en �vidence l'architecture voulue avec un r�gulateur bien s�par� du simulateur.

# README.txt #

*LE README.

# regulateur.py #

*Ce fichier contient le r�gulateur par champs de potentiels.
*Il contient une classe "Robot" qui sert � contenir les variables d'�tat du robot (position, vitesse, cap), � la quelle est associ� les fonctions de r�gulation. Le r�gulateur est fait pour recalculer le cap des robots (fonction calculCap()) par diff�rence finie.
*La fonction de plus haut niveau � appeler est la fonction regulate(t) pour un robot, il faut en entr�e le temps �coul�, elle renvoit les commandes pour le robot (l'une pour avancer tout droit, l'autre pour tourner).
Cette fonction d�termine dans un premier temps le point cible pour le robot par la fonction traj(t,delta) puis calcule la commande associ� � cette cible avec la fonction controlCommande().
*La fonction traj(t,delta) sert � g�rer l'ellipse que doivent suivre les robots. La structure de gestion de l'ellipse est celle d�velopp�e par Sylvain Hunault et Thomas Boulier, moins performante que celle developp�e par Alice Danckaers.

# simulateur.py #

*Ce fichier impl�mente une simulation par Euler d'un robot.
*Il prend en entr�e un robot et une commande moteur (pwm) puis retourne le nouvel �tat du robot.

# simulationPotentielsGascogne.py #

Fichier source originale : simulationControl.py r�alis� par Alice Danckaers et Elouan Autret

*R�alise la simulation sur le Golfe de Gascogne avec la r�gulation par potentiels, le r�gulateur dans regulateur.py a �t� adapt� puis inclus dans ce fichier.
*L'�volution de l'ellipse � suivre par les robots est plus �labor�e. Ce fichier a donc �t� repris pour r�aliser une meilleure simulation avec la r�gulation par potentiels.

# simulationPotentielsTerrainFoot.py #

Fichier source originale : simulationControl.py r�alis� par Alice Danckaers et Elouan Autret

*R�alise la simulation sur le terrain de l'ENSTA Bretagne avec la r�gulation par potentiels, le r�gulateur dans regulateur.py a �t� adapt� puis inclus dans ce fichier.
*L'�volution de l'ellipse � suivre par les robots est plus �labor�e. Ce fichier a donc �t� repris pour r�aliser une meilleure simulation avec la r�gulation par potentiels.
