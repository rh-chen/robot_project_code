=============
Path planning
=============

.. |path_planning| image:: temp_static/path_planning/path_planning.png
   :scale: 50 %

.. |contours| image:: temp_static/path_planning/contours.png
   :scale: 50 %

.. |follow_poses| image:: temp_static/path_planning/follow_poses.png
   :scale: 50 %

|path_planning| |contours| |follow_poses|

.. ATTENTION::
   L’action de générer une trajectoire supprime toutes les modifications effectuées sur la trajectoire courante !

Génération d'une trajectoire
============================
* **Generation algorithm:** Permet de choisir l’algorithme à utiliser.
* **YAML or mesh file:** Permet de sélectionner le fichier modèle pour générer les trajectoires:

  * Les fichiers maillages acceptés sont représentés par les extensions ``.obj`` ``.stl`` et ``.ply``.
  * Un fichier YAML est représenté par l’extension ``.yaml`` (ou ``yml``). C’est un fichier texte. Voici un exemple de fichier YAML:

  .. code-block:: YAML

    ---
    – layer:
      – polygon:
        – [0.075, -0.025, 0]
        – [0.025, -0.025, 0]
        – [0, 0, 0]
        – [0.025, 0.075, 0]
        – [0.125, 0.025, 0]
      – polygon:
        – [0.025, 0, 0]
        – [0.025, 0.025, 0]
        – [0.05, 0.05, 0]
        – [0.075, 0.025, 0]
        – [0.075, 0, 0]
    – layer:
      – polygon:
        – [0.15, -0.025, 0]
        – [0.15, 0.025, 0]
        – [0.175, 0.025, 0]
        – [0.2, 0, 0]
        – [0.18, -0.0125, 0]
        – [0.2, -0.025, 0]

* Les 3 premières lignes sont obligatoires pour définir une couche et un polygone.
* L'indentation est obligatoire et doit être respectée.
* Il est possible d’ajouter autant de polygones que souhaité.
* Suivant l’algorithme il est possible ou non d’ajouter plusieurs couches.

Les fichiers YAML peuvent être plus complets: on peut spécifier l’orientation aux points, sous la forme ``[position, orientation]`` avec ``position=[X, Y, Z]`` et ``orientation=[qX, qY, qZ, qW]`` (quaternions normalisés). Voici un exemple:

.. code-block:: YAML

  ---
  – layer:
    – polygon:
  #   – [X, Y, Z, qX, qY, qZ, qW]
      – [0, 0, 0, 0, 0, 0, 1]
      – [0.05, 0, 0, 0, 0, 0.707, 0.707]
      – [0.05, 0.05, 0, 0, 0, 1, 0]
      – [0, 0.05, 0, 0, 0, 0.707, -0.707]

Descriptions des algorithmes
============================
* **DonghongDing**: Algorithme qui maximise le remplissage de la géométrie et sans arrêt dans la trajectoire.
* **Contours**: Génère une trajectoire sur le contour de la géométrie (pas de remplissage) sans arrêt dans la trajectoire.
* **Follow poses**: Permet de suivre une liste de poses dans un fichier YAML.

DonghongDing et Contours
========================
* **Number of layers:** Le nombre de couches à générer, les couches suivantes sont dupliquées à partir de la 1ère couche (disponible uniquement avec un fichier YAML).
* **Height between layers:** La distance entre chaque couche.
* **Deposited material width:** La largeur de dépose du matériau, tenir compte du recouvrement si nécessaire.
* **Slicing direction:** Le vecteur direction pour le découpage du maillage (disponible uniquement avec un fichier maillage).

Il n’est pas possible de définir plusieurs couches dans un fichier YAML avec ces deux algorithmes.

DonghongDing
------------
**Contours filtering tolerance:** Insérer la tolérance à respecter entre le contour réel et celui utilisé. Le contour est d’autant plus simplifié que la tolérance est haute. Si la valeur demandée est trop faible (dépend de la forme demandée et de la capacité de l’algorithme à remplir la forme) l’erreur ``Failed to generate trajectory in one of the convex polygons`` va apparaître, et si la valeur demandée est trop grande l’erreur ``Failed to merge colinear edges`` va apparaître. Pour arriver à un résultat fonctionnel il faut tester plusieurs valeurs en s’aidant de ces 2 indications. Par exemple un cercle avec le fichier YAML suivant:

.. code-block:: YAML

  ---
  – layer:
    – polygon:
      – [0.05, 0, 0]
      – [0.0489074, 0.0103956, 0]
      – [0.0456773, 0.0203368, 0]
      – [0.0404509, 0.0293893, 0]
      – [0.0334565, 0.0371572, 0]
      – [0.025, 0.0433013, 0]
      – [0.0154508, 0.0475528, 0]
      – [0.00522642, 0.0497261, 0]
      – [-0.00522642, 0.0497261, 0]
      – [-0.0154508, 0.0475528, 0]
      – [-0.025, 0.0433013, 0]
      – [-0.0334565, 0.0371572, 0]
      – [-0.0404509, 0.0293893, 0]
      – [-0.0456773, 0.0203368, 0]
      – [-0.0489074, 0.0103956, 0]
      – [-0.05, 2.83277e-17, 0]
      – [-0.0489074, -0.0103956, 0]
      – [-0.0456773, -0.0203368, 0]
      – [-0.0404509, -0.0293893, 0]
      – [-0.0334565, -0.0371572, 0]
      – [-0.025, -0.0433013, 0]
      – [-0.0154508, -0.0475528, 0]
      – [-0.00522642, -0.0497261, 0]
      – [0.00522642, -0.0497261, 0]
      – [0.0154508, -0.0475528, 0]
      – [0.025, -0.0433013, 0]
      – [0.0334565, -0.0371572, 0]
      – [0.0404509, -0.0293893, 0]
      – [0.0456773, -0.0203368, 0]
      – [0.0489074, -0.0103956, 0]

Avec l’algorithme **Follow poses** comme référence (aucun filtrage):

.. image:: temp_static/path_planning/circle_follow.png
   :align: center
   :scale: 30 %

Avec l’algorithme DonghongDing avec respectivement une tolérance de 1, 1.1, 9 et 9.1 mm:

.. |circle_fill_1| image:: temp_static/path_planning/circle_fill_1.png

.. |circle_fill_9_1| image:: temp_static/path_planning/circle_fill_9_1.png

.. |circle_fill_9| image:: temp_static/path_planning/circle_fill_9.png

.. |circle_fill_1_1| image:: temp_static/path_planning/circle_fill_1_1.png

+-------------------+-------------------+
| 1 mm:             | 9.1 mm:           |
+-------------------+-------------------+
| |circle_fill_1|   | |circle_fill_9_1| |
+-------------------+-------------------+
| 1.1 mm            | 9 mm              |
+-------------------+-------------------+
| |circle_fill_1_1| | |circle_fill_9|   |
+-------------------+-------------------+

Lorsque le filtrage est très élevé la géométrie est non préservée: sur le dernier exemple le cercle a été simplifié en un triangle.

Follow poses
============
Follow poses n’est pas réellement un algorithme, c’est un programme qui se contente de suivre les poses qu’on lui fournit.
Il n'est possible d'utiliser que des fichiers YAML (``.yaml``) en entrée.

* **Duplicate layers:** Permet de dupliquer les poses du fichier YAML suivant l’axe Z.

  * **Number of layers:** Le nombre de couches à générer.
  * **Height between layers:** La distance entre chaque couche.
  * **Invert one of two layers:** Permet d’inverser le sens de lecture d’une couche sur 2, afin que l’entrée de la couche suivante se retrouve au niveau de la sortie de la couche précédente.

Il est possible de définir plusieurs couches dans un fichier YAML avec l'algorithme ``Follow poses``. Dans ce cas il n'est pas possible de dupliquer les couches.

Exemple de trajectoire avec orientation des poses spécifiées dans le fichier YAML:

.. image:: temp_static/path_planning/square_quaternion.png
   :align: center
   :scale: 30 %
