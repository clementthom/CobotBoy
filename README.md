# Organisation du répertoire de projet MECA22-3 (CobotCNJS)

Ce document retrace l'arborescence du répertoire de travail du groupe MECA22-3, travaillant sur le Cobot CNJS. Il fournit aussi quelques informations sur son contenu. Cette démarche a pour but de faciliter la navigation des personnes de l'équipe ou en lecture, et de ne pas égarer de documents.

## Code

Ce dossier rassemble l'ensemble du code produit par l'équipe, ainsi que sa documentation :

- **Archives** : contient du code archivé (non utilisé en l'état dans la version finale) ou des documents de travail ayant permis de créer la documentation finale
- **CODE_MEC22_3** : contient le code Arduino sous forme modulaire, facilitant la collaboration, la maintenabilité et la compréhension de ce dernier
- **Documentation** : contient l'intégralité de la documentation liée de près ou de loin au code et à ses choix technologiques. Sont englobés dans la documentation :
  - Le code et les ressources complètes liées au **dimensionnement du PID du système préhenseur**
  - Les **calculs mécaniques**, justifiant la conversion des coordonnées de l'objet en angles de commande pour les servomoteurs
  - Un compte rendu du **dimensionnement du PID**, résumant le travail effectué dans le dossier antérieur
  - Un **grafcet du cycle 2**
  - Un document .svg rassemblant tous les **schémas de représentation mécaniques du systèmes** ayant permis d'expliquer la conversion coordonnées/angles de commande
- **Terminal** : contient le code, les icônes et les exécutables de l'interface du terminal, accessible sur ordinateur, et permettant au Cobot CNJS d'entrer en mode maintenance
- **Lien GitHub** : fichier contenant les 2 liens liés à des répertoires GitHub liés au projet (ils sont à ce stade du projet obsolètes)

## Electronique

Ce dossier contient tout le travail effectué par l'équipe sur l'électronique, en particulier sur l'Interface Humain Machine (hors conception de l'identité graphique/UX) : 

- **Archives** : contient d'anciennes version de la carte de l'IHM 
- **boardDesign** : contient tous les fichiers liés à la conception de la carte
- **Documentation** : contient la documentation liée à cette conception, comme la **BOM**, le **bilan entrées-sorties et de puissance**, les **schémas du circuit électronique et de la carte**, ainsi que des captures d'écran représentant la carte en 3D
- un fichier compressé du **Gerber**, ainsi que des fichiers d'empreinte de l'encodeur rotatif

## Raccourci vers l'espace commun

Ce raccourci permet à l'équipe d'accéder en un clic aux ressources fournies par le client et par les experts.

## Gestion de projet

Ce dossier rassemble tous les outils de gestion de projet, incluant des **comptes rendus d'entrevues avec le client**, le **planning projet**, la banque horaire et autres fichiers similaires.

## Identité et communication

Ce dossier contient tout le travail en relation avec l'image de marque du projet, comme les fichiers de conception graphique de l'IHM, les versions successives des différents logos liés à CNJS, ainsi que la vidéo de présentation du système. On y retrouve aussi des ressources liées à la création de cette image, comme des rushs vidéos ou des fichiers d'édition vidéo.

## Manuels clients

Ce dossier contient les livrables à fournir au client qui ne peuvent être associés à un aspect du projet uniquement : 

- le **Manuel d'utilisation**
- le **Manuel de maintenance**

## Mécanique

Ce dossier contient tout le travail de conception et de calculs mécaniques, ainsi que les livrables et rendus associés : 

- **L'étude mécanique du 24/04/2026** : inclut le rapport d'étude dynamique et les plans de définitions finaux des préhenseurs
- **Mise en plan préhenseurs** : plans des préhenseurs antérieurs au 24 avril
- **Pièces 3D** : contient l'ensemble des pièces 3D produites par l'équipe (principalement en .step)
- **Le schéma cinématique du système**
