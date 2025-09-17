# Template Driver Zephyr

Structure de base pour la création d'un driver personnalisé ZephyrRTOS

## dts/bindings

Modifier __vendor-prefixes.txt__ avec :
```txt
fabricant[TAB]device
```
**Utiliser le bloc note**, la tabulation de VSCode ne fonctionne pas

Ensuite modifier le fichier yaml pour définir les propriétés/bus obligatoires dans le devicetree

## include

Modifier/supprimer la regmap selon les registres nécessaires

Dans le header :
- Importer les librairies nécessaires
- Remplir la configuration du device (valeurs en lecture seule : bus, constantes de configuration, ...)
- Remplir la data du device (valeurs modifiées à l'utilisation : valeur de registres)
- Déclarer les typedef fonctions
- Remplir l'API avec les fonctions souhaitées
- Pour chaque fonction 2 déclarations :
    - syscall : permet à Zephyr d'appeler la fonction depuis l'API
    - z_impl : fait la relation entre le syscall et la fonction C
- Enfin inclure le fichier __syscalls/device.h__

**NB :** le fichier syscall est créé lors de la première compilation, ainsi il n'existe pas encore et crée une erreur, il suffit de compiler une seconde fois pour qu'elle s'efface

## zephyr

Le fichier __module.yml__ indique à zephyr la position des différents composants du module. En respectant la structure du template rien à modifier ici.

## .c

Le premier define est en relation avec la mention de compatibilité créé (fichier yaml)

Le fichier contient ensuite la définition des fonctions. Celles-ci sont utilisées pour remplir l'API

Il faut ensuite créer la structure de données du device ainsi que la fonction __init__ appelée automatiquement lors de la création

Enfin utiliser la macro pour créer l'instance à partir du DTS
```c
DEVICE_DT_INST_DEFINE(  inst,
                        init_function,
                        power_management_isr,
                        data_structure_ptr,
                        config_structure_ptr,
                        init_timing,
                        init_prio,
                        api_structure_ptr);

DT_INST_FOREACH_STATUS_OKAY(TEMPLATE_DEFINE)
```

La dernière ligne permet de générer automatiquement l'instance pour chaque device présent dans le devicetree

## CMakeLists

Inclus les fichiers c et le fichier syscall lors de la compilation

## KConfig

Permet de définir les flags de config (sélectionnable ou non) qui peuvent être appelés dans le C.

Défini aussi les dépendances et les librairies importées par le driver