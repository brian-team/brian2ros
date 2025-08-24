# Interface

## Function

Fichier Python servant à gérer toute les actions des différents boutons ou onglets de l'interface de Brian2ROS.
Les fonctions se trouvent toute dans une classe nommée "B2R_UI_Plugin" qui est une classe qui hérite de Plugin qui appartient à Qt.

### __init__

Dans l'initialisation de la fonction, on crée toute les variables qui seront nécessaire au bon fonctionnement des méthodes de la classe.
Ensuite on ouvre le fichier JSON transmis grâce a __init__.py puis on récupere les information nécessaire au bon fonctionnemrnt des méthodes :
- self.duration : durée de simulation
- self.monitors : Information de tout les monitors 
- self.variable_info : Information de toute les variables à modifier apres une simulation (en construction).
Voir "description json" pour plus de détail.

Puis on crée tout ce qui est nécessaire au bon fonctionnement de l'interface utilisateur cad :
- Après avoir inclut le fichier contenant les description graphique de chaque composant de l'interface on les connectent au bonne fonction 

### publish_stop_brian

Envoie un message par le publisher "publisher_control" pour stoper la simulation Brian.
Cette information sera receptionné par le fichier "brianros.h"

### publish_shutdown_brian
Envoie un message par le publisher "publisher_control" pour arreter complètement la simulation (cad arret de l'interface et donc impossiblité de restart la simulation).

### empty_file

Focntion qui a pour but de vider un dossier. Cette fonction sera utilisé pour vider le dossier des résultats au lancement de chaque simulation pour eviter la demande d'affichage des resultats d'une ancienne simulation pendant la nouvelle simulation.

### is_file_empty

Fonction ayant pour but de verifier si un dossier est vide ou non (renvoie True ou False). Permet de s'assurer que le dossier est bien vide et permet de bloquer l'utilisation du visionnaire de résultat lorsque le dossier est vide.

### show_results

Fonction qui va ouvrir la fenetre de visionnage de resultat. Ne peut etre lancé qu'à la fin de chaque simulation car ne peut etre lancé que si le dossier result est remplie.


### find_file_name
Cette méthode va chercher dans le dossier result tout les fichiers ayant "statemonitor", "spikemonitor" ou "ratemonitor" dans leur nom pour avoir une liste de tout les monitor de la simulation. A chaque monitor on associera un fichier ayant les valeurs temporelles correspondant au monitor. Attention : changer le nom des monitors rend cette méthode inoperante.

### get_value
Cette méthode a pour but de récuperer les valeurs de chaque fichier donnée pour en ressortir le x et le y. En utilisant la fonction numpy.fromfile elle va récuperer les donnée et va ensuite, en fonction du type de monitor, établir ou non un reshpape du tableau de valeur.

### plot_array_2d
Cette méthode va utiliser le self.ui_results pour plot les données transmise a la méthode.

### add_display
Méthode connecté au bouton "+" de l'interface. 
Fonction pour ajouter un plot au visionneur de resultat. La fonction va d'abord récupérer le choix selectionné dans la comboBox (menu defilant) puis l'indice demandé par l'utilisateur dans la fenetre de texte. Puis va vérifier que ce choix n'a pas déja ete affiché. Puis a partir de ces informations va récupérer les noms des fichiers de l'axe x (le temps) et y (les valeurs obtenue). Ensuite, apres avoir verifier qu'un indice (ou numero de courbe) a été selectionné (seul les spikemonitor n'ont pas besoin d'avoir d'indice car ils ont tout le temps qu'un seul array de valeur) la méthode va récupéré les array x et y grace a self.get_value. Enfin elle va afficher les résultats grâce a la fonction self.plot_array_2d puis va ajouter les information a un tableau nommé self.current_plot qui a pour but de réferencer tout les plot actuellement affiché.

### remove_last_display
Méthode connecté au bouton "-" de l'interface. 
