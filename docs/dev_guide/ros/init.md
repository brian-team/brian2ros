# init_\_\.py

When you start the simulation, the `__init__.py` file start and it will have the purpose to create all the file require to start the simulation.
It will first create the {class}`.Subscriber` and the {class}`.Publisher` objects.

## Publisher

### Class

The Publisher class have the purpose to keep all the information needed. It is basically like a container. Then in the {class}`.ROSStandaloneDevice` class a function name add_publisher will be called to create all . 

### add_publisher

- check if object of class Publisher

- check if a header is given, if not look in the HEADER_FILES to see if the header is already know (define header)

- check if the name is provide, if not give a name 

- process the input information :
    ~ check if the format is good 
    ~ add information to the input dict (explain what templater is), add the name and a variable call `value` which is the true name of the input variable internally in brian
- check if the reset is provide
- append all this information to the templater


## Subscriber

### Class

#### Init

It store all  the data in this object then call the fonction {func}`.create_code` 

#### create_code

- create a script c++ which have the purpose to :
        - make brian and ros time the same 
        - add the output variable to the brian code 

### add_subscriber
- check if a header is given, if not look in the HEADER_FILES to see if the header is already know 

- create a variable to store the time of the ROS topic 

- Process the output information :
    - look the type of the output_value and give a out_value in consequence
    - We have to find the name of the variable in Brian (explain or source how name of variable work in Brian)
    - update the `outs` variable of arg
- add all the information in the templater

## Misc

### HEADER_FILE

## LaserScanSubscriber and TwistPublisher

This class are just child of their respective class. They just have the purpose to reduce the time of typing by having some preset.
This class are adapt for the use of a turtlebot using jazzy with topic named `cmd_vel` (for Twist) and `scan` (for)


### compile_source

Sert a remplir tout d'abord les deux fichiers json. Le premier **b_control.txt** sert a transmettre les différentes informations sur les monitors a l'interface. 
on y trouve les ifnormations sur les **publishers**, les **subscriber**, la **duration**, les **pub_monitors** et les **variable_info**.
Ensuite ajoute le dossier **msg** au dossier **brian_project**.
Enfin il lance le **colcon build** avec comme parametre le nombre de cpu disponible la load_limit. Si la **preference** sans interface est validé alors il ne chargera pas le paquet **turtlebot3_gz** et **brian_interface**.

### Data for interface
Liste des informations envoyées a l'interface.

#### publisher
    Varibale global d'environnement, qui est remplie lors de l'appel de la fonction **add_publisher**. Contient le nom du publisher, le nom de son topic avec le type de celui-ci, le header_path(?), le taux de rafraichissement, les valeurs d'entrées et la ou les valeurs d'arret

#### subscriber
    Variable global d'environnement, qui est remplie lors de l'appel de la fonction **add_subscriber**. Contient le nom du subscriber, le nom de son topic avec le type de celui-ci, le header_path(?), le nom entier de la variable de temps associé a ce subscriber et les valeurs de sorties

#### duration
    Store the duration of the simulation

#### pub_monitors
    Variable global d'environnement, qui est remplie lors de l'appel de la fonction **publish_monitors**. A pour but de contenir le nom et le type de chaque monitor.
    Les types de chaque monitor sont les suivants :
    - StateMonitor : FloatStateMonitor
    - SpikeMonitor : Float64
    - PopulationRateMonitor : Float64

#### variable_info
    Part of the variable modifier so it's in building.
    Ajoute le nom, le type et la dimension de toute les variables de tout les NeuronGroup qui appartiennent au **stateupdater**, qui appartiennent a **ArrayVariable** et qui ne sont pas en read_only.