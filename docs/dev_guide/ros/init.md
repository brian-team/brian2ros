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