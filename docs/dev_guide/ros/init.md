# init_\_\.py

When you start the simulation, the {func}`__init__.py` file start and it will have the purpose to create all the file require to start the simulation.
It will first create the {class}`Subscriber` and the {class}`Publisher` objects.

## Publisher

### Class

The Publisher class have the purpose to keep all the information needed. It is basically like a container. Then in the {class}`ROSStandaloneDevice` class a function name add_publisher will be called to create all . 

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

### add_subscriber