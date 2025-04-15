#include <stdlib.h>
    #include "objects.h"
    #include <ctime>
    
    {{ openmp_pragma('include') }}
    #include "run.h"
    
    {% for codeobj in code_objects | sort(attribute='name') %}
    #include "code_objects/{{codeobj.name}}.h"
    {% for block in codeobj.before_after_blocks %}
    #include "code_objects/{{block}}_{{codeobj.name}}.h"
    {% endfor %}
    {% endfor %}
    
    {% for name in user_headers | sort %}
    #include {{name}}
    {% endfor %}
    
    #include <unistd.h>
    {{report_func|autoindent}}
    #include "std_msgs/msg/float64.hpp"
    #include "std_msgs/msg/string.hpp"
    #include <cmath>
    #include <iostream>
    #include <functional>
    #include <thread>
    
    #include "rclcpp/rclcpp.hpp"
    
    using namespace std::chrono_literals;
    using std::placeholders::_1;
    
    std::shared_ptr<ROS> ros_obj;
    
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> node_executors;
    
    
    
            
    //std::vector<std::string> args;
    std::thread* brian_sim;
    
    
    
    void set_from_command_line(const std::vector<std::string> args)
    {
        for (const auto& arg : args) {
            // Split into two parts
            size_t equal_sign = arg.find("=");
            auto name = arg.substr(0, equal_sign);
            auto value = arg.substr(equal_sign + 1, arg.length());
            brian::set_variable_by_name(name, value);
    }
    }    
        
    
    void brian_running(const std::vector<std::string> args){
        std::cout << "Starting brian_running" << std::endl;
        {{'\n'.join(code_lines['before_start'])|autoindent}}
        brian_start();
        {{'\n'.join(code_lines['after_start'])|autoindent}}
        {
            using namespace brian;
    
            {{ openmp_pragma('set_num_threads') }}
            {{main_lines|autoindent}}
        }
        
        {{'\n'.join(code_lines['before_end'])|autoindent}}
        brian_end();
        {{'\n'.join(code_lines['after_end'])|autoindent}}
        ros_obj->brian_state = false;
        //sleep(1);
        node_executors->remove_node(ros_obj);
        {% for subscriber in subscribers %}
        ros_obj->{{subscriber["name"]}}_test = false;
        {% endfor %}
    }
    
    
    
    
    void brian_control_callback(std::vector<std::string>& args){
    // Callback function to receive commands from interface to start/stop/restart the simulation.
    
       
    
        // Start the simulation
        // If the simulation is already running, print a message and return.
        // Otherwise, create a ROS2 node and start the simulation in a separate thread.
    
        std::cout << "Start simulation" << std::endl;
    
        // Create a ROS2 object wich is shared.
        ros_obj = std::make_shared<ROS>();
        
        // Start the Brian simulation in a separate thread.
        brian_sim = new std::thread(brian_running, args);
        
        // Add the ROS2 node to the executor.
        node_executors->add_node(ros_obj);
        
        // Set the Brian state to true to indicate that the simulation is running.
        ros_obj->brian_state = true;
    
        }
    
    
    int main(int argc, char **argv)
    {
        
    
        std::vector<std::string> args(argv + 1, argv + argc);
        if (args.size() >=2 && args[0] == "--results_dir")
        {
            brian::results_dir = args[1];
            #ifdef DEBUG
            std::cout << "Setting results dir to '" << brian::results_dir << "'" << std::endl;
            #endif
            args.erase(args.begin(), args.begin()+2);
        }
    
        // Start rclcpp
        rclcpp::init(argc, argv);
        
        // Create a MultiThreadedExecutor to run all the thread
        node_executors = rclcpp::executors::MultiThreadedExecutor::make_shared();
    
        // Add and start the control node to the executor
        brian_control_callback(args);
        node_executors->spin();
        rclcpp::shutdown();
    
        return 0;
    }
        