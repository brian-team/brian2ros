{# USES_VARIABLES { t, _clock_t, _indices, N } #}
{# WRITES_TO_READ_ONLY_VARIABLES { t, N } #}
{% extends 'common_group.cpp' %}
{% block maincode %}
    {{_dynamic_t}}.push_back({{_clock_t}});

    const size_t _new_size = {{_dynamic_t}}.size();
    // Resize the dynamic arrays
    {% for varname, var in _recorded_variables | dictsort %}
    {% set _recorded =  get_array_name(var, access_data=False) %}
    {{_recorded}}.resize(_new_size, _num_indices);
    {% endfor %}

    // scalar code
    const size_t _vectorisation_idx = -1;
    {{scalar_code|autoindent}}

    {{ openmp_pragma('parallel-static') }}
    {% for varname, var in _recorded_variables | dictsort %}
    std::vector<double> _message_record_{{varname}}(_num_indices);
    {% endfor %}
    for (int _i = 0; _i < (int)_num_indices; _i++)
    {
        // vector code
        const size_t _idx = {{_indices}}[_i];
        const size_t _vectorisation_idx = _idx;
        {{vector_code|autoindent}}

        {% for varname, var in _recorded_variables | dictsort %}
        {% set _recorded =  get_array_name(var, access_data=False) %}
        {% if c_data_type(var.dtype) == 'bool' %}
        {{ openmp_pragma('critical') }}
        { // std::vector<bool> is not threadsafe
        {{_recorded}}(_new_size-1, _i) = _to_record_{{varname}};
        }
        {% else %}
        {{_recorded}}(_new_size-1, _i) = _to_record_{{varname}};
        {% endif %}


    //================================//
    // This code is add by brian2ros  //
    //================================//

    // Publish the ask recorded variables
        _message_record_{{varname}}[_i] =_to_record_{{varname}};
    //================================//

        {% endfor %}
    }
    using brian_project::msg::FloatStateMonitor; 
    {% for monitor in pub_monitors %}            
        {% for varname, var in _recorded_variables | dictsort %}
            {% if owner.name+"_"+varname == monitor["name"]%}

    //================================//
    // This code is add by brian2ros  //
    //================================//

    // Publish the ask recorded variables
            FloatStateMonitor message_{{varname}};
            message_{{varname}}.array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
            message_{{varname}}.array.layout.dim[0].size = _num_indices;
            message_{{varname}}.array.data = _message_record_{{varname}};

            static double start_time_{{varname}} = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
            double time_in_seconds_{{varname}} = {{_clock_t}} + start_time_{{varname}};
            int32_t seconds_{{varname}} = static_cast<int32_t>(time_in_seconds_{{varname}});
            uint32_t nanoseconds_{{varname}} = static_cast<uint32_t>((time_in_seconds_{{varname}} - seconds_{{varname}}) * 1e9);

            message_{{varname}}.header.stamp.sec = seconds_{{varname}};
            message_{{varname}}.header.stamp.nanosec = nanoseconds_{{varname}};
            ros_obj->publisher_{{owner.name+"_"+varname}}->publish(message_{{varname}});
    //================================//

            {% endif %}
            {% endfor %}
    {% endfor %}
    {{N}} = _new_size;

{% endblock %}