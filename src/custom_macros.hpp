#ifndef CUSTOM_MACROS_HPP
#define CUSTOM_MACROS_HPP

// Property defining macro
#define BIND_PROPERTY(class_name, var_name, var_type) \
    ClassDB::bind_method(D_METHOD("get_" #var_name), &class_name::get_##var_name); \
    ClassDB::bind_method(D_METHOD("set_" #var_name, "_" #var_name), &class_name::set_##var_name); \
    ADD_PROPERTY(PropertyInfo(var_type, #var_name), "set_" #var_name, "get_" #var_name)


#endif