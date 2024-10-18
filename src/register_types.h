#ifndef GDEXAMPLE_REGISTER_TYPES_H
#define GDEXAMPLE_REGISTER_TYPES_H

#include <godot_cpp/core/class_db.hpp>
//Modules
#include "mirena_car.hpp"
#include "mirena_cam.hpp"
#include "mirena_lidar.hpp"

using namespace godot;

void mirenasim_init(ModuleInitializationLevel p_level);
void mirenasim_deinit(ModuleInitializationLevel p_level);

#endif // GDEXAMPLE_REGISTER_TYPES_H