#ifndef COMPAT2_H
#define COMPAT2_H
enum ap_var_type {
    AP_PARAM_NONE    = 0,
    AP_PARAM_INT8,
    AP_PARAM_INT16,
    AP_PARAM_INT32,
    AP_PARAM_FLOAT,
    AP_PARAM_VECTOR3F,
    AP_PARAM_VECTOR6F,
    AP_PARAM_MATRIX3F,
    AP_PARAM_GROUP
};
#endif

#define NOINLINE __attribute__ ((noinline))
