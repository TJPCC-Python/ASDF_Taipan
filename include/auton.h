// -----------------------------------------------------------------------------
// ASDF 2027 'Taipan' language - (c) 2026 Riley Lorenz  & ASDF Robotics
// -----------------------------------------------------------------------------

#ifndef tp_auton_h
#define tp_auton_h

#include <stdbool.h>

typedef struct {
        float x;
        float y;
} vec2;

struct within_range_data {
        float start_time;
        float range;
        float time;
        bool is_done;
};

struct pid_data {
        float kp;
        float ki;
        float kd;
        float integral;
        float anti_windup_range;

        float prev_err;
        float prev_time;
};

struct p2p_command_data {
        vec2 target;
        float timeout_millis;
        bool reverse;
};

struct p2p_params {
        struct within_range_data* lat_exit_range_data;
        float exit_range;

        float max_lat_speed;
        float min_lat_speed;
        float max_ang_speed;
        float min_ang_speed;

        struct pid_data* lat_pid;
        struct pid_data* ang_pid;

        float lat_slew;
        float ang_slew;

        float dt;
};

void initialise_within_range_data(struct within_range_data* data);
void initialise_pid_data(struct pid_data* data);
void auton_point_to_point(vec2 target, bool drive_reverse);
void ll_point_to_point(struct p2p_command_data, struct p2p_params params);

void set_motors_brake();
void unset_motors_brake();
// void auton_read_sensors();
#endif
