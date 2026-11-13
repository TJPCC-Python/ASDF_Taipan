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

struct auton_command_data {
        float timeout_millis;
        bool reverse;
};

enum turn_dir {
        TURN_CCW = -1,
        TURN_NONE = 0,
        TURN_CW = 1
};

enum locked_side {
        LOCKED_LEFT = -1,
        LOCKED_NONE = 0,
        LOCKED_RIGHT = 1
};

struct auton_turn_target {
        bool is_position;
        union as
        {
                vec2 position;
                float heading;
        } as;

        enum turn_dir dir;
        enum locked_side lock_side;
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

struct turn_params {
        struct within_range_data* turn_exit_range_data;

        float exit_range;

        float max_speed;
        float min_speed;

        struct pid_data* turn_pid;

        float slew;
        float dt;
};


void initialise_within_range_data(struct within_range_data* data);
void initialise_pid_data(struct pid_data* data);

// TODO: fix usage
void improved_point_to_point(vec2 target_pos, struct auton_command_data cmd, struct p2p_params params);
void improved_auton_turn(struct auton_turn_target target, struct auton_command_data cmd, struct turn_params params);

void set_motors_brake();
void unset_motors_brake();

#endif
