// -----------------------------------------------------------------------------
// ASDF 2027 'Taipan' language - (c) 2026 Riley Lorenz  & ASDF Robotics
// -----------------------------------------------------------------------------

#include "auton.h"
#include "dead_reckoning.h"
#include "hardware.h"
#include "_math.h"
#include <stdatomic.h>

#define IN_RAD(deg) ((deg) * M_PI / 180.0f)

static int sgn(float v) {
        return (v > 0) - (v < 0);
}

void set_motors_brake() {
        motor_set_brake_mode(motor_left_front, E_MOTOR_BRAKE_BRAKE);
        motor_set_brake_mode(motor_left_mid, E_MOTOR_BRAKE_BRAKE);
        motor_set_brake_mode(motor_left_rear, E_MOTOR_BRAKE_BRAKE);

        motor_set_brake_mode(motor_right_front, E_MOTOR_BRAKE_BRAKE);
        motor_set_brake_mode(motor_right_mid, E_MOTOR_BRAKE_BRAKE);
        motor_set_brake_mode(motor_right_rear, E_MOTOR_BRAKE_BRAKE);
}

void unset_motors_brake() {
        motor_set_brake_mode(motor_left_front, E_MOTOR_BRAKE_COAST);
        motor_set_brake_mode(motor_left_mid, E_MOTOR_BRAKE_COAST);
        motor_set_brake_mode(motor_left_rear, E_MOTOR_BRAKE_COAST);

        motor_set_brake_mode(motor_right_front, E_MOTOR_BRAKE_COAST);
        motor_set_brake_mode(motor_right_mid, E_MOTOR_BRAKE_COAST);
        motor_set_brake_mode(motor_right_rear, E_MOTOR_BRAKE_COAST);
}

#define SPIN_DT_MOTORS(power_l, power_r) \
        do { \
                motor_move(motor_left_front, (power_l)); \
                motor_move(motor_left_mid, (power_l)); \
                motor_move(motor_left_rear, (power_l)); \
        \
                motor_move(motor_right_front, (power_r)); \
                motor_move(motor_right_mid, (power_r)); \
                motor_move(motor_right_rear, (power_r)); \
        } while (0);

#define BRAKE_DT_MOTORS() \
        do { \
                motor_brake(motor_left_front); \
                motor_brake(motor_left_mid); \
                motor_brake(motor_left_rear); \
        \
                motor_brake(motor_right_front); \
                motor_brake(motor_right_mid); \
                motor_brake(motor_right_rear); \
        } while (0)

inline static bool is_time_expired(float start_time, float amount) {
        if (millis() - start_time >= amount) return true;
        return false;
}

void initialise_pid_data(struct pid_data* pid) {
        // human settable
        pid->kp = 0;
        pid->ki = 0;
        pid->kd = 0;
        pid->anti_windup_range = 0;

        // internal
        pid->integral = 0;
        pid->prev_err = 0;
        pid->prev_time = -1;
}

#define TO_SEC(millis) ((millis) / 1000.0f)

static float pid_data_update(struct pid_data* pid, float err) {
        const float now = millis();
        float dt = (pid->prev_time == -1) ? 0 : now - pid->prev_time;
        pid->prev_time = now;

        const float prop = pid->kp * err;
        const float deriv = pid->kd * ((dt != 0) ? (err - pid->prev_err) / TO_SEC(dt) : 0);

        pid->integral += err * dt;
        if (sgn(err) != sgn(pid->prev_err)) pid->integral = 0;
        if (fabs(err) > pid->anti_windup_range && pid->anti_windup_range != 0) pid->integral = 0;

        const float integ = pid->ki * pid->integral;

        pid->prev_err = err;

        return prop + integ + deriv;
}

static float get_angle_err(float t_angle, float actual_angle, enum turn_dir dir) {
        while (t_angle < 0) t_angle += 360;
        while (t_angle > 360) t_angle -= 360;

        float error = t_angle - actual_angle;
        if (dir == TURN_NONE) return remainderf(error, 360);

        if (dir == TURN_CCW) return error < 0 ? error + 360 : error;
        else return error > 0 ? error - 360 : error;
}

static bool within_range_for_time(float input, struct within_range_data* data) {
        if (data->start_time == -1) data->start_time = millis();

        const float curr_time = millis();
        if (fabs(input) >= data->range) data->start_time = -1;
        else if (curr_time >= data->start_time + data->time) data->is_done = true;

        return data->is_done;
}

void initialise_within_range_data(struct within_range_data* data) {
        data->start_time = -1; // sentinel value
        data->time = -1;
        data->range = -1;

        data->is_done = false;
}

enum prev_side {
        PREVSIDE_NONE = -1,
        PREVSIDE_DIFFERENT = 0,
        PREVSIDE_SAME = 1
};

static enum slew_direction {
        SLEW_DECREASING = -1,
        SLEW_NONE = 0,
        SLEW_INCREASING = 1
};

static float slew(float target, float curr, float max_delta, float dt) {
        directional_slew(target, curr, max_delta, dt, SLEW_NONE);
}


static float directional_slew(float target, float curr, float max_delta, float dt, enum slew_direction dir) {
        if (max_delta == 0) return target;

        const float delta = target - curr;

        if (dir == SLEW_INCREASING && delta < 0) return target;
        if (dir == SLEW_DECREASING && delta > 0) return target;

        if (fabs(delta) > fabs(max_delta * dt)) return curr + (max_delta * dt * sgn(delta));

        return target;
}

struct out_data
{
        float err;
        float* prev_out;
        struct pid_data* pid;
        float slew_amt;
        float max_speed;
        float min_speed;
        bool is_reversed;
        bool is_close;

        float dt;
};

static float get_lat_out(struct out_data params) {
        float out = pid_data_update(params.pid, params.err);
        out = clamp(out, -params.max_speed, params.max_speed);

        out = params.is_close ? out : slew(out, *params.prev_out, params.slew_amt, params.dt);

        if (!params.is_close && params.is_reversed) out = clamp(out, -params.max_speed, -params.min_speed);
        else if (!params.is_close && !params.is_reversed) out = clamp(out, params.min_speed, params.max_speed);

        *params.prev_out = out;
        return out;
}

static float get_ang_out(struct out_data params) {
        if (params.is_close && fabsf(params.err) > 40.0f) return 0;

        float out = pid_data_update(params.pid, params.err);

        out = clamp(out, -params.max_speed, params.max_speed);

        out = slew(out, *params.prev_out, params.slew_amt, params.dt);

        *params.prev_out = out;
        return out;
}

#define ANGLE_TO(delta) (argd((delta)) - 90.0f)

static vec2 desaturate_power(float lat, float ang) {
        vec2 out;
        out.x = lat - ang;
        out.y = lat + ang;

        const float sum = fmax(fabs(out.x), fabs(out.y));
        if (sum <= 1.0) return out;

        out.x /= sum;
        out.y /= sum;
        return out;
}

static float calc_angle_err(struct auton_turn_target target) {
        if (target.is_position) return ANGLE_TO(target.as.position);
        else return get_angle_err(target.as.heading, atomic_load(&heading), TURN_NONE);
}

static enum slew_direction get_slew_dir(struct auton_turn_target target) {
        if (target.dir == TURN_CCW) return SLEW_INCREASING;
        if (target.dir == TURN_CW) return SLEW_DECREASING;

        float error = calc_angle_err(target);
        if (error > 0) return SLEW_INCREASING;
        else return SLEW_DECREASING;
}

struct motor_brake_set {
        motor_brake_mode_e_t lf;
        motor_brake_mode_e_t lr;

        motor_brake_mode_e_t rf;
        motor_brake_mode_e_t rr;
};

// TODO: Check drive motor ports

static void get_motor_brake_status(struct motor_brake_set* brake_status) {
        brake_status->lf = motor_get_brake_mode(motor_left_front);
        brake_status->lr = motor_get_brake_mode(motor_left_rear);

        brake_status->rf = motor_get_brake_mode(motor_right_front);
        brake_status->rr = motor_get_brake_mode(motor_right_rear);
}

static void set_motor_brake_status(struct motor_brake_set brake_status) {

};

void improved_auton_turn(struct auton_turn_target target, struct p2p_command_data cmd, struct p2p_params params) {
        // TODO: reset structs

        const enum slew_direction slew_dir = get_slew_dir(target);

        float prev_raw_dtheta = 0;
        float prev_dtheta = 0;
        bool has_prev_raw_dtheta = false;
        bool has_prev_dtheta = false;

        float dtheta = INFINITY;
        bool is_settling = false;
        float prev_motor_power = 0;

        // Restore the brake modes of the motors
        const motor_brake_mode_e_t left_brake_mode = motor_get_brake_mode(motor_left_mid);
}

void improved_point_to_point(vec2 target_pos, struct p2p_command_data cmd, struct p2p_params params) {
        if (params.lat_exit_range_data != NULL) {
                params.lat_exit_range_data->start_time = -1;
                params.lat_exit_range_data->is_done = false;
        }

        if (params.lat_pid != NULL) {
                params.lat_pid->integral = 0;
                params.lat_pid->prev_err = 0;
                params.lat_pid->prev_time = -1;
        }

        if (params.ang_pid != NULL) {
                params.ang_pid->integral = 0;
                params.ang_pid->prev_err = 0;
                params.ang_pid->prev_time = -1;
        }

        vec2 initial_delta;
        initial_delta.x = target_pos.x - atomic_load(&curr_x);
        initial_delta.y = target_pos.y - atomic_load(&curr_y);

        const float initial_angle = ANGLE_TO(initial_delta);
        const float start_millis = millis();

        float prev_lat_out = 0.0f;
        float prev_ang_out = 0.0f;

        enum prev_side previous_side = PREVSIDE_NONE;

        vec2 pos, delta;

        bool is_close = false;

        while (!is_time_expired(start_millis, cmd.timeout_millis)) {
                // Not totally threadsafe since y may be updated after x has been read
                pos.x = atomic_load(&curr_x);
                pos.y = atomic_load(&curr_y);

                delta.x = target_pos.x - pos.x;
                delta.y = target_pos.y - pos.y;

                const float dist_to_target = get_mag(delta);

                if (!is_close && dist_to_target < 20.0f) {
                        is_close = true;

                        params.max_lat_speed = fmax(fabs(prev_lat_out), 4.7);
                        params.max_ang_speed = fmax(fabs(prev_ang_out), 4.7);
                }

                float h = atomic_load(&heading);

                const float lat_err = dist_to_target * cos(IN_RAD(get_angle_err(h, ANGLE_TO(delta), TURN_NONE)));
                const float ang_err = get_angle_err(cmd.reverse ? h + 180.0f : h, ANGLE_TO(delta), TURN_NONE);

                if (within_range_for_time(lat_err, params.lat_exit_range_data) && is_close) break;

                const bool side = (pos.x - target_pos.x) * -sin(IN_RAD(initial_angle)) + (pos.y - target_pos.y) * cos(IN_RAD(initial_angle)) <= -params.exit_range;
                if (previous_side == PREVSIDE_NONE) previous_side = (int)side;
                const bool is_same_side = (side == (int)previous_side);

                if (!is_same_side && params.min_lat_speed != 0) break;
                previous_side = (int)side;

                struct out_data lat_out_params;
                lat_out_params.err = lat_err;
                lat_out_params.prev_out = &prev_lat_out;
                lat_out_params.pid = params.lat_pid;
                lat_out_params.slew_amt = params.lat_slew;
                lat_out_params.max_speed = params.max_lat_speed;
                lat_out_params.min_speed = params.min_lat_speed;
                lat_out_params.is_reversed = cmd.reverse;
                lat_out_params.is_close = is_close;
                lat_out_params.dt = params.dt;

                struct out_data ang_out_params;
                ang_out_params.err = ang_err;
                ang_out_params.prev_out = &prev_ang_out;
                ang_out_params.pid = params.ang_pid;
                ang_out_params.slew_amt = params.ang_slew;
                ang_out_params.max_speed = params.max_ang_speed;
                ang_out_params.min_speed = params.min_ang_speed;
                ang_out_params.is_reversed = cmd.reverse;
                ang_out_params.is_close = is_close;
                ang_out_params.dt = params.dt;

                float lat_out = get_lat_out(lat_out_params);
                const float ang_out = get_ang_out(ang_out_params);

                if (is_close)
                        lat_out *= cosf(IN_RAD(ang_err));

                const vec2 out = desaturate_power(lat_out, ang_out);

                screen_print(E_TEXT_MEDIUM, 3, "Moving with {%.3f} lat, {%.3f} ang", lat_out, ang_out);
                screen_print(E_TEXT_MEDIUM, 4, "...{%.3f} lat err, {%.3f} ang err, {%.3f} dt", lat_err, ang_err, params.dt);
                screen_print(E_TEXT_MEDIUM, 5, "Power: {%.3f}, {%.3f}", out.x, out.y);

                SPIN_DT_MOTORS(127.0f * out.x, 127.0f * out.y);

                delay(params.dt);
        }

        BRAKE_DT_MOTORS();
}
