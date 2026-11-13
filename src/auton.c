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

static float get_angle_err(float t_angle, float actual_angle) {
        while (t_angle < 0) t_angle += 360;
        while (t_angle > 360) t_angle -= 360;

        float err = t_angle - actual_angle;
        return remainderf(err, 360);
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
        // TODO:
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

static void internal_do_auton_turn() {
        // Shared code
}

void improved_auton_turn(vec2 target_pos, struct p2p_command_data cmd, struct p2p_params params) {

}

void improved_auton_turn(float target_heading, struct p2p_command_data cmd, struct p2p_params params) {
        // TODO:
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

                const float lat_err = dist_to_target * cos(IN_RAD(get_angle_err(h, ANGLE_TO(delta))));
                const float ang_err = get_angle_err(cmd.reverse ? h + 180.0f : h, ANGLE_TO(delta));

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

void auton_point_to_point(vec2 target, bool drive_reverse) {
        vec2 delta;
        delta.x = target.x - atomic_load(&curr_x);
        delta.y = target.y - atomic_load(&curr_y);

        float dist = get_mag(delta);
        float target_degrees = argd(delta) + (drive_reverse ? 90 : -90.0);
        if (target_degrees < -180) target_degrees += 360;
        printf("delta: %.2f, %.2f\n", delta.x, delta.y);
        printf("Target angle: %f\n", target_degrees);

        // Do a simple P (without I D) to point roughly at target
        float turn_kp = 8.0f;
        float turn_ki = 0.0f;
        float turn_kd = 0.8f;
        float turn_integral = 0.0f;
        float turn_power;

        uint32_t timeout_ms = 8000;
        uint32_t start_time = millis();
        int settling_counter = 0;
        int settling_target = 10;

        float last_err;

        do {
                float turn_error = shd(atomic_load(&heading), target_degrees);

                float prop = clamp(turn_kp * turn_error, -127, 127);
                float deriv = clamp(turn_kd * (last_err - turn_error), -127, 127);

                turn_integral *= 0.95; // counter integral windup
                turn_integral += turn_error;
                turn_integral = clamp(turn_integral, -1000.0f, 1000.0f); // check clamp values, this also is for integral winup
                float integ = turn_ki * turn_integral;
                integ = clamp(integ, -127.0f, 127.0f); // unneccesary?


                float turn_power = prop + integ + deriv;

                turn_power = clamp(turn_power, -127, 127);

                int dt_left = turn_power;
                int dt_right = -turn_power;

                motor_move(motor_left_front, dt_left);
                motor_move(motor_left_mid, dt_left);
                motor_move(motor_left_rear, dt_left);

                motor_move(motor_right_front, dt_right);
                motor_move(motor_right_mid, dt_right);
                motor_move(motor_right_rear, dt_right);

                screen_print(E_TEXT_MEDIUM, 0, "Error: %f", turn_error);

                delay(10);

                if (fabs(turn_error) <= 1.0f) {
                        settling_counter++;
                } else {
                        settling_counter = 0;
                }

                if (settling_counter >= settling_target) {
                        break;
                }

                if (millis() - start_time > timeout_ms) {
                        printf("Auton Warning: Point-to-point timed out!\n");
                        break;
                }

                last_err = turn_error;

                delay(10);
        } while (true);

        // PID with heading lock
        float dkp = 0.35; // P to distance -> roughly speed
        float dki = 0.00;
        float dkd = 0.08;
        float integral = 0;
        float hkp = 0.05; // P to heading error -> heading lock gain | must be less than 0 or robot will not be able to drive straight

        dist = 99999;
        float last_dist = 999999;
        float dz = 1.0f;
        float fpower;

        timeout_ms = 3500;
        start_time = millis();
        settling_counter = 0;
        settling_target = 10;

        do {
                delta.x = target.x - atomic_load(&curr_x);
                delta.y = target.y - atomic_load(&curr_y);

                dist = get_mag(delta);
                float h = atomic_load(&heading) + 90;

                printf("delta: %.2f, %.2f\n", delta.x, delta.y);
                printf("dist: %.2f\n", dist);
                screen_print(E_TEXT_MEDIUM, 0, "delta: %.2f, %.2f\n", delta.x, delta.y);
                screen_print(E_TEXT_MEDIUM, 1, "dist: %.2f\n", dist);
                fpower = dkp * dist;
                float dot = delta.x * cos(h * (M_PI / 180.0f)) + delta.y * sin(h * (M_PI / 180.0f));

                if (dot < 0) fpower = -dkp * dist;

                float target_h = argd(delta);

                static float frozen_target_heading = 0;
                static bool heading_frozen = false;

                frozen_target_heading = target_h;
                if (dist > 5.0f) {
                        heading_frozen = false;
                } else if (!heading_frozen) {
                        heading_frozen = true;
                }

                float active_target_heading = (dist <= 5.0f) ? frozen_target_heading : target_h;

                float h_error = shd(h, active_target_heading);

                screen_print(E_TEXT_MEDIUM, 2, "h_err: %.2f", h_error);

                float turn_correct = heading_frozen ? 0 : h_error * hkp;

                if (dist < dz) fpower = 0;

                fpower = clamp(fpower, -127, 127);
                turn_correct = clamp(turn_correct, -50, 50);

                int dt_left = fpower - turn_correct;
                int dt_right = fpower + turn_correct;

                dt_left = clamp(dt_left, -127, 127);
                dt_right = clamp(dt_right, -127, 127);

                screen_print(E_TEXT_MEDIUM, 4, "%i >  |^|  < %i", dt_left, dt_right);

                motor_move(motor_left_front, dt_left); // make this a fucking macro
                motor_move(motor_left_mid, dt_left);
                motor_move(motor_left_rear, dt_left);

                motor_move(motor_right_front, dt_right);
                motor_move(motor_right_mid, dt_right);
                motor_move(motor_right_rear, dt_right);

                last_dist = dist;

                if (fabs(dist) <= dz) {
                        settling_counter++;
                } else {
                        settling_counter = 0;
                }

                if (settling_counter >= settling_target) {
                        break;
                }

                if (millis() - start_time > timeout_ms) {
                        printf("P2P timed out...\n");
                        break;
                }

                delay(10);
        } while (true);
}
