// -----------------------------------------------------------------------------
// ASDF 2026 'Taipan' language - (c) 2026 Riley Lorenz  & ASDF Robotics
// -----------------------------------------------------------------------------

#ifndef ODOM_H
#define ODOM_H

#include "sensor_system.hpp"
#include "units.hpp"

#include <inttypes.h>
#include <memory>
#include <cmath>

extern "C" {
#include "api.h"
#include "hardware.h"
};

constexpr double cD2rots = 1.0 / 36000.0;


class tracking_wheel {
public:
        tracking_wheel(int8_t port, double radius, double offset)
                : circ(2.0 * M_PI * radius), offset(offset), port(port) {}

        ~tracking_wheel() {};

        void init_tracking_wheel() {
                wheel = std::make_unique<pros::Rotation>(port);
                wheel->set_position(0);

                last_p = wheel->get_position();
                printf("posi %.0f\n", last_p);
                printf("wheel_pointer: %xu\n", wheel.get());
        }

        double get_delta() {
                double pos = wheel->get_position();

                double raw_delta = pos - last_p;
                last_p = pos;

                double final_result = raw_delta * cD2rots * circ;
                return final_result;
        }

        double get_offset() {
                return offset;
        }

        double get_total_length() {
                return wheel->get_position() * cD2rots * circ;
        }

private:
        std::unique_ptr<pros::Rotation> wheel;
        double last_p;
        const double circ;
        double offset;
        int port;
};

class straight_odom_2_wheel : public sensor_sys { // Make an arc_odom
public:
        straight_odom_2_wheel(tracking_wheel& lateral_wheel, tracking_wheel& parallel_wheel) {
                x.store(0);
                y.store(0);

                lat_wheel = &lateral_wheel;
                para_wheel = &parallel_wheel;
        };

        void calc_position() override {
                double ds = -lat_wheel->get_delta();
                double dr = -para_wheel->get_delta();

                // printf("lat_pos: %f\n", ds);
                // printf("para_pos: %f\n", dr);

                double h = heading.load();
                if (!std::isfinite(h))
                {
                        printf("Heading not valid.\n");
                        return;
                }

                double theta = h * (M_PI / 180.0f);
                while (theta > M_PI) theta -= M_TWOPI;
                while (theta < -M_PI) theta += M_TWOPI;

                double dtheta = theta - last_theta; // TODO: normalise angle

                double ddlx;
                double ddly;
                if (fabs(dtheta) <= 0.01f) {
                        ddlx = ds;
                        ddly = dr;
                } else {
                        ddlx = 2 * std::sin(dtheta / 2.0) * ((dr / (dtheta)) + para_wheel->get_offset());
                        ddly = 2 * std::sin(dtheta / 2.0) * ((ds / (dtheta)) + lat_wheel->get_offset());
                }

                double thetam = last_theta + (dtheta / 2);

                // rotate by -thetam to get to global position space
                double ddx = ddlx * std::cos(thetam) - ddly * std::sin(thetam);
                double ddy = ddlx * std::sin(thetam) + ddly * std::cos(thetam);

                last_theta = theta;

                x.store(x.load() + ddx);
                y.store(y.load() + ddy);

                curr_x.store(x.load());
                curr_y.store(y.load());
        }

private:
        tracking_wheel* lat_wheel;
        tracking_wheel* para_wheel;

        double last_s = 0;
        double last_r = 0;
        double last_theta = 0;

        double offset_s = -60.0;
        double offset_r = -10.0;
};

#endif
