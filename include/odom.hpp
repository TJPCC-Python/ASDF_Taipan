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
        template<typename radius_unit = mm, typename offset_unit = mm>
        tracking_wheel(int8_t port, radius_unit radius, offset_unit offset) 
        : circ(2.0 * M_PI * mm(radius).internal()), offset(offset) {
                wheel = std::make_unique<pros::Rotation>(port);
                wheel->set_position(0);

                last_p = wheel->get_position();
        }

        ~tracking_wheel() {};

        template<typename T = mm>
        T get_delta() {
                cdeg pos = wheel->get_position();

                printf("pos %.0f\n", pos.internal());
                cdeg raw_delta = pos - last_p;
                last_p = pos;


                printf("raw delta: %.0f\n", raw_delta.internal());
                mm final_result = mm(raw_delta.internal() * cD2rots * circ.internal());
                printf("calced delta: %f\n", final_result.internal());
                return (T)final_result;
        }

        template<typename T = mm>
        T get_offset() {
                return offset;
        }

private:
        std::unique_ptr<pros::Rotation> wheel;
        cdeg last_p;
        const mm circ;
        mm offset;
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
                mm ds = lat_wheel->get_delta();
                mm dr = para_wheel->get_delta();

                printf("lat_pos: %f\n", ds.internal());
                printf("para_pos: %f\n", dr.internal());

                deg h = heading.load();
                if (!std::isfinite(h))
                {
                        printf("Heading not valid.\n");
                        return;
                }

                rad theta = (rad)h;

                rad dtheta = theta - last_theta; // TODO: normalise angle

                last_theta = theta;

                mm ddlx;
                mm ddly;
                if (abs(static_cast<double>(dtheta)) <= 0.01) {
                        ddlx = ds;
                        ddly = dr;
                } else {
                        ddlx = 2 * std::sin(dtheta / 2.0) * ((ds / (dtheta)) + lat_wheel->get_offset<mm>());
                        ddly = 2 * std::sin(dtheta / 2.0) * ((dr / (dtheta)) + para_wheel->get_offset<mm>());
                }

                rad thetam = last_theta + (rad)(dtheta / 2);

                // rotate by -thetam to get to global position space
                mm ddx = ddlx * std::cos(thetam) - ddly * std::sin(thetam);
                mm ddy = ddlx * std::sin(thetam) + ddly * std::cos(thetam);

                x.store(x.load() + ddx);
                y.store(y.load() + ddy);
        }

private:
        tracking_wheel* lat_wheel;
        tracking_wheel* para_wheel;

        mm last_s = 0;
        mm last_r = 0;
        rad last_theta = 0;

        mm offset_s = -60.0;
        mm offset_r = -10.0;
};

#endif
