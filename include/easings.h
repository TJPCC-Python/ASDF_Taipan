// -----------------------------------------------------------------------------
// ASDF 2026-27 'Taipan' language - (c) 2026 Riley Lorenz  & ASDF Robotics
// -----------------------------------------------------------------------------

#ifndef asdf_easings_h
#define asdf_easings_h

#include <math.h>

#define turn_nonlinearity 0.8

const inline double ease_in_cubic(double t) {
        return t * t * t;
}

const inline double ease_in_cubic_boosted(double t, double boost_factor) {
        return (t * t * t) + boost_factor;
}

const inline double ease_in_circ(double t) {
        return 1 - sqrt(1 - pow(t, 2));
}

const inline double ease_in_quint(double t) {
        return t * t * t * t * t;
}

const inline double ease_cheesy(double t) {
        double denom = sin(M_PI_2 * turn_nonlinearity);
        double first_remap = sin(M_PI_2 * turn_nonlinearity * t) / denom;
        double second_remap = sin(M_PI_2 * turn_nonlinearity * first_remap) / denom;

        return second_remap;
}


#endif
