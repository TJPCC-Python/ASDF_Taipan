// -----------------------------------------------------------------------------
// ASDF 2026-27 'Taipan' language - (c) 2026 Riley Lorenz  & ASDF Robotics
// -----------------------------------------------------------------------------

#ifndef asdf_easings_h
#define asdf_easings_h

#include <math.h>

class easing {
        const inline virtual double get(double t) = 0;
};

class ease_in_cubic : public easing {
        const inline double get(double t) override {
                return t * t * t;
        }
};

class ease_in_circ : public easing {
        const inline double get(double t) override {
                return 1 - sqrt(1 - pow(t, 2));
        }
};

class ease_in_quint : public easing {
        const inline double get(double t) override {
                return t * t * t * t * t;
        }
};

class ease_cheesy : public easing {
        const inline double get(double t) override {
                double denom = sin(M_PI_2 * turn_nonlinearity);
                double first_remap = sin(M_PI_2 * turn_nonlinearity * t) / denom;
                double second_remap = sin(M_PI_2 * turn_nonlinearity * first_remap) / denom;

                return second_remap;
        }

        private:
        double turn_nonlinearity = 0.8;
};

#endif
