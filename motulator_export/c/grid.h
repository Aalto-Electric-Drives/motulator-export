/*
 * Continuous-time model of grid converter systems for the simulation (see sim.h),
 * ported from motulator.
 *
 * Python counterparts:
 *   motulator/grid/model/_converter_system.py  GridConverterSystem
 *   motulator/grid/model/_ac_filter.py         LFilter, LCLFilter
 *   motulator/grid/model/_ac_source.py         ThreePhaseSource
 *
 * The AC filter is an L filter or an LCL filter with an inductive-resistive grid
 * impedance, and the grid is a three-phase voltage source, whose angular frequency
 * and the magnitudes and phase shifts of the positive- and negative-sequence
 * components are signals of time. The converter has a constant DC-bus voltage, or it
 * is fed by a DC-bus model (see sim.h). The measurements are ideal.
 *
 * The states are those of motulator, in the same order: the states of the DC bus,
 * the converter current (L filter) or the converter current, the capacitor voltage,
 * and the grid current (LCL filter), and exp(1j*theta_g) of the source.
 */

#ifndef MOTULATOR_GRID_H
#define MOTULATOR_GRID_H

#include "common.h"
#include "sim.h"

/* Indices of the states after those of the DC bus */
#define GRID_i_c_ab 0
#define GRID_u_f_ab 1 /* LCL filter */
#define GRID_i_g_ab 2 /* LCL filter */
#define L_FILTER_STATES "i_c_ab exp_j_theta_g"
#define LCL_FILTER_STATES "i_c_ab u_f_ab i_g_ab exp_j_theta_g"

/* L filter (LFilter) or LCL filter (LCLFilter) with the grid impedance */
typedef struct {
    int lcl; /* LCL filter (else L filter) */
    double L_f, R_f;                    /* L filter */
    double L_fc, L_fg, C_f, R_fc, R_fg; /* LCL filter */
    double L_g, R_g;                    /* Grid impedance */
} ACFilter;

/* Three-phase voltage source (ThreePhaseSource) */
typedef struct {
    Signal w_g;     /* Angular frequency (rad/s) */
    Signal e_g;     /* Magnitude of the positive-sequence component (V), peak */
    Signal phi;     /* Phase shift of the positive-sequence component (rad) */
    Signal e_g_neg; /* Magnitude of the negative-sequence component (V), peak */
    Signal phi_neg; /* Phase shift of the negative-sequence component (rad) */
} ThreePhaseSource;

/* Grid converter system (GridConverterSystem) */
typedef struct {
    Model base;
    ACFilter ac_filter;
    ThreePhaseSource ac_source;
    char state_names[96];
} GridConverterSystem;

/* Set the functions of the model, an L filter, and the defaults of the source. The
 * parameters of the filter, the source, the converter, the DC bus, and the modulator
 * are set by the API functions. */
static void grid_init(GridConverterSystem *self);
/* Voltage (V) of the source at the time t with the state exp_j_theta_g
 * (ThreePhaseSource.generate_space_vector) */
static double complex source_voltage(const ThreePhaseSource *self, double t,
                                     double complex exp_j_theta_g);
/* Measured converter current (abc2complex(ac_filter.meas_currents())) */
static double complex grid_meas_current(const GridConverterSystem *self,
                                        const double complex *x);
/* Measured PCC voltage (line2complex(ac_filter.meas_pcc_voltages())) at the time t */
static double complex grid_meas_pcc_voltage(const GridConverterSystem *self, double t,
                                            const double complex *x);

#endif
