/*
 * Continuous-time model of a synchronous machine drive for the simulation (see
 * sim.h), ported from motulator.
 *
 * Python counterparts:
 *   motulator/drive/model/_drive.py       Drive
 *   motulator/drive/model/_machine.py     SynchronousMachine
 *   motulator/drive/model/_mechanics.py   MechanicalSystem
 *   motulator/drive/utils/_parameters.py  SynchronousMachinePars,
 *                                         SpatialSaturatedSynchronousMachinePars
 *
 * The magnetic model of the machine has constant inductances
 * (SynchronousMachinePars), or it is a GradNet current map with spatial harmonics
 * (CurrentMapWithHarmonics in SpatialSaturatedSynchronousMachinePars). The core
 * losses are not modeled (G_c = 0), and the measurements are ideal.
 *
 * The states are those of motulator, in the same order: the stator flux linkage in
 * rotor coordinates and the electrical rotor angle of the machine, and the mechanical
 * rotor angle and speed of the mechanics.
 */

#ifndef MOTULATOR_DRIVE_H
#define MOTULATOR_DRIVE_H

#include "common.h"
#include "gradnet.h"
#include "sim.h"

/* Indices of the states */
#define SM_DRIVE_psi_s_dq 0
#define SM_DRIVE_theta_m 1
#define SM_DRIVE_theta_M 2
#define SM_DRIVE_w_M 3
#define SM_DRIVE_STATES "psi_s_dq theta_m theta_M w_M"

typedef struct {
    double n_p;
    double R_s;
    int harmonics; /* GradNet current map with spatial harmonics (else constant
                    * inductances) */
    double L_d;    /* Constant inductances */
    double L_q;
    double psi_f;
    GradNet current_map; /* GradNet current map with spatial harmonics */
    int k;               /* Order of the spatial harmonics */
} SynchronousMachine;

typedef struct {
    double J;
    double B_L;   /* Viscous friction coefficient (Nm/(rad/s)) */
    Signal tau_L; /* External load torque (Nm) */
} MechanicalSystem;

/* Drive (Drive with SynchronousMachine and MechanicalSystem) */
typedef struct {
    Model base;
    SynchronousMachine machine;
    MechanicalSystem mechanics;
} SynchronousMachineDrive;

/* Set the functions of the model and a zero load torque. The parameters of the
 * machine, the mechanics, the converter, and the modulator are set by the caller. */
static void sm_drive_init(SynchronousMachineDrive *self);
/* Stator current (A) in rotor coordinates and the electromagnetic torque (Nm) as
 * functions of the stator flux linkage (Vs) in rotor coordinates and the electrical
 * rotor angle (SynchronousMachine.compute_outputs) */
static void sm_magnetic_map(const SynchronousMachine *self, double complex psi_s_dq,
                            double theta_m, double complex *i_s_dq, double *tau_M);
/* Measured converter current (abc2complex(machine.meas_currents())) */
static double complex sm_drive_meas_current(const SynchronousMachineDrive *self,
                                            const double complex *x);
/* Measured mechanical rotor angle (wrap(mechanics.meas_position())) */
static double sm_drive_meas_position(const SynchronousMachineDrive *self,
                                     const double complex *x);

#endif
