/*
 * Continuous-time models of machine drives for the simulation (see sim.h), ported
 * from motulator.
 *
 * Python counterparts:
 *   motulator/drive/model/_drive.py       Drive
 *   motulator/drive/model/_machine.py     SynchronousMachine, InductionMachine
 *   motulator/drive/model/_mechanics.py   MechanicalSystem, ExternalRotorSpeed
 *   motulator/drive/utils/_parameters.py  SynchronousMachinePars,
 *                                         SpatialSaturatedSynchronousMachinePars,
 *                                         InductionMachinePars
 *
 * The machine is a synchronous machine, whose magnetic model has constant inductances
 * (SynchronousMachinePars) or is a GradNet current map with spatial harmonics
 * (CurrentMapWithHarmonics in SpatialSaturatedSynchronousMachinePars), or an
 * induction machine with constant parameters (the Γ model of InductionMachinePars).
 * The core losses are not modeled (G_c = 0), and the measurements are ideal. The
 * converter has a constant DC-bus voltage, or it is fed by a DC-bus model (see
 * sim.h).
 *
 * The states are those of motulator, in the same order: the states of the DC bus,
 * the stator flux linkage in rotor coordinates and the electrical rotor angle of the
 * synchronous machine (or the stator and rotor flux linkages in stator coordinates of
 * the induction machine), and the mechanical rotor angle and speed of the mechanics
 * (MechanicalSystem), or the mechanical rotor angle (ExternalRotorSpeed, whose speed
 * is a function of time).
 */

#ifndef MOTULATOR_DRIVE_H
#define MOTULATOR_DRIVE_H

#include "common.h"
#include "gradnet.h"
#include "sim.h"

/* Indices of the states after those of the DC bus */
#define DRIVE_psi_s 0   /* psi_s_dq (synchronous) or psi_s_ab (induction machine) */
#define DRIVE_theta_m 1 /* Synchronous machine */
#define DRIVE_psi_r 1   /* psi_r_ab (induction machine) */
#define DRIVE_theta_M 2
#define DRIVE_w_M 3
#define SM_DRIVE_STATES "psi_s_dq theta_m theta_M w_M"
#define IM_DRIVE_STATES "psi_s_ab psi_r_ab theta_M w_M"
/* The states with the external rotor speed (ExternalRotorSpeed) */
#define SM_DRIVE_STATES_EXT "psi_s_dq theta_m theta_M"
#define IM_DRIVE_STATES_EXT "psi_s_ab psi_r_ab theta_M"

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

/* Γ model with constant parameters (InductionMachinePars) */
typedef struct {
    double n_p;
    double R_s;
    double R_r;
    double L_ell;
    double L_s;
} InductionMachine;

/* MechanicalSystem, or ExternalRotorSpeed (external nonzero), whose speed w_M is a
 * function of time and whose only state is the rotor angle */
typedef struct {
    double J;
    Signal B_L;   /* Friction coefficient (Nm/(rad/s)), a function of the speed */
    Signal tau_L; /* External load torque (Nm), a function of time */
    int external; /* ExternalRotorSpeed (else MechanicalSystem) */
    Signal w_M;   /* External rotor speed (rad/s), a function of time */
} MechanicalSystem;

/* Drive (Drive with SynchronousMachine or InductionMachine and MechanicalSystem or
 * ExternalRotorSpeed) */
typedef struct {
    Model base;
    int induction; /* InductionMachine (else SynchronousMachine) */
    SynchronousMachine sm;
    InductionMachine im;
    MechanicalSystem mechanics;
    char state_names[96];
} Drive;

/* Set the functions of the model, a synchronous machine, and zero friction and load
 * torque. The
 * parameters of the machine, the mechanics, the converter, the DC bus, and the
 * modulator are set by the API functions. */
static void drive_init(Drive *self);
/* Stator current (A) in rotor coordinates and the electromagnetic torque (Nm) as
 * functions of the stator flux linkage (Vs) in rotor coordinates and the electrical
 * rotor angle (SynchronousMachine.compute_outputs) */
static void sm_magnetic_map(const SynchronousMachine *self, double complex psi_s_dq,
                            double theta_m, double complex *i_s_dq, double *tau_M);
/* Measured converter current (abc2complex(machine.meas_currents())) */
static double complex drive_meas_current(const Drive *self, const double complex *x);
/* Measured mechanical rotor speed (mechanics.meas_speed()) at the time t */
static double drive_meas_speed(const Drive *self, double t, const double complex *x);
/* Measured mechanical rotor angle (wrap(mechanics.meas_position())) */
static double drive_meas_position(const Drive *self, const double complex *x);

#endif
