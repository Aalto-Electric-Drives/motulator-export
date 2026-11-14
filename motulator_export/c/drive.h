/*
 * Continuous-time models of machine drives for the simulation (see sim.h), ported
 * from motulator.
 *
 * Python counterparts:
 *   motulator/drive/model/_drive.py       Drive
 *   motulator/drive/model/_machine.py     SynchronousMachine, InductionMachine,
 *                                         _compute_ac_port
 *   motulator/drive/model/_mechanics.py   MechanicalSystem, ExternalRotorSpeed
 *   motulator/drive/utils/_parameters.py  SynchronousMachinePars,
 *                                         SpatialSaturatedSynchronousMachinePars,
 *                                         InductionMachineInvGammaPars,
 *                                         InductionMachinePars
 *
 * The machine is a synchronous machine, whose magnetic model has constant inductances
 * (SynchronousMachinePars) or is a GradNet current map with spatial harmonics
 * (CurrentMapWithHarmonics in SpatialSaturatedSynchronousMachinePars), or an
 * induction machine with constant parameters, whose magnetic model is the inverse-Γ
 * model (InductionMachineInvGammaPars) or the Γ model (InductionMachinePars with a
 * constant L_s) and whose rotor cage is a resistance. Both machines are modeled in
 * rotor coordinates, and the rotor quantities of the induction machine are scaled as
 * in its parameters. The core losses are not modeled (G_c = 0), and the measurements
 * are ideal. The converter has a constant DC-bus voltage, or it is fed by a DC-bus
 * model (see sim.h).
 *
 * The states are those of motulator, in the same order: the states of the DC bus,
 * the states of the machine (the stator flux linkage in rotor coordinates, the rotor
 * flux linkage in rotor coordinates of the induction machine, and the electrical
 * rotor angle), and the mechanical rotor angle and speed of the mechanics
 * (MechanicalSystem), or the mechanical rotor angle (ExternalRotorSpeed, whose speed
 * is a function of time). The cage of the induction machine, being a resistance, has
 * no states.
 */

#ifndef MOTULATOR_DRIVE_H
#define MOTULATOR_DRIVE_H

#include "common.h"
#include "gradnet.h"
#include "sim.h"

/* Indices of the flux linkages after the states of the DC bus. The electrical rotor
 * angle follows them, and the states of the mechanics follow the angle (see
 * drive_theta_m). */
#define DRIVE_psi_s 0 /* psi_s_dq */
#define DRIVE_psi_r 1 /* psi_r_dq of the induction machine */
#define SM_DRIVE_STATES "psi_s_dq theta_m theta_M w_M"
#define IM_DRIVE_STATES "psi_s_dq psi_r_dq theta_m theta_M w_M"
/* The states with the external rotor speed (ExternalRotorSpeed) */
#define SM_DRIVE_STATES_EXT "psi_s_dq theta_m theta_M"
#define IM_DRIVE_STATES_EXT "psi_s_dq psi_r_dq theta_m theta_M"

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

/* Induction machine with constant parameters, whose magnetic model is the inverse-Γ
 * model (InductionMachineInvGammaPars) or the Γ model (InductionMachinePars). Its
 * rotor quantities are scaled as in the parameters, and so is the rotor resistance
 * R_r, which is the cage of the parameters (R_R of the inverse-Γ model). */
typedef struct {
    double n_p;
    double R_s;
    double R_r;
    int inv_gamma; /* Inverse-Γ model (else Γ model) */
    double L_sgm;  /* Inverse-Γ model */
    double L_M;
    double L_ell;  /* Γ model */
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
 * rotor angle (magnetic_map of the parameters of the synchronous machine) */
static void sm_magnetic_map(const SynchronousMachine *self, double complex psi_s_dq,
                            double theta_m, double complex *i_s_dq, double *tau_M);
/* Stator and rotor currents (A) in rotor coordinates and the electromagnetic torque
 * (Nm) as functions of the stator and rotor flux linkages (Vs) in rotor coordinates
 * (magnetic_map of the parameters of the induction machine) */
static void im_magnetic_map(const InductionMachine *self, double complex psi_s_dq,
                            double complex psi_r_dq, double complex *i_s_dq,
                            double complex *i_r_dq, double *tau_M);
/* Measured converter current of the ideal current sensor
 * (VectorControlSystem.get_measurement) */
static double complex drive_meas_current(const Drive *self, const double complex *x);
/* Measured mechanical rotor speed of the ideal speed sensor at the time t */
static double drive_meas_speed(const Drive *self, double t, const double complex *x);
/* Measured mechanical rotor angle of the ideal position sensor, wrapped
 * (VectorControlSystem.get_measurement) */
static double drive_meas_position(const Drive *self, const double complex *x);

#endif
