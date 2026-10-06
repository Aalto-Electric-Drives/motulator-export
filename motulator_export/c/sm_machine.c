/*
 * Continuous-time model of a synchronous machine drive, ported from motulator. See
 * sm_machine.h.
 */

#include "sm_machine.h"

/* d-axis current at the d-axis flux linkage psi_d and theta_m = 0 (for brentq) */
static double pm_flux_i_d(double psi_d, void *data)
{
    const SpatialSaturatedSynchronousMachinePars *par = data;
    double complex i_s;
    double tau;
    gradnet_current_map_harmonics(&par->current_map, par->k, psi_d, 0.0, &i_s, &tau);
    return creal(i_s);
}

/* PM-flux linkage from i_d(psi_f) = 0 (_solve_pm_flux): the root is bracketed by
 * doubling the upper bound and then found using Brent's method. Zero if there are no
 * permanent magnets, NAN if the root cannot be bracketed. */
static double pm_flux(const SpatialSaturatedSynchronousMachinePars *par)
{
    void *data = (void *)par;
    double i_lo = pm_flux_i_d(0.0, data);
    if (i_lo >= 0.0) {
        return 0.0; /* No permanent magnets */
    }
    double psi_lo = 0.0, psi_hi = 1e-3;
    double i_hi = pm_flux_i_d(psi_hi, data);
    while (i_hi < 0.0 && psi_hi < 1e3) {
        psi_lo = psi_hi;
        psi_hi = 2.0 * psi_hi;
        i_hi = pm_flux_i_d(psi_hi, data);
    }
    if (isfinite(i_lo) && i_hi >= 0.0) {
        return brentq(pm_flux_i_d, psi_lo, psi_hi, data);
    }
    return NAN;
}

static SpatialSaturatedSynchronousMachinePars spatial_saturated_synchronous_machine_pars(
    double n_p, double R_s, const GradNet *current_map, int k)
{
    SpatialSaturatedSynchronousMachinePars par;
    par.n_p = n_p;
    par.R_s = R_s;
    par.current_map = *current_map;
    par.k = k;
    par.psi_f = pm_flux(&par);
    if (par.psi_f < 1e-3) {
        par.psi_f = 0.0; /* No permanent magnets */
    }
    return par;
}

static void machine_magnetic_map(const SpatialSaturatedSynchronousMachinePars *par,
                                 double complex psi_s_dq, double theta_m,
                                 double complex *i_s_dq, double *tau_M)
{
    double tau_m;
    gradnet_current_map_harmonics(&par->current_map, par->k, psi_s_dq, theta_m, i_s_dq,
                                  &tau_m);
    *tau_M = par->n_p * tau_m;
}

static double complex machine_flux_rhs(const SpatialSaturatedSynchronousMachinePars *par,
                                       double complex u_s_ab, double complex psi_s_dq,
                                       double theta_M, double w_M)
{
    double theta_m = par->n_p * theta_M;
    double complex i_s_dq;
    double tau_M;
    machine_magnetic_map(par, psi_s_dq, theta_m, &i_s_dq, &tau_M);
    double complex u_s_dq = u_s_ab * cexp(-I * theta_m);
    return u_s_dq - par->R_s * i_s_dq - I * par->n_p * w_M * psi_s_dq;
}

static void machine_rhs(const SpatialSaturatedSynchronousMachinePars *par, double J,
                        double complex u_s_ab, double tau_L, double complex psi_s_dq,
                        double theta_M, double w_M, double complex *d_psi_s_dq,
                        double *d_theta_M, double *d_w_M)
{
    double complex i_s_dq;
    double tau_M;
    machine_magnetic_map(par, psi_s_dq, par->n_p * theta_M, &i_s_dq, &tau_M);
    *d_psi_s_dq = machine_flux_rhs(par, u_s_ab, psi_s_dq, theta_M, w_M);

    /* Mechanical system */
    *d_theta_M = w_M;
    *d_w_M = (tau_M - tau_L) / J;
}
