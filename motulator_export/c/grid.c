/*
 * Continuous-time model of grid converter systems, ported from motulator. See
 * grid.h.
 */

#include "grid.h"

static double complex source_voltage(const ThreePhaseSource *self, double t,
                                     double complex exp_j_theta_g)
{
    double e_g = signal_value(&self->e_g, t);
    double phi = signal_value(&self->phi, t);
    double e_g_neg = signal_value(&self->e_g_neg, t);
    double phi_neg = signal_value(&self->phi_neg, t);
    double complex e_g_ab = e_g * exp_j_theta_g * cexp(I * phi);
    /* Negative-sequence component */
    e_g_ab += e_g_neg * conj(exp_j_theta_g * cexp(I * phi_neg));
    return e_g_ab;
}

/* exp(1j*theta_g) of the angle state of the source, wrapped as by
 * ThreePhaseSource.set_states */
static double complex source_rotation(double complex theta_g)
{
    return cexp(I * wrap(creal(theta_g)));
}

/* Number of the states of the filter */
static int filter_states(const ACFilter *self)
{
    return self->lcl ? 3 : 1;
}

/* PCC voltage (V) between the filter and the grid impedance
 * (LFilter.pcc_voltage or LCLFilter.pcc_voltage), at the states x_f of the filter
 * with the converter voltage u_c_ab and the grid voltage e_g_ab */
static double complex pcc_voltage(const ACFilter *self, const double complex *x_f,
                                  double complex u_c_ab, double complex e_g_ab)
{
    if (self->lcl) {
        double L_t = self->L_fg + self->L_g;
        return (self->L_fg * e_g_ab + self->L_g * x_f[GRID_u_f_ab] +
                (self->R_g * self->L_fg - self->R_fg * self->L_g) * x_f[GRID_i_g_ab]) /
               L_t;
    }
    double L_t = self->L_f + self->L_g;
    double complex i_c_ab = x_f[GRID_i_c_ab];
    return (self->L_g * (u_c_ab - self->R_f * i_c_ab) +
            self->L_f * (e_g_ab + self->R_g * i_c_ab)) /
           L_t;
}

/* Converter current (A), the first state of the filter */
static double complex grid_current(const Model *base, const double complex *x)
{
    return x[dc_bus_states(&base->dc_bus) + GRID_i_c_ab];
}

/* State derivatives (DCBus.rhs, LFilter.rhs or LCLFilter.rhs, and
 * ThreePhaseSource.rhs) */
static void grid_rhs(const Model *base, double t, const double complex *x,
                     double complex *dx)
{
    const GridConverterSystem *self = (const GridConverterSystem *)base;
    const ACFilter *f = &self->ac_filter;
    int k = dc_bus_states(&base->dc_bus);
    const double complex *x_f = x + k;
    double complex *dx_f = dx + k;
    int k_s = filter_states(f);
    double complex exp_j_theta_g = source_rotation(x_f[k_s]);
    double complex u_c_ab = model_converter_voltage(base, t, x, NULL);
    double complex e_g_ab = source_voltage(&self->ac_source, t, exp_j_theta_g);

    /* Filter */
    double complex i_c_ab = x_f[GRID_i_c_ab];
    if (f->lcl) {
        double L_t = f->L_fg + f->L_g;
        double R_t = f->R_fg + f->R_g;
        double complex u_f_ab = x_f[GRID_u_f_ab];
        double complex i_g_ab = x_f[GRID_i_g_ab];
        dx_f[GRID_i_c_ab] = (u_c_ab - u_f_ab - f->R_fc * i_c_ab) / f->L_fc;
        dx_f[GRID_u_f_ab] = (i_c_ab - i_g_ab) / f->C_f;
        dx_f[GRID_i_g_ab] = (u_f_ab - e_g_ab - R_t * i_g_ab) / L_t;
    } else {
        double L_t = f->L_f + f->L_g;
        double R_t = f->R_f + f->R_g;
        dx_f[GRID_i_c_ab] = (u_c_ab - e_g_ab - R_t * i_c_ab) / L_t;
    }

    /* Source */
    dx_f[k_s] = signal_value(&self->ac_source.w_g, t);

    /* DC bus */
    dc_bus_rhs(&base->dc_bus, x, converter_dc_current(&base->converter, i_c_ab), dx);
}

/* AC port of the filter for the converter with blocked gates
 * (LFilter.compute_ac_port or LCLFilter.compute_ac_port) */
static void grid_ac_port(const Model *base, double t, const double complex *x,
                         ACPort *port)
{
    const GridConverterSystem *self = (const GridConverterSystem *)base;
    const ACFilter *f = &self->ac_filter;
    const double complex *x_f = x + dc_bus_states(&base->dc_bus);
    double complex i_c_ab = x_f[GRID_i_c_ab];
    double L;
    if (f->lcl) {
        port->e = x_f[GRID_u_f_ab] + f->R_fc * i_c_ab;
        L = f->L_fc;
    } else {
        double complex e_g_ab = source_voltage(&self->ac_source, t,
                                               source_rotation(x_f[filter_states(f)]));
        port->e = e_g_ab + (f->R_f + f->R_g) * i_c_ab;
        L = f->L_f + f->L_g;
    }
    port->i = i_c_ab;
    port->Y[0][0] = 1.0 / L;
    port->Y[0][1] = 0.0 / L;
    port->Y[1][0] = 0.0 / L;
    port->Y[1][1] = 1.0 / L;
}

/* Keep the angle of the source within [-pi, pi) at the start of an integration
 * interval (ThreePhaseSource.set_states) */
static void grid_set_states(const Model *base, double complex *x)
{
    const GridConverterSystem *self = (const GridConverterSystem *)base;
    dc_bus_set_states(&base->dc_bus, x);
    double complex *x_f = x + dc_bus_states(&base->dc_bus);
    int k_s = filter_states(&self->ac_filter);
    x_f[k_s] = wrap(creal(x_f[k_s]));
}

static void grid_init_states(Model *base)
{
    GridConverterSystem *self = (GridConverterSystem *)base;
    const ACFilter *f = &self->ac_filter;
    base->n = dc_bus_states(&base->dc_bus) + filter_states(f) + 1;
    const char *names = f->lcl ? LCL_FILTER_STATES : L_FILTER_STATES;
    model_state_names(base, names, self->state_names, (int)sizeof(self->state_names));
    base->states = self->state_names;
}

static void grid_init(GridConverterSystem *self)
{
    Model *base = &self->base;
    base->rhs = grid_rhs;
    base->converter_current = grid_current;
    base->set_states = grid_set_states;
    base->init_states = grid_init_states;
    base->ac_port = grid_ac_port;
    base->converter.enabled = 1;
    base->dc_bus.type = DC_BUS_NONE;
    self->ac_filter.lcl = 0;
    grid_init_states(base);
    ThreePhaseSource *src = &self->ac_source;
    signal_constant(&src->w_g, 2.0 * M_PI * 50.0);
    signal_constant(&src->e_g, sqrt(2.0 / 3.0) * 400.0);
    signal_constant(&src->phi, 0.0);
    signal_constant(&src->e_g_neg, 0.0);
    signal_constant(&src->phi_neg, 0.0);
}

static double complex grid_meas_current(const GridConverterSystem *self,
                                        const double complex *x)
{
    double i_abc[3];
    complex2abc(grid_current(&self->base, x), i_abc);
    return abc2complex(i_abc);
}

static double complex grid_meas_pcc_voltage(const GridConverterSystem *self, double t,
                                            const double complex *x)
{
    const Model *base = &self->base;
    const double complex *x_f = x + dc_bus_states(&base->dc_bus);
    int k_s = filter_states(&self->ac_filter);
    double complex exp_j_theta_g = source_rotation(x_f[k_s]);
    double complex u_c_ab = model_converter_voltage(base, t, x, NULL);
    double complex e_g_ab = source_voltage(&self->ac_source, t, exp_j_theta_g);
    double complex u_g_ab = pcc_voltage(&self->ac_filter, x_f, u_c_ab, e_g_ab);
    /* Line-to-line voltages u_ab and u_bc (complex2line) */
    double u_abc[3];
    complex2abc(u_g_ab, u_abc);
    double u_line[2] = {u_abc[0] - u_abc[1], u_abc[1] - u_abc[2]};
    return line2complex(u_line);
}

/* API for Python (motulator_export.native) --------------------------------- */

/* The systems of the grid converters embed their simulation as the first member,
 * and its model is a GridConverterSystem. */

static GridConverterSystem *grid_of(Simulation *self)
{
    return (GridConverterSystem *)self->mdl;
}

/* LFilter with the grid impedance */
void set_l_filter(Simulation *self, double L_f, double R_f, double L_g, double R_g)
{
    GridConverterSystem *grid = grid_of(self);
    ACFilter *f = &grid->ac_filter;
    f->lcl = 0;
    f->L_f = L_f;
    f->R_f = R_f;
    f->L_g = L_g;
    f->R_g = R_g;
    grid_init_states(&grid->base);
}

/* LCLFilter with the grid impedance */
void set_lcl_filter(Simulation *self, double L_fc, double L_fg, double C_f,
                    double R_fc, double R_fg, double L_g, double R_g)
{
    GridConverterSystem *grid = grid_of(self);
    ACFilter *f = &grid->ac_filter;
    f->lcl = 1;
    f->L_fc = L_fc;
    f->L_fg = L_fg;
    f->C_f = C_f;
    f->R_fc = R_fc;
    f->R_fg = R_fg;
    f->L_g = L_g;
    f->R_g = R_g;
    grid_init_states(&grid->base);
}

/* Signal of the source, set by signal_steps or signal_function: 0 the angular
 * frequency w_g, 1 the magnitude e_g, 2 the phase shift phi, 3 the magnitude e_g_neg,
 * and 4 the phase shift phi_neg of the negative sequence */
Signal *source_signal(Simulation *self, int k)
{
    ThreePhaseSource *src = &grid_of(self)->ac_source;
    Signal *signals[5] = {&src->w_g, &src->e_g, &src->phi, &src->e_g_neg,
                          &src->phi_neg};
    return (k >= 0 && k < 5) ? signals[k] : NULL;
}
