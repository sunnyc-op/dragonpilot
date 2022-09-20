#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9063551671781569783);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3184610972210744396);
void gnss_H_mod_fun(double *state, double *out_5174772375165381252);
void gnss_f_fun(double *state, double dt, double *out_3910409860434011468);
void gnss_F_fun(double *state, double dt, double *out_3815441773285650391);
void gnss_h_6(double *state, double *sat_pos, double *out_4891984551865118720);
void gnss_H_6(double *state, double *sat_pos, double *out_2764292410338766363);
void gnss_h_20(double *state, double *sat_pos, double *out_1944521021824495821);
void gnss_H_20(double *state, double *sat_pos, double *out_7355405256728486282);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3020125195924369443);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8334839139454016519);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3020125195924369443);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8334839139454016519);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}