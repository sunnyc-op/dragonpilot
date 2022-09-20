#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_3169895717458430692);
void live_err_fun(double *nom_x, double *delta_x, double *out_5960126488089186864);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2276241451244496957);
void live_H_mod_fun(double *state, double *out_5684835422488032446);
void live_f_fun(double *state, double dt, double *out_933963577286632326);
void live_F_fun(double *state, double dt, double *out_3138669316175151833);
void live_h_4(double *state, double *unused, double *out_1226409472811819737);
void live_H_4(double *state, double *unused, double *out_1732689352410962800);
void live_h_9(double *state, double *unused, double *out_3068773126775602675);
void live_H_9(double *state, double *unused, double *out_5554529582853484670);
void live_h_10(double *state, double *unused, double *out_4151377405693917610);
void live_H_10(double *state, double *unused, double *out_8439795331779786379);
void live_h_12(double *state, double *unused, double *out_8847172595912409067);
void live_H_12(double *state, double *unused, double *out_3286767055620998995);
void live_h_35(double *state, double *unused, double *out_3026637623320665145);
void live_H_35(double *state, double *unused, double *out_6032330087946012704);
void live_h_32(double *state, double *unused, double *out_4009636241033453827);
void live_H_32(double *state, double *unused, double *out_8737354600847762641);
void live_h_13(double *state, double *unused, double *out_5802952281715000435);
void live_H_13(double *state, double *unused, double *out_131055384060839772);
void live_h_14(double *state, double *unused, double *out_3068773126775602675);
void live_H_14(double *state, double *unused, double *out_5554529582853484670);
void live_h_33(double *state, double *unused, double *out_7466372281167726913);
void live_H_33(double *state, double *unused, double *out_9182887092584870308);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}