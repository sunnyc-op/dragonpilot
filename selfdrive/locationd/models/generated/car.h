#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_2990059360614886699);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_316514450138690931);
void car_H_mod_fun(double *state, double *out_2064577180794065928);
void car_f_fun(double *state, double dt, double *out_6429104743904085530);
void car_F_fun(double *state, double dt, double *out_8895463431350518309);
void car_h_25(double *state, double *unused, double *out_3471221924162000290);
void car_H_25(double *state, double *unused, double *out_8244121788343254413);
void car_h_24(double *state, double *unused, double *out_3576466136437052064);
void car_H_24(double *state, double *unused, double *out_4686196189540931277);
void car_h_30(double *state, double *unused, double *out_1955846280475620045);
void car_H_30(double *state, double *unused, double *out_5725788829836005786);
void car_h_26(double *state, double *unused, double *out_3045826987185225600);
void car_H_26(double *state, double *unused, double *out_6461118966492240979);
void car_h_27(double *state, double *unused, double *out_8472895191822845935);
void car_H_27(double *state, double *unused, double *out_3502194758652062569);
void car_h_29(double *state, double *unused, double *out_7026122891531220720);
void car_H_29(double *state, double *unused, double *out_5215557485521613602);
void car_h_28(double *state, double *unused, double *out_4972175614225860036);
void car_H_28(double *state, double *unused, double *out_8148787571118407440);
void car_h_31(double *state, double *unused, double *out_2813899509742510757);
void car_H_31(double *state, double *unused, double *out_5834910864258889503);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}