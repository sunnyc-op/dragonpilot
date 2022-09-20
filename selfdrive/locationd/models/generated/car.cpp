#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2990059360614886699) {
   out_2990059360614886699[0] = delta_x[0] + nom_x[0];
   out_2990059360614886699[1] = delta_x[1] + nom_x[1];
   out_2990059360614886699[2] = delta_x[2] + nom_x[2];
   out_2990059360614886699[3] = delta_x[3] + nom_x[3];
   out_2990059360614886699[4] = delta_x[4] + nom_x[4];
   out_2990059360614886699[5] = delta_x[5] + nom_x[5];
   out_2990059360614886699[6] = delta_x[6] + nom_x[6];
   out_2990059360614886699[7] = delta_x[7] + nom_x[7];
   out_2990059360614886699[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_316514450138690931) {
   out_316514450138690931[0] = -nom_x[0] + true_x[0];
   out_316514450138690931[1] = -nom_x[1] + true_x[1];
   out_316514450138690931[2] = -nom_x[2] + true_x[2];
   out_316514450138690931[3] = -nom_x[3] + true_x[3];
   out_316514450138690931[4] = -nom_x[4] + true_x[4];
   out_316514450138690931[5] = -nom_x[5] + true_x[5];
   out_316514450138690931[6] = -nom_x[6] + true_x[6];
   out_316514450138690931[7] = -nom_x[7] + true_x[7];
   out_316514450138690931[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2064577180794065928) {
   out_2064577180794065928[0] = 1.0;
   out_2064577180794065928[1] = 0;
   out_2064577180794065928[2] = 0;
   out_2064577180794065928[3] = 0;
   out_2064577180794065928[4] = 0;
   out_2064577180794065928[5] = 0;
   out_2064577180794065928[6] = 0;
   out_2064577180794065928[7] = 0;
   out_2064577180794065928[8] = 0;
   out_2064577180794065928[9] = 0;
   out_2064577180794065928[10] = 1.0;
   out_2064577180794065928[11] = 0;
   out_2064577180794065928[12] = 0;
   out_2064577180794065928[13] = 0;
   out_2064577180794065928[14] = 0;
   out_2064577180794065928[15] = 0;
   out_2064577180794065928[16] = 0;
   out_2064577180794065928[17] = 0;
   out_2064577180794065928[18] = 0;
   out_2064577180794065928[19] = 0;
   out_2064577180794065928[20] = 1.0;
   out_2064577180794065928[21] = 0;
   out_2064577180794065928[22] = 0;
   out_2064577180794065928[23] = 0;
   out_2064577180794065928[24] = 0;
   out_2064577180794065928[25] = 0;
   out_2064577180794065928[26] = 0;
   out_2064577180794065928[27] = 0;
   out_2064577180794065928[28] = 0;
   out_2064577180794065928[29] = 0;
   out_2064577180794065928[30] = 1.0;
   out_2064577180794065928[31] = 0;
   out_2064577180794065928[32] = 0;
   out_2064577180794065928[33] = 0;
   out_2064577180794065928[34] = 0;
   out_2064577180794065928[35] = 0;
   out_2064577180794065928[36] = 0;
   out_2064577180794065928[37] = 0;
   out_2064577180794065928[38] = 0;
   out_2064577180794065928[39] = 0;
   out_2064577180794065928[40] = 1.0;
   out_2064577180794065928[41] = 0;
   out_2064577180794065928[42] = 0;
   out_2064577180794065928[43] = 0;
   out_2064577180794065928[44] = 0;
   out_2064577180794065928[45] = 0;
   out_2064577180794065928[46] = 0;
   out_2064577180794065928[47] = 0;
   out_2064577180794065928[48] = 0;
   out_2064577180794065928[49] = 0;
   out_2064577180794065928[50] = 1.0;
   out_2064577180794065928[51] = 0;
   out_2064577180794065928[52] = 0;
   out_2064577180794065928[53] = 0;
   out_2064577180794065928[54] = 0;
   out_2064577180794065928[55] = 0;
   out_2064577180794065928[56] = 0;
   out_2064577180794065928[57] = 0;
   out_2064577180794065928[58] = 0;
   out_2064577180794065928[59] = 0;
   out_2064577180794065928[60] = 1.0;
   out_2064577180794065928[61] = 0;
   out_2064577180794065928[62] = 0;
   out_2064577180794065928[63] = 0;
   out_2064577180794065928[64] = 0;
   out_2064577180794065928[65] = 0;
   out_2064577180794065928[66] = 0;
   out_2064577180794065928[67] = 0;
   out_2064577180794065928[68] = 0;
   out_2064577180794065928[69] = 0;
   out_2064577180794065928[70] = 1.0;
   out_2064577180794065928[71] = 0;
   out_2064577180794065928[72] = 0;
   out_2064577180794065928[73] = 0;
   out_2064577180794065928[74] = 0;
   out_2064577180794065928[75] = 0;
   out_2064577180794065928[76] = 0;
   out_2064577180794065928[77] = 0;
   out_2064577180794065928[78] = 0;
   out_2064577180794065928[79] = 0;
   out_2064577180794065928[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6429104743904085530) {
   out_6429104743904085530[0] = state[0];
   out_6429104743904085530[1] = state[1];
   out_6429104743904085530[2] = state[2];
   out_6429104743904085530[3] = state[3];
   out_6429104743904085530[4] = state[4];
   out_6429104743904085530[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6429104743904085530[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6429104743904085530[7] = state[7];
   out_6429104743904085530[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8895463431350518309) {
   out_8895463431350518309[0] = 1;
   out_8895463431350518309[1] = 0;
   out_8895463431350518309[2] = 0;
   out_8895463431350518309[3] = 0;
   out_8895463431350518309[4] = 0;
   out_8895463431350518309[5] = 0;
   out_8895463431350518309[6] = 0;
   out_8895463431350518309[7] = 0;
   out_8895463431350518309[8] = 0;
   out_8895463431350518309[9] = 0;
   out_8895463431350518309[10] = 1;
   out_8895463431350518309[11] = 0;
   out_8895463431350518309[12] = 0;
   out_8895463431350518309[13] = 0;
   out_8895463431350518309[14] = 0;
   out_8895463431350518309[15] = 0;
   out_8895463431350518309[16] = 0;
   out_8895463431350518309[17] = 0;
   out_8895463431350518309[18] = 0;
   out_8895463431350518309[19] = 0;
   out_8895463431350518309[20] = 1;
   out_8895463431350518309[21] = 0;
   out_8895463431350518309[22] = 0;
   out_8895463431350518309[23] = 0;
   out_8895463431350518309[24] = 0;
   out_8895463431350518309[25] = 0;
   out_8895463431350518309[26] = 0;
   out_8895463431350518309[27] = 0;
   out_8895463431350518309[28] = 0;
   out_8895463431350518309[29] = 0;
   out_8895463431350518309[30] = 1;
   out_8895463431350518309[31] = 0;
   out_8895463431350518309[32] = 0;
   out_8895463431350518309[33] = 0;
   out_8895463431350518309[34] = 0;
   out_8895463431350518309[35] = 0;
   out_8895463431350518309[36] = 0;
   out_8895463431350518309[37] = 0;
   out_8895463431350518309[38] = 0;
   out_8895463431350518309[39] = 0;
   out_8895463431350518309[40] = 1;
   out_8895463431350518309[41] = 0;
   out_8895463431350518309[42] = 0;
   out_8895463431350518309[43] = 0;
   out_8895463431350518309[44] = 0;
   out_8895463431350518309[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8895463431350518309[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8895463431350518309[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8895463431350518309[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8895463431350518309[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8895463431350518309[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8895463431350518309[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8895463431350518309[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8895463431350518309[53] = -9.8000000000000007*dt;
   out_8895463431350518309[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8895463431350518309[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8895463431350518309[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8895463431350518309[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8895463431350518309[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8895463431350518309[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8895463431350518309[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8895463431350518309[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8895463431350518309[62] = 0;
   out_8895463431350518309[63] = 0;
   out_8895463431350518309[64] = 0;
   out_8895463431350518309[65] = 0;
   out_8895463431350518309[66] = 0;
   out_8895463431350518309[67] = 0;
   out_8895463431350518309[68] = 0;
   out_8895463431350518309[69] = 0;
   out_8895463431350518309[70] = 1;
   out_8895463431350518309[71] = 0;
   out_8895463431350518309[72] = 0;
   out_8895463431350518309[73] = 0;
   out_8895463431350518309[74] = 0;
   out_8895463431350518309[75] = 0;
   out_8895463431350518309[76] = 0;
   out_8895463431350518309[77] = 0;
   out_8895463431350518309[78] = 0;
   out_8895463431350518309[79] = 0;
   out_8895463431350518309[80] = 1;
}
void h_25(double *state, double *unused, double *out_3471221924162000290) {
   out_3471221924162000290[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8244121788343254413) {
   out_8244121788343254413[0] = 0;
   out_8244121788343254413[1] = 0;
   out_8244121788343254413[2] = 0;
   out_8244121788343254413[3] = 0;
   out_8244121788343254413[4] = 0;
   out_8244121788343254413[5] = 0;
   out_8244121788343254413[6] = 1;
   out_8244121788343254413[7] = 0;
   out_8244121788343254413[8] = 0;
}
void h_24(double *state, double *unused, double *out_3576466136437052064) {
   out_3576466136437052064[0] = state[4];
   out_3576466136437052064[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4686196189540931277) {
   out_4686196189540931277[0] = 0;
   out_4686196189540931277[1] = 0;
   out_4686196189540931277[2] = 0;
   out_4686196189540931277[3] = 0;
   out_4686196189540931277[4] = 1;
   out_4686196189540931277[5] = 0;
   out_4686196189540931277[6] = 0;
   out_4686196189540931277[7] = 0;
   out_4686196189540931277[8] = 0;
   out_4686196189540931277[9] = 0;
   out_4686196189540931277[10] = 0;
   out_4686196189540931277[11] = 0;
   out_4686196189540931277[12] = 0;
   out_4686196189540931277[13] = 0;
   out_4686196189540931277[14] = 1;
   out_4686196189540931277[15] = 0;
   out_4686196189540931277[16] = 0;
   out_4686196189540931277[17] = 0;
}
void h_30(double *state, double *unused, double *out_1955846280475620045) {
   out_1955846280475620045[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5725788829836005786) {
   out_5725788829836005786[0] = 0;
   out_5725788829836005786[1] = 0;
   out_5725788829836005786[2] = 0;
   out_5725788829836005786[3] = 0;
   out_5725788829836005786[4] = 1;
   out_5725788829836005786[5] = 0;
   out_5725788829836005786[6] = 0;
   out_5725788829836005786[7] = 0;
   out_5725788829836005786[8] = 0;
}
void h_26(double *state, double *unused, double *out_3045826987185225600) {
   out_3045826987185225600[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6461118966492240979) {
   out_6461118966492240979[0] = 0;
   out_6461118966492240979[1] = 0;
   out_6461118966492240979[2] = 0;
   out_6461118966492240979[3] = 0;
   out_6461118966492240979[4] = 0;
   out_6461118966492240979[5] = 0;
   out_6461118966492240979[6] = 0;
   out_6461118966492240979[7] = 1;
   out_6461118966492240979[8] = 0;
}
void h_27(double *state, double *unused, double *out_8472895191822845935) {
   out_8472895191822845935[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3502194758652062569) {
   out_3502194758652062569[0] = 0;
   out_3502194758652062569[1] = 0;
   out_3502194758652062569[2] = 0;
   out_3502194758652062569[3] = 1;
   out_3502194758652062569[4] = 0;
   out_3502194758652062569[5] = 0;
   out_3502194758652062569[6] = 0;
   out_3502194758652062569[7] = 0;
   out_3502194758652062569[8] = 0;
}
void h_29(double *state, double *unused, double *out_7026122891531220720) {
   out_7026122891531220720[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5215557485521613602) {
   out_5215557485521613602[0] = 0;
   out_5215557485521613602[1] = 1;
   out_5215557485521613602[2] = 0;
   out_5215557485521613602[3] = 0;
   out_5215557485521613602[4] = 0;
   out_5215557485521613602[5] = 0;
   out_5215557485521613602[6] = 0;
   out_5215557485521613602[7] = 0;
   out_5215557485521613602[8] = 0;
}
void h_28(double *state, double *unused, double *out_4972175614225860036) {
   out_4972175614225860036[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8148787571118407440) {
   out_8148787571118407440[0] = 1;
   out_8148787571118407440[1] = 0;
   out_8148787571118407440[2] = 0;
   out_8148787571118407440[3] = 0;
   out_8148787571118407440[4] = 0;
   out_8148787571118407440[5] = 0;
   out_8148787571118407440[6] = 0;
   out_8148787571118407440[7] = 0;
   out_8148787571118407440[8] = 0;
}
void h_31(double *state, double *unused, double *out_2813899509742510757) {
   out_2813899509742510757[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5834910864258889503) {
   out_5834910864258889503[0] = 0;
   out_5834910864258889503[1] = 0;
   out_5834910864258889503[2] = 0;
   out_5834910864258889503[3] = 0;
   out_5834910864258889503[4] = 0;
   out_5834910864258889503[5] = 0;
   out_5834910864258889503[6] = 0;
   out_5834910864258889503[7] = 0;
   out_5834910864258889503[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_2990059360614886699) {
  err_fun(nom_x, delta_x, out_2990059360614886699);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_316514450138690931) {
  inv_err_fun(nom_x, true_x, out_316514450138690931);
}
void car_H_mod_fun(double *state, double *out_2064577180794065928) {
  H_mod_fun(state, out_2064577180794065928);
}
void car_f_fun(double *state, double dt, double *out_6429104743904085530) {
  f_fun(state,  dt, out_6429104743904085530);
}
void car_F_fun(double *state, double dt, double *out_8895463431350518309) {
  F_fun(state,  dt, out_8895463431350518309);
}
void car_h_25(double *state, double *unused, double *out_3471221924162000290) {
  h_25(state, unused, out_3471221924162000290);
}
void car_H_25(double *state, double *unused, double *out_8244121788343254413) {
  H_25(state, unused, out_8244121788343254413);
}
void car_h_24(double *state, double *unused, double *out_3576466136437052064) {
  h_24(state, unused, out_3576466136437052064);
}
void car_H_24(double *state, double *unused, double *out_4686196189540931277) {
  H_24(state, unused, out_4686196189540931277);
}
void car_h_30(double *state, double *unused, double *out_1955846280475620045) {
  h_30(state, unused, out_1955846280475620045);
}
void car_H_30(double *state, double *unused, double *out_5725788829836005786) {
  H_30(state, unused, out_5725788829836005786);
}
void car_h_26(double *state, double *unused, double *out_3045826987185225600) {
  h_26(state, unused, out_3045826987185225600);
}
void car_H_26(double *state, double *unused, double *out_6461118966492240979) {
  H_26(state, unused, out_6461118966492240979);
}
void car_h_27(double *state, double *unused, double *out_8472895191822845935) {
  h_27(state, unused, out_8472895191822845935);
}
void car_H_27(double *state, double *unused, double *out_3502194758652062569) {
  H_27(state, unused, out_3502194758652062569);
}
void car_h_29(double *state, double *unused, double *out_7026122891531220720) {
  h_29(state, unused, out_7026122891531220720);
}
void car_H_29(double *state, double *unused, double *out_5215557485521613602) {
  H_29(state, unused, out_5215557485521613602);
}
void car_h_28(double *state, double *unused, double *out_4972175614225860036) {
  h_28(state, unused, out_4972175614225860036);
}
void car_H_28(double *state, double *unused, double *out_8148787571118407440) {
  H_28(state, unused, out_8148787571118407440);
}
void car_h_31(double *state, double *unused, double *out_2813899509742510757) {
  h_31(state, unused, out_2813899509742510757);
}
void car_H_31(double *state, double *unused, double *out_5834910864258889503) {
  H_31(state, unused, out_5834910864258889503);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
