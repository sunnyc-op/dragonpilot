#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_9063551671781569783) {
   out_9063551671781569783[0] = delta_x[0] + nom_x[0];
   out_9063551671781569783[1] = delta_x[1] + nom_x[1];
   out_9063551671781569783[2] = delta_x[2] + nom_x[2];
   out_9063551671781569783[3] = delta_x[3] + nom_x[3];
   out_9063551671781569783[4] = delta_x[4] + nom_x[4];
   out_9063551671781569783[5] = delta_x[5] + nom_x[5];
   out_9063551671781569783[6] = delta_x[6] + nom_x[6];
   out_9063551671781569783[7] = delta_x[7] + nom_x[7];
   out_9063551671781569783[8] = delta_x[8] + nom_x[8];
   out_9063551671781569783[9] = delta_x[9] + nom_x[9];
   out_9063551671781569783[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3184610972210744396) {
   out_3184610972210744396[0] = -nom_x[0] + true_x[0];
   out_3184610972210744396[1] = -nom_x[1] + true_x[1];
   out_3184610972210744396[2] = -nom_x[2] + true_x[2];
   out_3184610972210744396[3] = -nom_x[3] + true_x[3];
   out_3184610972210744396[4] = -nom_x[4] + true_x[4];
   out_3184610972210744396[5] = -nom_x[5] + true_x[5];
   out_3184610972210744396[6] = -nom_x[6] + true_x[6];
   out_3184610972210744396[7] = -nom_x[7] + true_x[7];
   out_3184610972210744396[8] = -nom_x[8] + true_x[8];
   out_3184610972210744396[9] = -nom_x[9] + true_x[9];
   out_3184610972210744396[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_5174772375165381252) {
   out_5174772375165381252[0] = 1.0;
   out_5174772375165381252[1] = 0;
   out_5174772375165381252[2] = 0;
   out_5174772375165381252[3] = 0;
   out_5174772375165381252[4] = 0;
   out_5174772375165381252[5] = 0;
   out_5174772375165381252[6] = 0;
   out_5174772375165381252[7] = 0;
   out_5174772375165381252[8] = 0;
   out_5174772375165381252[9] = 0;
   out_5174772375165381252[10] = 0;
   out_5174772375165381252[11] = 0;
   out_5174772375165381252[12] = 1.0;
   out_5174772375165381252[13] = 0;
   out_5174772375165381252[14] = 0;
   out_5174772375165381252[15] = 0;
   out_5174772375165381252[16] = 0;
   out_5174772375165381252[17] = 0;
   out_5174772375165381252[18] = 0;
   out_5174772375165381252[19] = 0;
   out_5174772375165381252[20] = 0;
   out_5174772375165381252[21] = 0;
   out_5174772375165381252[22] = 0;
   out_5174772375165381252[23] = 0;
   out_5174772375165381252[24] = 1.0;
   out_5174772375165381252[25] = 0;
   out_5174772375165381252[26] = 0;
   out_5174772375165381252[27] = 0;
   out_5174772375165381252[28] = 0;
   out_5174772375165381252[29] = 0;
   out_5174772375165381252[30] = 0;
   out_5174772375165381252[31] = 0;
   out_5174772375165381252[32] = 0;
   out_5174772375165381252[33] = 0;
   out_5174772375165381252[34] = 0;
   out_5174772375165381252[35] = 0;
   out_5174772375165381252[36] = 1.0;
   out_5174772375165381252[37] = 0;
   out_5174772375165381252[38] = 0;
   out_5174772375165381252[39] = 0;
   out_5174772375165381252[40] = 0;
   out_5174772375165381252[41] = 0;
   out_5174772375165381252[42] = 0;
   out_5174772375165381252[43] = 0;
   out_5174772375165381252[44] = 0;
   out_5174772375165381252[45] = 0;
   out_5174772375165381252[46] = 0;
   out_5174772375165381252[47] = 0;
   out_5174772375165381252[48] = 1.0;
   out_5174772375165381252[49] = 0;
   out_5174772375165381252[50] = 0;
   out_5174772375165381252[51] = 0;
   out_5174772375165381252[52] = 0;
   out_5174772375165381252[53] = 0;
   out_5174772375165381252[54] = 0;
   out_5174772375165381252[55] = 0;
   out_5174772375165381252[56] = 0;
   out_5174772375165381252[57] = 0;
   out_5174772375165381252[58] = 0;
   out_5174772375165381252[59] = 0;
   out_5174772375165381252[60] = 1.0;
   out_5174772375165381252[61] = 0;
   out_5174772375165381252[62] = 0;
   out_5174772375165381252[63] = 0;
   out_5174772375165381252[64] = 0;
   out_5174772375165381252[65] = 0;
   out_5174772375165381252[66] = 0;
   out_5174772375165381252[67] = 0;
   out_5174772375165381252[68] = 0;
   out_5174772375165381252[69] = 0;
   out_5174772375165381252[70] = 0;
   out_5174772375165381252[71] = 0;
   out_5174772375165381252[72] = 1.0;
   out_5174772375165381252[73] = 0;
   out_5174772375165381252[74] = 0;
   out_5174772375165381252[75] = 0;
   out_5174772375165381252[76] = 0;
   out_5174772375165381252[77] = 0;
   out_5174772375165381252[78] = 0;
   out_5174772375165381252[79] = 0;
   out_5174772375165381252[80] = 0;
   out_5174772375165381252[81] = 0;
   out_5174772375165381252[82] = 0;
   out_5174772375165381252[83] = 0;
   out_5174772375165381252[84] = 1.0;
   out_5174772375165381252[85] = 0;
   out_5174772375165381252[86] = 0;
   out_5174772375165381252[87] = 0;
   out_5174772375165381252[88] = 0;
   out_5174772375165381252[89] = 0;
   out_5174772375165381252[90] = 0;
   out_5174772375165381252[91] = 0;
   out_5174772375165381252[92] = 0;
   out_5174772375165381252[93] = 0;
   out_5174772375165381252[94] = 0;
   out_5174772375165381252[95] = 0;
   out_5174772375165381252[96] = 1.0;
   out_5174772375165381252[97] = 0;
   out_5174772375165381252[98] = 0;
   out_5174772375165381252[99] = 0;
   out_5174772375165381252[100] = 0;
   out_5174772375165381252[101] = 0;
   out_5174772375165381252[102] = 0;
   out_5174772375165381252[103] = 0;
   out_5174772375165381252[104] = 0;
   out_5174772375165381252[105] = 0;
   out_5174772375165381252[106] = 0;
   out_5174772375165381252[107] = 0;
   out_5174772375165381252[108] = 1.0;
   out_5174772375165381252[109] = 0;
   out_5174772375165381252[110] = 0;
   out_5174772375165381252[111] = 0;
   out_5174772375165381252[112] = 0;
   out_5174772375165381252[113] = 0;
   out_5174772375165381252[114] = 0;
   out_5174772375165381252[115] = 0;
   out_5174772375165381252[116] = 0;
   out_5174772375165381252[117] = 0;
   out_5174772375165381252[118] = 0;
   out_5174772375165381252[119] = 0;
   out_5174772375165381252[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_3910409860434011468) {
   out_3910409860434011468[0] = dt*state[3] + state[0];
   out_3910409860434011468[1] = dt*state[4] + state[1];
   out_3910409860434011468[2] = dt*state[5] + state[2];
   out_3910409860434011468[3] = state[3];
   out_3910409860434011468[4] = state[4];
   out_3910409860434011468[5] = state[5];
   out_3910409860434011468[6] = dt*state[7] + state[6];
   out_3910409860434011468[7] = dt*state[8] + state[7];
   out_3910409860434011468[8] = state[8];
   out_3910409860434011468[9] = state[9];
   out_3910409860434011468[10] = state[10];
}
void F_fun(double *state, double dt, double *out_3815441773285650391) {
   out_3815441773285650391[0] = 1;
   out_3815441773285650391[1] = 0;
   out_3815441773285650391[2] = 0;
   out_3815441773285650391[3] = dt;
   out_3815441773285650391[4] = 0;
   out_3815441773285650391[5] = 0;
   out_3815441773285650391[6] = 0;
   out_3815441773285650391[7] = 0;
   out_3815441773285650391[8] = 0;
   out_3815441773285650391[9] = 0;
   out_3815441773285650391[10] = 0;
   out_3815441773285650391[11] = 0;
   out_3815441773285650391[12] = 1;
   out_3815441773285650391[13] = 0;
   out_3815441773285650391[14] = 0;
   out_3815441773285650391[15] = dt;
   out_3815441773285650391[16] = 0;
   out_3815441773285650391[17] = 0;
   out_3815441773285650391[18] = 0;
   out_3815441773285650391[19] = 0;
   out_3815441773285650391[20] = 0;
   out_3815441773285650391[21] = 0;
   out_3815441773285650391[22] = 0;
   out_3815441773285650391[23] = 0;
   out_3815441773285650391[24] = 1;
   out_3815441773285650391[25] = 0;
   out_3815441773285650391[26] = 0;
   out_3815441773285650391[27] = dt;
   out_3815441773285650391[28] = 0;
   out_3815441773285650391[29] = 0;
   out_3815441773285650391[30] = 0;
   out_3815441773285650391[31] = 0;
   out_3815441773285650391[32] = 0;
   out_3815441773285650391[33] = 0;
   out_3815441773285650391[34] = 0;
   out_3815441773285650391[35] = 0;
   out_3815441773285650391[36] = 1;
   out_3815441773285650391[37] = 0;
   out_3815441773285650391[38] = 0;
   out_3815441773285650391[39] = 0;
   out_3815441773285650391[40] = 0;
   out_3815441773285650391[41] = 0;
   out_3815441773285650391[42] = 0;
   out_3815441773285650391[43] = 0;
   out_3815441773285650391[44] = 0;
   out_3815441773285650391[45] = 0;
   out_3815441773285650391[46] = 0;
   out_3815441773285650391[47] = 0;
   out_3815441773285650391[48] = 1;
   out_3815441773285650391[49] = 0;
   out_3815441773285650391[50] = 0;
   out_3815441773285650391[51] = 0;
   out_3815441773285650391[52] = 0;
   out_3815441773285650391[53] = 0;
   out_3815441773285650391[54] = 0;
   out_3815441773285650391[55] = 0;
   out_3815441773285650391[56] = 0;
   out_3815441773285650391[57] = 0;
   out_3815441773285650391[58] = 0;
   out_3815441773285650391[59] = 0;
   out_3815441773285650391[60] = 1;
   out_3815441773285650391[61] = 0;
   out_3815441773285650391[62] = 0;
   out_3815441773285650391[63] = 0;
   out_3815441773285650391[64] = 0;
   out_3815441773285650391[65] = 0;
   out_3815441773285650391[66] = 0;
   out_3815441773285650391[67] = 0;
   out_3815441773285650391[68] = 0;
   out_3815441773285650391[69] = 0;
   out_3815441773285650391[70] = 0;
   out_3815441773285650391[71] = 0;
   out_3815441773285650391[72] = 1;
   out_3815441773285650391[73] = dt;
   out_3815441773285650391[74] = 0;
   out_3815441773285650391[75] = 0;
   out_3815441773285650391[76] = 0;
   out_3815441773285650391[77] = 0;
   out_3815441773285650391[78] = 0;
   out_3815441773285650391[79] = 0;
   out_3815441773285650391[80] = 0;
   out_3815441773285650391[81] = 0;
   out_3815441773285650391[82] = 0;
   out_3815441773285650391[83] = 0;
   out_3815441773285650391[84] = 1;
   out_3815441773285650391[85] = dt;
   out_3815441773285650391[86] = 0;
   out_3815441773285650391[87] = 0;
   out_3815441773285650391[88] = 0;
   out_3815441773285650391[89] = 0;
   out_3815441773285650391[90] = 0;
   out_3815441773285650391[91] = 0;
   out_3815441773285650391[92] = 0;
   out_3815441773285650391[93] = 0;
   out_3815441773285650391[94] = 0;
   out_3815441773285650391[95] = 0;
   out_3815441773285650391[96] = 1;
   out_3815441773285650391[97] = 0;
   out_3815441773285650391[98] = 0;
   out_3815441773285650391[99] = 0;
   out_3815441773285650391[100] = 0;
   out_3815441773285650391[101] = 0;
   out_3815441773285650391[102] = 0;
   out_3815441773285650391[103] = 0;
   out_3815441773285650391[104] = 0;
   out_3815441773285650391[105] = 0;
   out_3815441773285650391[106] = 0;
   out_3815441773285650391[107] = 0;
   out_3815441773285650391[108] = 1;
   out_3815441773285650391[109] = 0;
   out_3815441773285650391[110] = 0;
   out_3815441773285650391[111] = 0;
   out_3815441773285650391[112] = 0;
   out_3815441773285650391[113] = 0;
   out_3815441773285650391[114] = 0;
   out_3815441773285650391[115] = 0;
   out_3815441773285650391[116] = 0;
   out_3815441773285650391[117] = 0;
   out_3815441773285650391[118] = 0;
   out_3815441773285650391[119] = 0;
   out_3815441773285650391[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_4891984551865118720) {
   out_4891984551865118720[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_2764292410338766363) {
   out_2764292410338766363[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2764292410338766363[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2764292410338766363[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2764292410338766363[3] = 0;
   out_2764292410338766363[4] = 0;
   out_2764292410338766363[5] = 0;
   out_2764292410338766363[6] = 1;
   out_2764292410338766363[7] = 0;
   out_2764292410338766363[8] = 0;
   out_2764292410338766363[9] = 0;
   out_2764292410338766363[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_1944521021824495821) {
   out_1944521021824495821[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_7355405256728486282) {
   out_7355405256728486282[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7355405256728486282[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7355405256728486282[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7355405256728486282[3] = 0;
   out_7355405256728486282[4] = 0;
   out_7355405256728486282[5] = 0;
   out_7355405256728486282[6] = 1;
   out_7355405256728486282[7] = 0;
   out_7355405256728486282[8] = 0;
   out_7355405256728486282[9] = 1;
   out_7355405256728486282[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_3020125195924369443) {
   out_3020125195924369443[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_8334839139454016519) {
   out_8334839139454016519[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8334839139454016519[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8334839139454016519[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8334839139454016519[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8334839139454016519[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8334839139454016519[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8334839139454016519[6] = 0;
   out_8334839139454016519[7] = 1;
   out_8334839139454016519[8] = 0;
   out_8334839139454016519[9] = 0;
   out_8334839139454016519[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_3020125195924369443) {
   out_3020125195924369443[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_8334839139454016519) {
   out_8334839139454016519[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8334839139454016519[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8334839139454016519[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8334839139454016519[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8334839139454016519[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8334839139454016519[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8334839139454016519[6] = 0;
   out_8334839139454016519[7] = 1;
   out_8334839139454016519[8] = 0;
   out_8334839139454016519[9] = 0;
   out_8334839139454016519[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9063551671781569783) {
  err_fun(nom_x, delta_x, out_9063551671781569783);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3184610972210744396) {
  inv_err_fun(nom_x, true_x, out_3184610972210744396);
}
void gnss_H_mod_fun(double *state, double *out_5174772375165381252) {
  H_mod_fun(state, out_5174772375165381252);
}
void gnss_f_fun(double *state, double dt, double *out_3910409860434011468) {
  f_fun(state,  dt, out_3910409860434011468);
}
void gnss_F_fun(double *state, double dt, double *out_3815441773285650391) {
  F_fun(state,  dt, out_3815441773285650391);
}
void gnss_h_6(double *state, double *sat_pos, double *out_4891984551865118720) {
  h_6(state, sat_pos, out_4891984551865118720);
}
void gnss_H_6(double *state, double *sat_pos, double *out_2764292410338766363) {
  H_6(state, sat_pos, out_2764292410338766363);
}
void gnss_h_20(double *state, double *sat_pos, double *out_1944521021824495821) {
  h_20(state, sat_pos, out_1944521021824495821);
}
void gnss_H_20(double *state, double *sat_pos, double *out_7355405256728486282) {
  H_20(state, sat_pos, out_7355405256728486282);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3020125195924369443) {
  h_7(state, sat_pos_vel, out_3020125195924369443);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8334839139454016519) {
  H_7(state, sat_pos_vel, out_8334839139454016519);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3020125195924369443) {
  h_21(state, sat_pos_vel, out_3020125195924369443);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8334839139454016519) {
  H_21(state, sat_pos_vel, out_8334839139454016519);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
