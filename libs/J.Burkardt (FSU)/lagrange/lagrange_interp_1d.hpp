
#ifndef   _LAGRANGE_INTERP_1D_H_
#define   _LAGRANGE_INTERP_1D_H_

double*  lagrange_basis_1d (int nd, const double xd[],
                            int ni, const double xi[]);
double*  lagrange_value_1d (int nd, const double xd[], const double yd[],
                            int ni, const double xi[]);

#endif // _LAGRANGE_INTERP_1D_H_
