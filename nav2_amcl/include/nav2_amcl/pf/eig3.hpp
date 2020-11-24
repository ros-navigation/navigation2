/* Eigen-decomposition for symmetric 3x3 real matrices.
   Public domain, copied from the public domain Java library JAMA. */

#ifndef NAV2_AMCL__PF__EIG3_HPP_
#define NAV2_AMCL__PF__EIG3_HPP_

/* Symmetric matrix A => eigenvectors in columns of V, corresponding
   eigenvalues in d. */
void eigen_decomposition(double A[3][3], double V[3][3], double d[3]);

#endif  // NAV2_AMCL__PF__EIG3_HPP_
