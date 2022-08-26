/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Useful pdf functions
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf_pdf.c 6348 2008-04-17 02:53:17Z gerkey $
 *************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
// #include <gsl/gsl_rng.h>
// #include <gsl/gsl_randist.h>

#include "nav2_amcl/pf/pf_pdf.hpp"

#include "nav2_amcl/portable_utils.hpp"

// Random number generator seed value
static unsigned int pf_pdf_seed;


/**************************************************************************
 * Gaussian
 *************************************************************************/

// Create a gaussian pdf
pf_pdf_gaussian_t * pf_pdf_gaussian_alloc(pf_vector_t x, pf_matrix_t cx)
{
  pf_matrix_t cd;
  pf_pdf_gaussian_t * pdf;

  pdf = calloc(1, sizeof(pf_pdf_gaussian_t));

  pdf->x = x;
  pdf->cx = cx;
  // pdf->cxi = pf_matrix_inverse(cx, &pdf->cxdet);

  // Decompose the convariance matrix into a rotation
  // matrix and a diagonal matrix.
  pf_matrix_unitary(&pdf->cr, &cd, pdf->cx);
  pdf->cd.v[0] = sqrt(cd.m[0][0]);
  pdf->cd.v[1] = sqrt(cd.m[1][1]);
  pdf->cd.v[2] = sqrt(cd.m[2][2]);

  // Initialize the random number generator
  // pdf->rng = gsl_rng_alloc(gsl_rng_taus);
  // gsl_rng_set(pdf->rng, ++pf_pdf_seed);
  srand48(++pf_pdf_seed);

  return pdf;
}


// Destroy the pdf
void pf_pdf_gaussian_free(pf_pdf_gaussian_t * pdf)
{
  // gsl_rng_free(pdf->rng);
  free(pdf);
}


/*
// Compute the value of the pdf at some point [x].
double pf_pdf_gaussian_value(pf_pdf_gaussian_t *pdf, pf_vector_t x)
{
  int i, j;
  pf_vector_t z;
  double zz, p;

  z = pf_vector_sub(x, pdf->x);

  zz = 0;
  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      zz += z.v[i] * pdf->cxi.m[i][j] * z.v[j];

  p =  1 / (2 * M_PI * pdf->cxdet) * exp(-zz / 2);

  return p;
}
*/


// Generate a sample from the pdf.
pf_vector_t pf_pdf_gaussian_sample(pf_pdf_gaussian_t * pdf)
{
  int i, j;
  pf_vector_t r;
  pf_vector_t x;

  // Generate a random vector
  for (i = 0; i < 3; i++) {
    // r.v[i] = gsl_ran_gaussian(pdf->rng, pdf->cd.v[i]);
    r.v[i] = pf_ran_gaussian(pdf->cd.v[i]);
  }

  for (i = 0; i < 3; i++) {
    x.v[i] = pdf->x.v[i];
    for (j = 0; j < 3; j++) {
      x.v[i] += pdf->cr.m[i][j] * r.v[j];
    }
  }

  return x;
}

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
double pf_ran_gaussian(double sigma)
{
  double x1, x2, w, r;

  do {
    do {
      r = drand48();
    } while (r == 0.0);
    x1 = 2.0 * r - 1.0;
    do {
      r = drand48();
    } while (r == 0.0);
    x2 = 2.0 * r - 1.0;
    w = x1 * x1 + x2 * x2;
  } while (w > 1.0 || w == 0.0);

  return sigma * x2 * sqrt(-2.0 * log(w) / w);
}
