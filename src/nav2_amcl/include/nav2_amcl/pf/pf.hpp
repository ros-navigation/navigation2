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
 * Desc: Simple particle filter for localization.
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf.h 3293 2005-11-19 08:37:45Z gerkey $
 *************************************************************************/

#ifndef NAV2_AMCL__PF__PF_HPP_
#define NAV2_AMCL__PF__PF_HPP_

#include "nav2_amcl/pf/pf_vector.hpp"
#include "nav2_amcl/pf/pf_kdtree.hpp"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
struct _pf_t;
struct _rtk_fig_t;
struct _pf_sample_set_t;

// Function prototype for the initialization model; generates a sample pose from
// an appropriate distribution.
typedef pf_vector_t (* pf_init_model_fn_t) (void * init_data);

// Function prototype for the action model; generates a sample pose from
// an appropriate distribution
typedef void (* pf_action_model_fn_t) (
  void * action_data,
  struct _pf_sample_set_t * set);

// Function prototype for the sensor model; determines the probability
// for the given set of sample poses.
typedef double (* pf_sensor_model_fn_t) (
  void * sensor_data,
  struct _pf_sample_set_t * set);


// Information for a single sample
typedef struct
{
  // Pose represented by this sample
  pf_vector_t pose;

  // Weight for this pose
  double weight;
} pf_sample_t;


// Information for a cluster of samples
typedef struct
{
  // Number of samples
  int count;

  // Total weight of samples in this cluster
  double weight;

  // Cluster statistics
  pf_vector_t mean;
  pf_matrix_t cov;

  // Workspace
  double m[4], c[2][2];
} pf_cluster_t;


// Information for a set of samples
typedef struct _pf_sample_set_t
{
  // The samples
  int sample_count;
  pf_sample_t * samples;

  // A kdtree encoding the histogram
  pf_kdtree_t * kdtree;

  // Clusters
  int cluster_count, cluster_max_count;
  pf_cluster_t * clusters;

  // Filter statistics
  pf_vector_t mean;
  pf_matrix_t cov;
  int converged;
} pf_sample_set_t;


// Information for an entire filter
typedef struct _pf_t
{
  // This min and max number of samples
  int min_samples, max_samples;

  // Population size parameters
  double pop_err, pop_z;

  // The sample sets.  We keep two sets and use [current_set]
  // to identify the active set.
  int current_set;
  pf_sample_set_t sets[2];

  // Running averages, slow and fast, of likelihood
  double w_slow, w_fast;

  // Decay rates for running averages
  double alpha_slow, alpha_fast;

  // Function used to draw random pose samples
  pf_init_model_fn_t random_pose_fn;

  double dist_threshold;  // distance threshold in each axis over which the pf is considered to not
                          // be converged
  int converged;
} pf_t;


// Create a new filter
pf_t * pf_alloc(
  int min_samples, int max_samples,
  double alpha_slow, double alpha_fast,
  pf_init_model_fn_t random_pose_fn);

// Free an existing filter
void pf_free(pf_t * pf);

// Initialize the filter using a guassian
void pf_init(pf_t * pf, pf_vector_t mean, pf_matrix_t cov);

// Initialize the filter using some model
void pf_init_model(pf_t * pf, pf_init_model_fn_t init_fn, void * init_data);

// Update the filter with some new action
// void pf_update_action(pf_t * pf, pf_action_model_fn_t action_fn, void * action_data);

// Update the filter with some new sensor observation
void pf_update_sensor(pf_t * pf, pf_sensor_model_fn_t sensor_fn, void * sensor_data);

// Resample the distribution
void pf_update_resample(pf_t * pf, void * random_pose_data);

// Compute the CEP statistics (mean and variance).
// void pf_get_cep_stats(pf_t * pf, pf_vector_t * mean, double * var);

// Compute the statistics for a particular cluster.  Returns 0 if
// there is no such cluster.
int pf_get_cluster_stats(
  pf_t * pf, int cluster, double * weight,
  pf_vector_t * mean, pf_matrix_t * cov);

// Re-compute the cluster statistics for a sample set
void pf_cluster_stats(pf_t * pf, pf_sample_set_t * set);


// Display the sample set
void pf_draw_samples(pf_t * pf, struct _rtk_fig_t * fig, int max_samples);

// Draw the histogram (kdtree)
void pf_draw_hist(pf_t * pf, struct _rtk_fig_t * fig);

// Draw the CEP statistics
// void pf_draw_cep_stats(pf_t * pf, struct _rtk_fig_t * fig);

// Draw the cluster statistics
void pf_draw_cluster_stats(pf_t * pf, struct _rtk_fig_t * fig);

// calculate if the particle filter has converged -
// and sets the converged flag in the current set and the pf
int pf_update_converged(pf_t * pf);

// sets the current set and pf converged values to zero
void pf_init_converged(pf_t * pf);

#ifdef __cplusplus
}
#endif


#endif  // NAV2_AMCL__PF__PF_HPP_
