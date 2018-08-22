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

#ifndef UTILITIES_CACHED_DIST_MAP_H
#define UTILITIES_CACHED_DIST_MAP_H


#include <queue>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "occ_grid_layer.h"


class CellData
{
  public:

    double* data;
    unsigned int i_, j_;
    unsigned int src_i_, src_j_;
    int size_x;
    int size_y;
};

class CachedDistanceMap
{
  public:
    CachedDistanceMap(double scale, double max_dist, int size_x, int size_y);
    CachedDistanceMap(MyLayer* occ_layer, double scale, double max_dist, int size_x, int size_y);

    ~CachedDistanceMap();

    double** distances_;
    double scale_;
    double max_dist_;
    int cell_radius_;
    int size_x_;
    int size_y_;
    double* ccmap;

    void enqueue(int i, int j,
	     int src_i, int src_j,
	     std::priority_queue<CellData>& Q,
	     unsigned char* marked);
    

    CachedDistanceMap* get_distance_map(double scale, double max_dist);   

    // Update the cspace distance values
    void map_update_cspace(MyLayer* occ_layer, int size_x, int size_y, double scale, double max_occ_dist);     
 
};

#endif