#include "utilities/cacheddistmap.h"


int MapToDataIndex_(int i, int j, int size_x_) {return i + j * size_x_;}
bool operator<(const CellData& a, const CellData& b)
    {
      return a.data[MapToDataIndex_(a.i_, a.j_,a.size_x)] > b.data[MapToDataIndex_(b.i_, b.j_, b.size_y)];

    }


CachedDistanceMap::CachedDistanceMap(double scale, double max_dist,int size_x, int size_y) : 
  distances_(NULL), scale_(scale), max_dist_(max_dist), size_x_(size_x), size_y_(size_y)
{
  cell_radius_ = max_dist / scale;
  distances_ = new double *[cell_radius_+2];
  for(int i=0; i<=cell_radius_+1; i++)
    {
	    distances_[i] = new double[cell_radius_+2];
      for(int j=0; j<=cell_radius_+1; j++)
	    {
	      distances_[i][j] = sqrt(i*i + j*j);
	    }
    }

    ccmap = (double*) malloc(size_x*size_y*sizeof(double));

}

CachedDistanceMap::CachedDistanceMap(MyLayer* occ_layer,double scale, double max_dist,int size_x, int size_y) : 
  distances_(NULL), scale_(scale), max_dist_(max_dist), size_x_(size_x), size_y_(size_y)
{
  cell_radius_ = max_dist / scale;
  distances_ = new double *[cell_radius_+2];
  for(int i=0; i<=cell_radius_+1; i++)
    {
	    distances_[i] = new double[cell_radius_+2];
      for(int j=0; j<=cell_radius_+1; j++)
	    {
	      distances_[i][j] = sqrt(i*i + j*j);
	    }
    }
    ccmap = (double*) malloc(size_x*size_y*sizeof(double));

    map_update_cspace(occ_layer, size_x_, size_y_, scale, max_dist);

}


CachedDistanceMap::~CachedDistanceMap()
{
  if(distances_)
  {
	  for(int i=0; i<=cell_radius_+1; i++)
	    delete[] distances_[i];
	  delete[] distances_;
  }
}




//probably won't need
CachedDistanceMap* CachedDistanceMap::get_distance_map(double scale, double max_dist)
{
  static CachedDistanceMap* cdm = NULL;

  if(!cdm || (cdm->scale_ != scale) || (cdm->max_dist_ != max_dist))
  {
    if(cdm)
      delete cdm;
    cdm = new CachedDistanceMap(scale, max_dist, size_x_, size_y_);
  }

  return cdm;
}


void CachedDistanceMap::enqueue(int i, int j, int src_i, int src_j, std::priority_queue<CellData>& Q,
      unsigned char* marked)
{
  if(marked[MapToDataIndex_(i, j,size_x_)])
    return;

  int di = abs(i - src_i);
  int dj = abs(j - src_j);
  double distance = distances_[di][dj];

  if(distance > cell_radius_)
    return;
  
  ccmap[MapToDataIndex_(i, j,size_x_)] = distance * scale_;

  CellData cell;
  cell.data = ccmap;

  cell.i_ = i;
  cell.j_ = j;
  cell.src_i_ = src_i;
  cell.src_j_ = src_j;
  cell.size_x = size_x_;
  cell.size_y = size_y_;



  Q.push(cell);

  marked[MapToDataIndex_(i, j, size_x_)] = 1;

}

// Update the cspace distance values
void CachedDistanceMap::map_update_cspace(MyLayer* occ_layer,  int size_x, int size_y, double scale, double max_occ_dist)
{

  unsigned char* marked;
  std::priority_queue<CellData> Q;

  marked = new unsigned char[size_x*size_y];
  memset(marked, 0, sizeof(unsigned char) * size_x*size_y);




  // Enqueue all the obstacle cells
  
  CellData cell;
  cell.data  = ccmap;
  cell.size_x = size_x_;
  cell.size_y = size_y_;
  

//Occupancy state (-1 = free, 0 = unknown, +1 = occ)
//-1 = unknown, +100 = occ, 0 = free


  for(int i=0; i<size_x; i++)
  {
    cell.src_i_ = cell.i_ = i;
    for(int j=0; j<size_y; j++)
    {
      if(occ_layer->get_value(MapToDataIndex_(i, j,size_x_))== +100)

      {

        ccmap[MapToDataIndex_(i, j,size_x_)] = 0.0;
	      cell.src_j_ = cell.j_ = j;
        marked[MapToDataIndex_(i, j,size_x_)] = 1;
	      Q.push(cell);
      }
      else      
      ccmap[MapToDataIndex_(i, j,size_x_)] = max_occ_dist;
    }
  }
  
  while(!Q.empty())
  {
    CellData current_cell = Q.top();
    if(current_cell.i_ > 0)
      enqueue(current_cell.i_-1, current_cell.j_, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q,marked);
    if(current_cell.j_ > 0)
      enqueue(current_cell.i_, current_cell.j_-1, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q,marked);
    if((int)current_cell.i_ < size_x - 1)
      enqueue(current_cell.i_+1, current_cell.j_, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q,marked);
    if((int)current_cell.j_ < size_y - 1)
      enqueue(current_cell.i_, current_cell.j_+1, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q,marked);

    Q.pop();
  }

  delete[] marked;

  

}

/*
Goal: create data layer for cspace distance values
Input: map object OR just meta info
Output: signed char * data
*/