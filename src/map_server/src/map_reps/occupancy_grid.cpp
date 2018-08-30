#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <cstring>
#include <stdexcept>
#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"
#include "map_server/map_reps/occupancy_grid.h"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

// We use SDL_image to load the image from disk
#include <SDL/SDL_image.h>

// Use Bullet's Quaternion object to create one from Euler angles
#include <LinearMath/btQuaternion.h>


#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

#ifdef HAVE_YAMLCPP_GT_0_5_0
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.

template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();

}

#endif


// Interface // 

void OccGridLoader::loadMapInfoFromFile(std::string fname)
{   

    std::ifstream fin(fname.c_str());
    if (fin.fail()) {
        fprintf(stderr,"[ERROR] [map_server] Map_server could not open %s\n", fname.c_str());
        exit(-1);
    }


#ifdef HAVE_YAMLCPP_GT_0_5_0
        // The document loading process changed in yaml-cpp 0.5.
        //YAML::Node doc = YAML::Load(fin);
        YAML::Node doc = YAML::LoadFile(fname);
#else
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);
#endif
    try {
        doc["resolution"] >> res;

    } catch (YAML::InvalidScalar) {
        fprintf(stderr,"[ERROR] [map_server]: The map does not contain a resolution tag or it is invalid.\n");

        exit(-1);
    }


    try {
        doc["negate"] >> negate;
    } catch (YAML::InvalidScalar) {
        fprintf(stderr,"[ERROR] [map_server]: The map does not contain a negate tag or it is invalid.\n");

        exit(-1);
    }
    try {
        doc["occupied_thresh"] >> occ_th;
    } catch (YAML::InvalidScalar) {
        fprintf(stderr,"[ERROR] [map_server]: The map does not contain an occupied_thresh tag or it is invalid.\n");

        exit(-1);
    }
    try {
        doc["free_thresh"] >> free_th;
    } catch (YAML::InvalidScalar) {
        fprintf(stderr,"[ERROR] [map_server]: The map does not contain a free_thresh tag or it is invalid.\n");

        exit(-1);
    }
    try {
        std::string modeS = "";
        doc["mode"] >> modeS;

        if(modeS=="trinary")
        mode = TRINARY;
        else if(modeS=="scale")
        mode = SCALE;
        else if(modeS=="raw")
        mode = RAW;
        else{
        fprintf(stderr,"Invalid mode tag \"%s\".\n", modeS.c_str());

        exit(-1);
        }
    } catch (YAML::Exception) {
        //fprintf(stdout,"[DEBUG] [map_server] The map does not contain a mode tag or it is invalid... assuming Trinary\n");
        mode = TRINARY;
    }
    try {
        doc["origin"][0] >> origin[0];
        doc["origin"][1] >> origin[1];
        doc["origin"][2] >> origin[2];
    } catch (YAML::InvalidScalar) {
        fprintf(stderr,"[ERROR] [map_server]: The map does not contain an origin tag or it is invalid.\n");

        exit(-1);
    }
    try {
        doc["image"] >> mapfname;
        // TODO: make this path-handling more robust
        if(mapfname.size() == 0)
        {
        fprintf(stderr,"[ERROR] [map_server]: The image tag cannot be an empty string.\n");

        exit(-1);
        }
        if(mapfname[0] != '/')
        {
        // dirname can modify what you pass it
        char* fname_copy = strdup(fname.c_str());
        mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
        free(fname_copy);
        }
    } catch (YAML::InvalidScalar) {
        fprintf(stderr,"[ERROR] [map_server]: The map does not contain an image tag or it is invalid.\n");

        exit(-1);
    }
    
}

void OccGridLoader::loadMapFromFile(std::string mapfname)
{

  SDL_Surface* img;
  const char* name = mapfname.c_str();
  unsigned char* pixels;
  unsigned char* p;
  unsigned char value;
  int rowstride, n_channels, avg_channels;
  unsigned int i,j;
  int k;
  double occ;
  int alpha;
  int color_sum;
  double color_avg;

  // Load the image using SDL.  If we get NULL back, the image load failed.
  if(!(img = IMG_Load(name)))
  {
    std::string errmsg = std::string("failed to open image file \"") +
            std::string(name) + std::string("\": ") + IMG_GetError();
    fprintf(stderr,"[ERROR] [map_server] %s\n", errmsg.c_str());

    throw std::runtime_error(errmsg);
  }
  // Copy the image data into the map structure
  map_msg_.info.width = img->w;
  map_msg_.info.height = img->h;
  map_msg_.info.resolution = res;
  map_msg_.info.origin.position.x = *(origin);
  map_msg_.info.origin.position.y = *(origin+1);
  map_msg_.info.origin.position.z = 0.0;
  btQuaternion q;
  // setEulerZYX(yaw, pitch, roll)
  q.setEulerZYX(*(origin+2), 0, 0);
  map_msg_.info.origin.orientation.x = q.x();
  map_msg_.info.origin.orientation.y = q.y();
  map_msg_.info.origin.orientation.z = q.z();
  map_msg_.info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  map_msg_.data.resize(map_msg_.info.width * map_msg_.info.height);

  // Get values that we'll need to iterate through the pixels
  rowstride = img->pitch;
  n_channels = img->format->BytesPerPixel;

  // NOTE: Trinary mode still overrides here to preserve existing behavior.
  // Alpha will be averaged in with color channels when using trinary mode.
  if (mode==TRINARY || !img->format->Amask)
    avg_channels = n_channels;
  else
    avg_channels = n_channels - 1;

  // Copy pixel data into the map structure
  pixels = (unsigned char*)(img->pixels);
  for(j = 0; j < map_msg_.info.height; j++)
  {
    for (i = 0; i < map_msg_.info.width; i++)
    {
      // Compute mean of RGB for this pixel
      p = pixels + j*rowstride + i*n_channels;
      color_sum = 0;
      for(k=0;k<avg_channels;k++)
        color_sum += *(p + (k));
      color_avg = color_sum / (double)avg_channels;

      if (n_channels == 1)
          alpha = 1;
      else
          alpha = *(p+n_channels-1);

      if(negate)
        color_avg = 255 - color_avg;

      if(mode==RAW){
          value = color_avg;
          map_msg_.data[MAP_IDX(map_msg_.info.width,i,map_msg_.info.height - j - 1)] = value;
          continue;
      }

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels free.  Otherwise, it's vice versa.
      occ = (255 - color_avg) / 255.0;

      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      if(occ > occ_th)
        value = +100;
      else if(occ < free_th)
        value = 0;
      else if(mode==TRINARY || alpha < 1.0)
        value = -1;
      else {
        double ratio = (occ - free_th) / (occ_th - free_th);
        value = 99 * ratio;
      }

      map_msg_.data[MAP_IDX(map_msg_.info.width,i,map_msg_.info.height - j - 1)] = value;
    }
  }
    
  SDL_FreeSurface(img);


}

void OccGridLoader::connectROS(rclcpp::Node::SharedPtr n)
{
    // Create a publisher

    occ_pub_ = n->create_publisher<nav_msgs::msg::OccupancyGrid>("occmap", rmw_qos_profile_default);
    
    // Create a service callback handle
    auto handle_occ_callback = [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
        std::shared_ptr<nav_msgs::srv::GetMap::Response> response) -> void
    {
        OccMapCallback(request_header, request, response);
    };

    // Create a service 
    occ_service_ = n->create_service<nav_msgs::srv::GetMap>("static_occ_grid",handle_occ_callback);
}

void OccGridLoader::setMap()
{
    occ_resp_.map = map_msg_;
}

void OccGridLoader::publishMap()
{
    occ_pub_->publish(map_msg_);
}

// OccGridLoader specific //

OccGridLoader::OccGridLoader(rclcpp::Node::SharedPtr n, std::string filename)
{
    // Set up //

    RCLCPP_INFO(n->get_logger(),"Load map info");
    loadMapInfoFromFile(filename);

    RCLCPP_INFO(n->get_logger(),"Load Map: %s", mapfname.c_str());
    loadMapFromFile(mapfname);

    connectROS(n);

    RCLCPP_INFO(n->get_logger(),"Set up Service");
    setMap();

    RCLCPP_INFO(n->get_logger(),"Set up Publisher");
    publishMap();

    RCLCPP_INFO(n->get_logger(),"Success!");

}

void OccGridLoader::OccMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
    const std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
{
    (void)request_header;
    (void)req;
    res->map = occ_resp_.map;

}
