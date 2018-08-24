#include "map_server/map_server.h"
#include "map_server/map_loader.h"

#include <typeinfo>

#define USAGE "\nUSAGE: map_server <map.yaml> <map_type>\n" \
              "  map.yaml: map description file\n" \
              "  map_type: the type of map to load (i.e. occupancy)\n"


MappingServerROS::MappingServerROS(const std::string& fname, const std::string& map_type)
{
    n = rclcpp::Node::make_shared("map_server");

    try
    {
        m = new MapLoader(); 
        
        RCLCPP_INFO(n->get_logger(),"Loading Map of Type '%s'", map_type.c_str());
        MyMap = m->createMap(map_type, n, fname); 

        rclcpp::spin(n);
    
    }
    catch (std::runtime_error e)
    {
        RCLCPP_ERROR(n->get_logger(),"Cannot load map");
        exit(-1); 
    }
   

} 

int main(int argc, char **argv)
{

  rclcpp::init(argc,argv);

  if(argc != 3 && argc != 2)
  {
    fprintf(stderr,"[ERROR] [map_server]: %s", USAGE);
    exit(-1);
  }

  std::string fname(argv[1]);
  std::string map_type = (argc ==2) ? "occupancy" : std::string(argv[2]);

  try
  {
    MappingServerROS Server(fname, map_type);

  }
  catch(std::runtime_error& e)
  {

    fprintf(stderr,"[ERROR] [map_server]: map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}

