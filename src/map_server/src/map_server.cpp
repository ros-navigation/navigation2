#include "map_server/map_server.h"
#include "map_server/map_loader.h"

#include <typeinfo>

#define USAGE "\nUSAGE: map_server <map.yaml> <map_type>\n" \
              "  map.yaml: map description file\n" \
              "  map_type: the type of map to load (i.e. occupancy)"


MappingServerROS::MappingServerROS(const std::string& fname, const std::string& map_type)
{
    n = rclcpp::Node::make_shared("map_server");

    
    // Load Map from File
    try
    {
        m = new MapLoader(); 

        // Initialize from derived constructor  //
        
        RCLCPP_INFO(n->get_logger(),"Create Map Object");
        MyMap = m->createMap(map_type, n, fname); 

  
        //Possible node initialization:
        //n = rclcpp::Node::make_shared<MyMap>("map_server");

        // Initialize from interface //

/*
        RCLCPP_INFO(n->get_logger(),"Create Map Object");
        MyMap = m->createMap(map_type); 
        RCLCPP_INFO(n->get_logger(),"Load map info");
        MyMap->loadMapInfoFromFile(fname,n);
        std::string temp = MyMap->mapfname;
        RCLCPP_INFO(n->get_logger(),"Load Map: %s", temp.c_str());
        MyMap->loadMapFromFile(MyMap->mapfname);
        RCLCPP_INFO(n->get_logger(),"Set up Service");
        MyMap->connectROS(n); 
        MyMap->setMap();
        RCLCPP_INFO(n->get_logger(),"Set up Publisher");      
        MyMap->publishMap();
        RCLCPP_INFO(n->get_logger(),"Success");
        
*/
        rclcpp::spin(n);
    
    }
    catch (std::runtime_error e)
    {
        RCLCPP_ERROR(n->get_logger(),"Cannot create map");
        exit(-1); 
    }
   

} 

int main(int argc, char **argv)
{

  rclcpp::init(argc,argv);

  if(argc != 3 && argc != 2)
  {
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
    return -1;
  }

  return 0;
}

