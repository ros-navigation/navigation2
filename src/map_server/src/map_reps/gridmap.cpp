#include "map_server/map_reps/gridmap.h"



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

void GridMapLoader::loadMapInfoFromFile(std::string fname)
{
    std::ifstream fin(fname.c_str());
    if (fin.fail()) {
        ROS_ERROR("Map_server could not open %s.", fname.c_str());
        exit(-1);
    }
#ifdef HAVE_YAMLCPP_GT_0_5_0
        // The document loading process changed in yaml-cpp 0.5.
        YAML::Node doc = YAML::Load(fin);
#else
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);
#endif
    try {
        doc["resolution"] >> res;
    } catch (YAML::InvalidScalar) {
        ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
        exit(-1);
    }
    try {
        doc["negate"] >> negate;
    } catch (YAML::InvalidScalar) {
        ROS_ERROR("The map does not contain a negate tag or it is invalid.");
        exit(-1);
    }
    try {
        doc["occupied_thresh"] >> occ_th;
    } catch (YAML::InvalidScalar) {
        ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
        exit(-1);
    }
    try {
        doc["free_thresh"] >> free_th;
    } catch (YAML::InvalidScalar) {
        ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
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
        ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
        exit(-1);
        }
    } catch (YAML::Exception) {
        ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
        mode = TRINARY;
    }
    try {
        doc["origin"][0] >> origin[0];
        doc["origin"][1] >> origin[1];
        doc["origin"][2] >> origin[2];
    } catch (YAML::InvalidScalar) {
        ROS_ERROR("The map does not contain an origin tag or it is invalid.");
        exit(-1);
    }
    try {
        doc["image"] >> mapfname;
        // TODO: make this path-handling more robust
        if(mapfname.size() == 0)
        {
        ROS_ERROR("The image tag cannot be an empty string.");
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
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        exit(-1);
    }


}
void GridMapLoader::loadMapFromFile(std::string mapfname)

{
    MapImage = cv::imread(mapfname, cv::IMREAD_UNCHANGED);
    int n_channels = MapImage.channels();
    grid_map::GridMap grid({"occupancy"}); 
    grid_map::Position pos(origin[0], origin[1]);
    
    grid_map::GridMapCvConverter::initializeFromImage(MapImage, res, grid, pos);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(MapImage, "occupancy", grid, 0, 255,1);

    grid_map_msgs::GridMap grid_msg;
    GridMapRosConverter::toMessage(grid, grid_msg);
    //map = grid_msg;
}

