#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>


using namespace cv;

class Params
{
  public:
    std::string posServer;
    std::string videoDevice;
    std::string camParamsFile;
    std::map <int, cv::Point3f> points;
    float markerSize = 1.0f;
    float pointsScale = 1.0f;
    std::string ffmpegCmd;
    bool headless;
    int xRes = 640;
    int yRes = 480;
    int xResOut = 640;
    
  void readConfig(std::string configFile) {
    FileStorage fs(configFile, FileStorage::READ);
    fs["ffmpegCmd"] >> ffmpegCmd;
    fs["posServer"] >> posServer;
    fs["videoDevice"] >> videoDevice;
    fs["camParamsFile"] >> camParamsFile;
    
    
    markerSize = (float)fs["markerSize"];
    pointsScale = (float)fs["pointsScale"];
    headless = (int)fs["headless"];
    xRes = (int)fs["xRes"];
    yRes = (int)fs["yRes"];
    xResOut = (int)fs["xResOut"];
    
    FileNode mypoints = fs["points"];
    FileNodeIterator it = mypoints.begin(), it_end = mypoints.end();
    for(int idx =0 ; it != it_end; ++it, idx++ )
    {
	Point3f *point = new Point3f();
	point->x=(int)(*it)["x"];
	point->y=(int)(*it)["y"];
	point->z=(int)(*it)["z"];
	points[(int)(*it)["id"]] = *point;
    }
    fs.release();
  }
};


