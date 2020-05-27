#ifndef GRID_TOOL_H
#define GRID_TOOL_H

#include <iostream>
#include <cmath>
#include <map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

using namespace std;
namespace MyTool
{
    void sigmod(int input,double &output);
    typedef pcl::PointXYZRGBA MyPoint;
    typedef pcl::PointCloud<MyPoint> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> NormalCloud;

    struct Param
    {
        double searchRadius=0.1;
        double deviation=0.5;
        double sigma=0.5;
        int buffer=5;
        double cellResolution=0.05;
        double limit_z=1.0;
        int global_delta=2;
        double map_length=200;
        int row_grids=int(map_length/cellResolution);
        int total_grids=row_grids*row_grids;
    };

    struct Coordiate{
        int x;
        int y;
        Coordiate(){};
        Coordiate(int _x,int _y):x(_x),y(_y){
        }
        bool operator<(const Coordiate &coord) const
        {
            return (x < coord.x) || (x == coord.x && y < coord.y) ;
        }

    };
    struct MapData{
        int width=0;
        int height=0;
        int xMin=0;
        int yMin=0;
        double resolution=0.05;
        std::vector<signed char> data;
    };
    struct GridPoint{
        int x;
        int y;
        int state=-1;
        double score=0;
        int obstal_counter=0;
        GridPoint(){};
        GridPoint(int _x,int _y):x(_x),y(_y){

        }

    };


    void ConvertPoint2XY(const double temp_x,const double temp_y,double cellResolution,int &x,int &y);
    void RemoveUnusedPoint(PointCloud::Ptr cloud,double limit_z);

    class MyGrid
    {
    public:
          bool is_update=false;
          Param param;
          NormalCloud::Ptr cloud_normals;
          std::map<Coordiate, GridPoint> gridPoints;
          std::vector<signed char> ocGrid;
          MapData mapData;
          void getMapdata(MapData &data);
          void getMinMaxXY(std::map<Coordiate, GridPoint> &tmp_gridPoints,int &min_x,int &max_x,int &min_y,int &max_y);
          MyGrid();
          void calcSurfaceNormals(PointCloud::Ptr& cloud);
          void updateGlobalGrid(PointCloud::Ptr& cloud);
          void updateGrid(PointCloud::Ptr& cloud);
          void inputCloudandUpdateGrid(PointCloud::Ptr input,std::vector<signed char> &data);
    protected:
          void scoreFun(double normal_z,double z,double &score);

    };

} //namespace ORB_SLAM

#endif 
