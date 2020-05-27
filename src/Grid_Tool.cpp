//
// Created by xia on 20-4-21.
//

#include <Grid_Tool.h>
namespace MyTool {
    void sigmod(const int &input, double &output) {
        output = 1.0 / (1 + exp(-input));
    }

    void ConvertPoint2XY(const double temp_x, const double temp_y, double cellResolution, int &x, int &y) {
        x = int(floor(temp_x / cellResolution))+1;
        y = int(floor(temp_y / cellResolution))+1;
    }

    void RemoveUnusedPoint(PointCloud::Ptr cloud, double limit_z) {
        pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloud->points.begin();
        while (it != cloud->points.end()) {
            float x, y, z, rgb;
            x = it->x;
            y = it->y;
            z = it->z;
            rgb = it->rgb;
            if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z) || !pcl_isfinite(rgb) || z > limit_z) {
                it = cloud->points.erase(it);
            } else
                ++it;
        }

    }

    void MyGrid::calcSurfaceNormals(PointCloud::Ptr &cloud) {
        pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(param.searchRadius);
        ne.compute(*cloud_normals);
    }

    void
    MyGrid::getMinMaxXY(std::map<Coordiate, GridPoint> &tmp_gridPoints, int &min_x, int &max_x, int &min_y, int &max_y) {
        cout << "start getMinMaxXY" << endl;
        int x, y;
        std::map<Coordiate, GridPoint>::iterator it;
        it = tmp_gridPoints.begin();
        while (it != tmp_gridPoints.end()) {

            x = it->first.x;
            y = it->first.y;
            if (x > max_x)
                max_x = x;
            if (x < min_x)
                min_x = x;
            if (y > max_y)
                max_y = y;
            if (y < min_y)
                min_y = y;
            ++it;
        }
        cout << "getMinMaxXY" << endl;
    }

    void MyGrid::updateGrid(PointCloud::Ptr &cloud) {
        is_update = false;
        RemoveUnusedPoint(cloud, param.limit_z);
        calcSurfaceNormals(cloud);

        for (size_t i = 0; i < cloud->size(); i++) {
            double x = cloud->points[i].x;
            double y = cloud->points[i].y;
            double z = cloud->points[i].z;
            double nomal_z = cloud_normals->points[i].normal_z;
            double score;
            scoreFun(nomal_z, z, score);

            int grid_x, grid_y;
            ConvertPoint2XY(x, y, param.cellResolution, grid_x, grid_y);
            Coordiate key(grid_x, grid_y);
            cout<<"grid_x :"<<grid_x<<"grid_y :"<<grid_y;
            int count = gridPoints.count(key);
            cout << "count key:" << count << endl;
            if (count > 0) {
                GridPoint *gridpoint = &gridPoints[key];
                if (score >= param.deviation) {
                    gridpoint->obstal_counter = min(10,gridpoint->obstal_counter+1);
                } else {
                    gridpoint->obstal_counter = max(0, gridpoint->obstal_counter - 1);
                }
            } else {
                GridPoint gridPoint(grid_x, grid_y);
                if (score >= param.deviation)
                    gridPoint.obstal_counter = 1;
                gridPoints.insert(std::pair<Coordiate, GridPoint>(key, gridPoint));
            }


        }
        is_update = true;
    }
    void MyGrid::updateGlobalGrid(PointCloud::Ptr &cloud) {
                is_update = false;
                gridPoints.clear();
                RemoveUnusedPoint(cloud, param.limit_z);
                calcSurfaceNormals(cloud);
                for (size_t i = 0; i < cloud->size(); i++) {
                    double x = cloud->points[i].x;
                    double y = cloud->points[i].y;
                    double z = cloud->points[i].z;
                    double nomal_z = cloud_normals->points[i].normal_z;
                    double score;
                    scoreFun(nomal_z, z, score);
                    int grid_x, grid_y;
                    ConvertPoint2XY(x, y, param.cellResolution, grid_x, grid_y);
                    Coordiate key(grid_x, grid_y);
                    cout<<"grid_x :"<<grid_x<<"grid_y :"<<grid_y;
                    int count = gridPoints.count(key);
                    cout << "count key:" << count << endl;
                    if (count > 0) {
                        GridPoint *gridpoint = &gridPoints[key];
                        if (score >= param.deviation) {
                            gridpoint->obstal_counter = min(10,gridpoint->obstal_counter+param.global_delta);
                        } else {
                            gridpoint->obstal_counter = max(0, gridpoint->obstal_counter -param.global_delta);
                        }
                    } else {
                        GridPoint gridPoint(grid_x, grid_y);
                        if (score >= param.deviation)
                            gridPoint.obstal_counter = param.global_delta;
                        gridPoints.insert(std::pair<Coordiate, GridPoint>(key, gridPoint));
                    }
                }
        is_update = true;
    }

    MyGrid::MyGrid(void) {
        NormalCloud::Ptr temp_cloud_normals(new NormalCloud);
        cloud_normals = temp_cloud_normals;
    }

    void MyGrid::scoreFun(double normal_z, double z, double &score) {
        double part_nomal = 1 - fabs(normal_z);
        double part_z = 1 - exp(-fabs(z));
        score = param.sigma * part_nomal + (1.0 - param.sigma) * part_z;
        cout << "score :" << score << endl;
    }

    void MyGrid::getMapdata(MapData &data) {
        std::map<Coordiate, GridPoint> tmp_gridPoints(this->gridPoints);
        if (tmp_gridPoints.size() > 0) {
            int min_x = 0, max_x = 0, min_y = 0, max_y = 0;
            getMinMaxXY(tmp_gridPoints, min_x, max_x, min_y, max_y);
            ocGrid = std::vector<signed char>(param.total_grids,-1);
            std::map<Coordiate, GridPoint>::iterator it;
            it = tmp_gridPoints.begin();
            while (it != tmp_gridPoints.end()) {
                int x = it->first.x;
                int y = it->first.y;
                int position = (y + param.row_grids/2) * param.row_grids + x + param.row_grids/2;
                if(it == tmp_gridPoints.end())
                    break;
                if(position>ocGrid.size()||position<0)
                    continue;
                int counter = it->second.obstal_counter;
                cout << "counter :" << counter << endl;
                if (counter == 0)
                    ocGrid[position] = 0;
                else if (counter > param.buffer)
                    ocGrid[position] = 100;
                else
                    ocGrid[position] = 0;
                ++it;
            }
            data.data = ocGrid;
            data.height = param.row_grids;
            data.width = param.row_grids;
            data.xMin = min_x;
            data.yMin = min_y;
            data.resolution = param.cellResolution;
            is_update = false;


        }
    }
}
