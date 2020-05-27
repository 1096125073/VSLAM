/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "Converter.h"
#include "PointCloude.h"
#include "System.h"
#include <pcl/filters/passthrough.h>

int currentloopcount = 0;

PointCloudMapping::PointCloudMapping(double resolution_,double meank_,double thresh_)
{
    this->resolution = resolution_;
    this->meank = thresh_;
    this->thresh = thresh_;
    statistical_filter.setMeanK(meank);
    statistical_filter.setStddevMulThresh(thresh);
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >( );
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
    T2=Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd rotationVector(-M_PI/2,Eigen::Vector3d(1.0,0.0,0.0));
    T2.rotate(rotationVector);
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}
pcl::PointCloud<PointCloudMapping::PointT>::Ptr PointCloudMapping::getGlobalMap() {
 
    return this->globalMap;
}
void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth,int idk,vector<KeyFrame*> vpKFs)
{
    cout<<"receive a keyframe, id = "<<idk<<" 第"<<kf->mnId<<"个"<<endl;
    //cout<<"vpKFs数量"<<vpKFs.size()<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    currentvpKFs = vpKFs;
    //colorImgs.push_back( color.clone() );
    //depthImgs.push_back( depth.clone() );
    PointCloude pointcloude;
    pointcloude.pcID = idk;
    pointcloude.T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    pointcloude.pcE = generatePointCloud(kf,color,depth);
    pointcloud.push_back(pointcloude);
    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)//,Eigen::Isometry3d T
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            cout<<"d:"<<d<<endl;
            if (d < 0.01||d>10)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;
            
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }
    return tmp;
}


void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                cout<<"shutDownFlag"<<endl;
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
            cout<<"lck_keyframeUpdated"<<endl;
        }
        
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        if(loopbusy || bStop)
        {
            continue;
        }
        cout<<lastKeyframeSize<<"    "<<N<<endl;
        if(lastKeyframeSize == N)
          cloudbusy = false;
          cloudbusy = true;
        globalMap->clear();
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {        
            PointCloud::Ptr p (new PointCloud);
            Eigen::Isometry3d T1=pointcloud[i].T.inverse();
            pcl::transformPointCloud( *(pointcloud[i].pcE), *p,(T2*T1).matrix());
            *globalMap += *p;
        }

         //depth filter and statistical removal
         PointCloud::Ptr tmp1 ( new PointCloud );
         statistical_filter.setInputCloud(globalMap);
         statistical_filter.filter( *tmp1 );

         PointCloud::Ptr tmp2(new PointCloud());
         voxel.setInputCloud( tmp1 );
         voxel.filter( *tmp2);
        viewer.showCloud( tmp2 );
        myGrid.updateGrid(tmp2);
        cout<<"show every map, size="<<N<<"   "<<tmp2->points.size()<<endl;
 
        lastKeyframeSize = N;
        cloudbusy = false;

    }
}
void PointCloudMapping::save()
{
	pcl::io::savePCDFile( "/home/result.pcd", *globalMap );
	cout<<"globalMap save finished"<<endl;
}
void PointCloudMapping::updatecloud()
{
	if(!cloudbusy)
	{
		loopbusy = true;
		cout<<"start loop map point"<<endl;
        PointCloud::Ptr tmp3(new PointCloud);
		for (int i=0;i<currentvpKFs.size();i++)
		{
		    for (int j=0;j<pointcloud.size();j++)
		    {
				if(pointcloud[j].pcID==currentvpKFs[i]->mnFrameId)
				{
					Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(currentvpKFs[i]->GetPose() );
					PointCloud::Ptr cloud(new PointCloud);
					pcl::transformPointCloud( *pointcloud[j].pcE, *cloud, (T2*T.inverse()).matrix());

					*tmp3 +=*cloud;
					cout<<"第pointcloud"<<j<<"与第vpKFs"<<i<<"匹配"<<endl;
				}
			}
		}

        cout<<"finish loop map-------------------------"<<endl;
        PointCloud::Ptr tmp4(new PointCloud());
        voxel.setInputCloud( tmp3 );
        voxel.filter( *tmp4 );
       myGrid.updateGlobalGrid(tmp3);
       loopbusy = false;
       loopcount++;	}

}
