#include "GridMapBase.h"
#include "OccGridMapBase.h"
#include "GridMap.h"
#include "OccGridMapUtil.h"
#include "ScanMatcher.h"
#include "MapRepMultiMap.h"
#include "HectorSlamProcessor.h"

#include "dataType.h"
#include "laserSimulation.h"

#include <iostream>
#include <opencv2/opencv.hpp>


/*void laserData2Container( const slam::sensor::LaserScan &scan, hectorslam::DataContainer &container, float scaleToMap )
{
	size_t size = 360;
	
	float angle = -3.12414f;
	container.clear();
	container.setOrigo( Eigen::Vector2f::Zero() );	

	for( int i = 0; i < size; i ++ ){
		float dist = scan.ranges[ i ];
		
		if( dist >= 0.1f && dist <= 12.0f ){
			dist *= scaleToMap;
			container.add( Eigen::Vector2f( cos(angle) * dist, sin(angle) * dist ) );
		}
	
		angle += 0.0174533f;
	}
	
	std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}*/

void laserData2Container( const slam::sensor::LaserScan &scan, hectorslam::DataContainer &container, float scaleToMap )
{
        size_t size = 901;

        float angle = -3.14159f;
        container.clear();
        container.setOrigo( Eigen::Vector2f::Zero() );

        for( int i = 0; i < size; i ++ ){
                float dist = scan.ranges[ i ];

                if( dist >= 0.2f && dist <= 50.0000000000f ){
                        dist *= scaleToMap;
                        container.add( Eigen::Vector2f( cos(angle) * dist, sin(angle) * dist ) );
                }

                angle += 0.0069800001f;
        }

        std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}


void displayMap( cv::Mat &image, const hectorslam::GridMap &gridMap )
{
	image = cv::Mat::zeros(gridMap.getSizeX(), gridMap.getSizeY(), CV_8UC3);
	
	int occupiedCount = 0;
	
	for( int i = 0; i < gridMap.getSizeX(); i ++ ){
		for( int j = 0; j < gridMap.getSizeY(); j ++ ){
			if( gridMap.isFree( i, j ) ){
				cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(255, 255, 255), 1);
                                //std::cout<<"Free Point: ( "<<i<<", "<<j<<" )"<<std::endl;
			}
			else if( gridMap.isOccupied( i, j ) ){
				occupiedCount ++;
                                //std::cout<<"Occupied Point: ( "<<i<<", "<<j<<" )"<<std::endl;
                                cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(0, 0, 255), 1);
			}
		}
	}

	std::cout<<"---------------- Result --------------------"<<std::endl;
        std::cout<<"Occupied Points Number: "<<occupiedCount<<std::endl;

	cv::imshow( "map", image);
}

	
	
int main()
{
	std::cout<<"-------------- Hector SLAM TEST ------------"<<std::endl;
		
	// ----------------- Init Parameter -------------------//
	// map parameters --- resolution / size / init pose / map levels 
        double p_map_resolution_ = 0.05;
        int p_map_size_ = 2048;
        double p_map_start_x_ = 0.5;
        double p_map_start_y_ = 0.5;
        int p_map_multi_res_levels_ = 3;

	// 地图更新参数
        double p_update_factor_free_ = 0.4;
        double p_update_factor_occupied_ = 0.9;
        double p_map_update_distance_threshold_ = 0.4;
        double p_map_update_angle_threshold_ = 0.9;


	// --------------- Laser Simulation Instance ----------//
	slam::simulation::Simulation simulation;
	// open the simulation file
        std::string file_name = "laser_data.txt";
        simulation.openSimulationFile( file_name );

	// ---------------- test -----------------------------//	
//	slam::sensor::LaserScan scan;
//        simulation.readAFrameData( scan ); // read the laser data
//	for( int i = 0; i < scan.size(); i ++ ){
//		std::cout<<"range["<<i<<"]: "<<scan.ranges[i]<<std::endl;
//	}
	

	// ------------- Create A Slam Processor ---------------//
	hectorslam::HectorSlamProcessor *slamProcessor = new hectorslam::HectorSlamProcessor( static_cast<float>( p_map_resolution_ ), 
		p_map_size_, 
		p_map_size_, 
		Eigen::Vector2f(p_map_start_x_, p_map_start_y_), 
		p_map_multi_res_levels_ );
	
	slamProcessor->setUpdateFactorFree(p_update_factor_free_);                // 0.4
        slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);        // 0.9
        slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_); // 0.4
        slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);   // 0.9

	// ----------------- Init a Image ---------------------//
        cv::Mat image = cv::Mat::zeros(slamProcessor->getGridMap(0).getSizeX(), slamProcessor->getGridMap(0).getSizeY(), CV_8UC3);
 //       cv::imshow("map", image);


	// --------------- Begin to Mapping ----------------//
	Eigen::Vector3f robotPose( 0.0f, 0.0f, 0.0f );
	
	while( !simulation.endOfFile() ){
		slam::sensor::LaserScan scan;
		simulation.readAFrameData( scan ); // read the laser data
		
		hectorslam::DataContainer scanContainer;
		laserData2Container( scan, scanContainer, slamProcessor->getScaleToMap() );
		
		std::cout<<"frame count: "<<simulation.getFrameCount()<<std::endl;
		slamProcessor->update( scanContainer, robotPose );
			
		robotPose = slamProcessor->getLastScanMatchPose();
		std::cout<<"robot pose now: "<<std::endl;
                std::cout<<robotPose<<std::endl;
                std::cout<<"------------------"<<std::endl;
		
		displayMap( image, slamProcessor->getGridMap(0) );
	
		cv::waitKey(10);
	}
	//cv::waitKey(0);
	
	// close the simulation file
        simulation.closeSimulationFile();

       	return 0;
}

