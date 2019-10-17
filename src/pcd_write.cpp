/*
Software in part adapted from

Name: Tsukasa Sugiura 
Github profile name: UnaNancyOwen
Microsoft MVP for Windows Development / Point Cloud Library Maintainer

HoloLab Inc.
Japan
t.sugiura0204@gmail.com
http://unanancyowen.com

This code is created purely for educational reasons

 */
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

//#include "kinect2_grabber.h" (may use later)
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <stdio.h>
#include <iostream> //write to console
#include <boost/thread/thread.hpp> //sleep 
#include <csignal>

#define NUM_ROTATIONS 72

typedef pcl::PointXYZRGBA PointType;
int rotate();
void savePointCloudToFile(pcl::PointCloud<PointType>::ConstPtr cloud, int cloudNumber);


int main( int argc, char* argv[] )
{
    //track how many point clouds have been saved
    static int cloudNumber = 0;
    //see if user has pressed space key to begin capture
    static bool startedCapture = false;

    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" ) );
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> pointcloud_function =
        [&cloud, &mutex]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
            boost::mutex::scoped_lock lock( mutex );
            cloud = ptr;
        };

    
    // Kinect2Grabber (may use later?)
    //boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();
    // create a new grabber for OpenNI devices
    pcl::Grabber* grabber = new pcl::OpenNIGrabber();

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( pointcloud_function );

    // Keyboard Callback Function
    boost::function<void( const pcl::visualization::KeyboardEvent& )> keyboard_function =
        [&cloud, &mutex]( const pcl::visualization::KeyboardEvent& event ){
            // Save Point Cloud to PCD File when Pressed Space Key
            if( (int)event.getKeyCode() == 32 && event.keyDown() ){
                boost::mutex::scoped_try_lock lock( mutex );
                if( lock.owns_lock() ){
                    if(cloudNumber < NUM_ROTATIONS && startedCapture == false){
                        startedCapture = true;
                    }      
                }
            }
        };

    // Register Callback Function
    viewer->registerKeyboardCallback( keyboard_function );

    // Start Grabber
    grabber->start();

    std::cout << "Press space key to begin capturing" << endl;

    while( !viewer->wasStopped() ){
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock( mutex );
        if( cloud && lock.owns_lock() ){
            if( cloud->size() != 0 ){
                /* Processing Point Cloud */

                // Update Point Cloud
                if( !viewer->updatePointCloud( cloud, "cloud" ) ){
                    viewer->addPointCloud( cloud, "cloud" );
                    viewer->resetCameraViewpoint( "cloud" );
                }
                //need to rotate NUM_ROTATIONS and save each
                if(cloudNumber < NUM_ROTATIONS && startedCapture){
                    //save point cloud
                    savePointCloudToFile(cloud, cloudNumber);
                    //rotate the turntable, check it was sucessful
                    if(rotate() == -1){
                        std::cout << "Stopping capture" << endl;
                        raise(SIGINT); //exit process
                    }
                    //increment number of point clouds captured
                    cloudNumber++;
                }
                //we've finished!
                else if(cloudNumber >= NUM_ROTATIONS){
                    std::cout << "Finished capture" << endl;
                    raise(SIGINT); //exit process
                }
                //else we haven't started capturing
               
            }
        }
    }
}

//send signal to arduino to rotate and sleep to make sure that it completes
int rotate(){
        FILE *arduino = NULL;
        //open connection
        arduino = fopen( "/dev/ttyACM0", "w");
        //boost::this_thread::sleep( boost::posix_time::milliseconds(1000) );
        //if we failed, print an error message so the user knows
        if(arduino == NULL){
            std::cout << "Connection to arduino failed" << endl;
            return -1;
        }
        //send signal to arduino to rotate the stepper motor
        fprintf(arduino,"%c",'r');
        fclose(arduino);
        //give turntable time to actually rotate before taking picture
        boost::this_thread::sleep( boost::posix_time::milliseconds(1000) );
        return 0;
        
}

//save each captured point cloud to .pcd file in the pcd_files folder (in build folder)
void savePointCloudToFile(pcl::PointCloud<PointType>::ConstPtr cloud, int cloudNumber){
    std::string file_name = "cloud.pcd"; 
    std::string file_path = "pcd_files/";
    std::stringstream full_file_path;
    full_file_path << file_path + std::to_string(cloudNumber) + file_name;
    pcl::io::savePCDFile( full_file_path.str(), *cloud, true ); // Binary format
    //pcl::io::savePCDFile( full_file_path.str(), *cloud, false ); // ASCII format
    std::cout << "Saving cloud #" << cloudNumber << endl;
    
}