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
#include <pcl/filters/passthrough.h>

#include <boost/thread/thread.hpp> //sleep 
#include <csignal>
#include <unistd.h> //UNIX Standard function definitions
#include <fcntl.h> //File control
#include <errno.h> //Error number def
#include <termios.h> //POSIX terminal control
#include <termio.h>


#define NUM_ROTATIONS 10

typedef pcl::PointXYZRGBA PointType;

int rotate(int fd);
void savePointCloudToFile(pcl::PointCloud<PointType>::ConstPtr cloud, int cloudNumber );
pcl::PointCloud<PointType>::Ptr filterCloud(pcl::PointCloud<PointType>::ConstPtr cloud, std::string axis, double minLimit, double maxLimit );

int open_port(){ //-1 is a error
  int port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
  /*O_RDWR POSIX read write
  O_NOCTTY: not a "controlling terminal"
  O_NDELAY": Ignore DCD signal state*/

  if(port == -1){
    perror("open_port: Unable to open /dev/ttyACM0 - ");
  }else fcntl(port, F_SETFL, 0);
  return (port);
}

int set_port(int port){
  struct termios options;
  tcgetattr(port, &options);

  cfsetispeed(&options, B9600); //Typical way of setting baud rate. Actual baud rate are contained in the c_ispeed and c_ospeed members
  cfsetospeed(&options, B9600);
  options.c_cflag |= (CLOCAL|CREAD);

  options.c_cflag &= ~CSIZE;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~PARENB;
  options.c_cflag |= CS8;     //No parity, 8bits, 1 stop bit (8N1)
  options.c_cflag &= ~CRTSCTS;//CNEW_RTSCTS; //Turn off flow control

  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //Make sure that Canonical input is off (raw input data)

  options.c_iflag &= ~(IXON | IXOFF | IXANY); //Turn off software control flow

  options.c_oflag &= ~OPOST; //Raw output data

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 10;

  return tcsetattr (port, TCSANOW, &options); //Make changes now without waiting for data to complete.
}

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
            cloud = filterCloud(cloud, "x", -0.5, 0.5);
            cloud = filterCloud(cloud, "y", -0.5, 0.5);
            cloud = filterCloud(cloud, "z", 0.0, 1.0);
           
        };

    
    // Kinect2Grabber (may use later?)
    //boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();
    // create a new grabber for OpenNI devices
    boost::shared_ptr<pcl::Grabber> grabber(new pcl::OpenNIGrabber());

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
    std::cout << "Setting up connection to arduino" << endl;
    //open up connection to arduino
    int fd = open_port();
    set_port(fd);
    
    usleep(1000000); //Wait for 1 sec
    int n = write(fd, "b", 1); //test that it works
    if(n < 0){
        std::cout <<" Connection to arduino failed" <<endl;
        close(fd);
        raise(SIGINT); //exit process
    }

    std::cout << "Starting grabber" << endl;
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
                    rotate(fd);
                    //increment number of point clouds captured
                    cloudNumber++;
                
                    
                }
                //we've finished!
                else if(cloudNumber >= NUM_ROTATIONS){
                    std::cout << "Finished capture" << endl;
                    close(fd);
                    raise(SIGINT); //exit process
                }
                //else we haven't started capturing
               
            }
        }
    }

}


//send signal to arduino to rotate and sleep to make sure that it completes
int rotate(int fd){
        int n;
        //send signal to arduino to rotate the stepper motor
        n = write(fd, "r", 1);
        //if we failed, print an error message so the user knows
        if( n < 0 ){
            close(fd);
            std::cout << "ERROR: unable to write data to arduino" << endl;
            raise(SIGINT);
        }
        //give turntable time to actually rotate before taking picture
        boost::this_thread::sleep( boost::posix_time::milliseconds(3000) );
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

//filter the cloud to fit within minLimit and maxLimit along axis
pcl::PointCloud<PointType>::Ptr filterCloud(pcl::PointCloud<PointType>::ConstPtr cloud, std::string axis, double minLimit, double maxLimit ){
    pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (axis);
    pass.setFilterLimits (minLimit, maxLimit);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
    return cloud_filtered;

}