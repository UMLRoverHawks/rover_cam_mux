#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <string>
#include <sstream>

// rock detection messages
#include <rover_cam_detect/imgData.h>
#include <rover_cam_detect/imgDataArray.h>

// individual camera nodes
#include "SwitchableCamera.h"

class CameraMux
{

private:
   // SwitchableCameras - independent nodes
   SwitchableCamera *cams_[];
  //SwitchableCamera cams_[4];
  static const int num_cameras_ = 4;

  // sub for camera select
  ros::NodeHandle nh_;
  ros::Subscriber select_msg_sub_; 

  // pub/sub for rock detections
  ros::Subscriber detect_sub_ ;
  ros::Publisher detect_pub_; 
public:
 
  // ctr 
  CameraMux()  
  {

    // camera select msg sub 
    select_msg_sub_ = nh_.subscribe("cam_select", 1000, &CameraMux::selectCb, this );
    
    // rock detection topic sub 
    detect_sub_ = nh_.subscribe("detects", 1000, &CameraMux::detectCb, this );
    detect_pub_ = nh_.advertise<rover_cam_detect::imgDataArray>( "detects_ui", 1000 ) ; 

/*
    cams_[0].setPublisher("image_detect0");
    cams_[0].setSubscriber("image_raw0");
    cams_[1].setPublisher("image_detect1");
    cams_[1].setSubscriber("image_raw1");
    cams_[2].setPublisher("image_detect2");
    cams_[2].setSubscriber("image_raw2");
    cams_[3].setPublisher("image_detect3");
    cams_[3].setSubscriber("image_raw3");
*/

    std::stringstream pub_topic; 
    std::stringstream sub_topic;

    for(int i=0; i<num_cameras_; ++i)
    {
	// Pub/sub topic names
	sub_topic << "image_raw" << i;	
	//sub_topic << "image_detect" << i;	
	//sub_topic << "image_detect"; // from blob detect	
	pub_topic << "image_raw_mux" << i;	

	cams_[i] = new SwitchableCamera();	
	cams_[i]->setStatus(1); // status 1 = switched on
     	cams_[i]->setSubscriber(sub_topic.str().c_str());
     	cams_[i]->setPublisher(pub_topic.str().c_str());

	ROS_INFO("Created camera node %d of %d\n", i+1, num_cameras_);

	sub_topic.str("");
	pub_topic.str("");
    }

  }

  ~CameraMux()
  {
        for(int j=0; j<num_cameras_; ++j)
	{
	   delete cams_[j];
	}  
  }


  // camera_select callback
  inline void selectCb(const std_msgs::Int16::ConstPtr& msg)
  //void selectCb(const std_msgs::String::ConstPtr& msg)
  {
	// loop over SwitchableCamera status messages    
	//ROS_INFO("Select cameras: %s", msg->data.c_str());
	ROS_INFO("Select cameras: %d", msg->data);
  }



  inline void detectCb(const rover_cam_detect::imgDataArray::ConstPtr& msg)
  {
    for ( int i = 0 ; i < msg->rockData.size() ; i++ ) 
    {
      const rover_cam_detect::imgData &data = msg->rockData[i] ;
      ROS_INFO( "x: %d, y: %d, width: %d, height: %d",
               data.x, data.y, data.width, data.height ) ;
     
      detect_pub_.publish(msg) ;
    }
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_mux");
//  static const int MAX_CAMERAS = 4;
  //CameraMux cmux(MAX_CAMERAS);
  CameraMux cmux;
  ros::spin();
  return 0;
}
