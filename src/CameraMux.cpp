#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Byte.h"

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <string>
#include <sstream>

// rock detection messages
#include <rock_publisher/imgData.h>
#include <rock_publisher/imgDataArray.h>

// individual camera nodes
#include "SwitchableCamera.h"

class CameraMux
{

private:
   // SwitchableCameras - independent nodes
   SwitchableCamera *cams_[];
  static const int num_cameras_ = 4;

  // sub for camera select
  ros::NodeHandle nh_;
  ros::Subscriber select_msg_sub_; 

  // pub/sub for rock detections
  ros::Subscriber detect_sub_ ;
  // inelegant replication
  ros::Publisher detect_pub_; 
public:
 
  // ctr 
  CameraMux()  
  {

    // camera select msg sub 
    select_msg_sub_ = nh_.subscribe("cam_select", 1, &CameraMux::selectCb, this );
    
    // rock detection topic sub
    // TODO: replicate for muliple streams of detects - is there
    // a better way rather than cut/paste?
    std::stringstream sub_topic, pub_topic;

    for(int i=0; i<num_cameras_; ++i)
    {
	// Pub/sub topic names
	sub_topic << "/camera" << i << "/image_raw";	
	pub_topic << "/camera" << i << "_stream/image_raw";	

	cams_[i] = new SwitchableCamera(i);	
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
  inline void selectCb(const std_msgs::Byte::ConstPtr& msg)
  {
	// loop over SwitchableCamera status messages 
	ROS_INFO("Select these cameras: %d", msg->data);

	char byteMsg =  msg->data | 1;

	for(int i=0; i<num_cameras_; ++i)
    	{
   	    cams_[i]->setStatus( (byteMsg >> cams_[i]->getId() ) & 1 ) ;	
	}
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_mux");
//  static const int MAX_CAMERAS = 4;
  //CameraMux cmux(MAX_CAMERAS);
  CameraMux cmux;
//  ros::spin();
  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin();
  return 0;
}
