#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <string>

namespace enc = sensor_msgs::image_encodings;

class SwitchableCamera  
{

public:

    // Ctr
    SwitchableCamera(int camId = -1)
        : it_(nh_), camId_(camId), status_(1)
    {
    }

    void setPublisher(const std::string &topic_name) 
    {
        pub_topic_ = topic_name;	
	image_pub_ = it_.advertise(pub_topic_.c_str(), 1);
	ROS_INFO("Started publisher: %s\n", pub_topic_.c_str());
    };

    // Subscriber has callback which publishes to robot UI
    void setSubscriber(const std::string &topic_name) 
    { 
	sub_topic_ = topic_name;	
    	image_sub_ = it_.subscribe( sub_topic_.c_str(), 1, &SwitchableCamera::imageCb, this);
	ROS_INFO("Started subscriber: %s\n", sub_topic_.c_str());
    
    };

    void setStatus(const int status)
    {
	status_ = status;
	ROS_INFO("Camera %d: status = %d", camId_, status_);
    };

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
    	cv_bridge::CvImagePtr cv_ptr;
   	
	try
    	{
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    	}
    	catch (cv_bridge::Exception& e)
    	{
      	    ROS_ERROR("cv_bridge exception: %s", e.what());
      	    return;
    	}

	// publish!
	if(status_)
	{
            image_pub_.publish(cv_ptr->toImageMsg());
	}
    }


private:

    // Camera Id
    int camId_;

    // Topic name
    std::string sub_topic_; 
    std::string pub_topic_;
 
    // Each camera needs its own publisher and subscriber
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    // Camera active flag 
    int status_;
};
