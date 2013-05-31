#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

namespace enc = sensor_msgs::image_encodings;

class SwitchableCamera  
{

public:

    // Ctr
    SwitchableCamera(int camId = -1)
        : it_(nh_), camId_(camId), status_(1)
    {

	setStatus(status_);
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

	if(status)
	{
	   ROS_INFO("Camera %d ON (status = %d)", camId_, status_);
	}
	else
	{
	    ROS_INFO("Camera %d OFF (status = %d)", camId_, status_);
	}
    };

    int getId() const { return camId_; }
	 
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
	    // publish!
        if(status_)
        {
            // swap R and B color channels
            // make a new message that looks like the one that came in
            cv_bridge::CvImagePtr cv_in = cv_bridge::toCvCopy(msg, enc::BGR8);
            cv_bridge::CvImage cv_out;
            cv_out.header   = cv_in->header;
            cv_out.encoding = cv_in->encoding;
            cv_out.image    = cv_in->image.clone(); // instantiates image
            cv::cvtColor(cv_in->image, cv_out.image, CV_BGR2RGB);

            image_pub_.publish(cv_out.toImageMsg());
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
