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
        //static cv_bridge::CvImagePtr cv_in,cv_out;
        static cv_bridge::CvImagePtr cv_out;
	static cv_bridge::CvImageConstPtr cv_in;
        //cv_in = cv_bridge::toCvCopy(msg);
	unsigned char *dataOut = 0; 
	cv_bridge::CvImage cim;
        
	// publish!
        if(status_)
        {
	   // copy the first image buffer once
	   if (!cv_out)
	   {
                cv_out = cv_bridge::toCvCopy(msg, enc::BGR8);
	        dataOut = reinterpret_cast<unsigned char *>
                                           (cv_out->image.data);
		ROS_INFO("cv out is a-ok");
	   }
 
     	   try
           {
	      // share image buffer, don't copy
              cv_in = cv_bridge::toCvShare(msg, enc::BGR8);

	      if(!cv_in)
              	ROS_INFO("cv in not a-ok");
              ROS_INFO("cv in  a-ok");
	      unsigned char *dataIn = reinterpret_cast<unsigned char *>
                                           (cv_in->image.data); 
              
		// i rows, j cols 
               	for(int i=0; i<cv_in->image.rows; ++i) 
	       	{
               	  for(int j=0; j<cv_in->image.cols; ++j) 
		  {
	
			int pixIdx = 3*(i*cv_in->image.rows + j);
/*			unsigned char r = dataIn[pixIdx];
			unsigned char g = dataIn[pixIdx+1];
			unsigned char b = dataIn[pixIdx + 2];
*/
			dataOut[pixIdx] = 0;
			dataOut[pixIdx + 1] = 0;
			dataOut[pixIdx + 2] = 0;
			//dataOut[pixIdx] = b;
			//dataOut[pixIdx + 1] = g;
			//dataOut[pixIdx + 2] = r;
			//data[pixIdx] = r;
			//data[pixIdx + 2] = b;
			//data[pixIdx] = 0;
			//data[pixIdx + 2] = 0;
	
		  }
			
		}
		
           }
           catch (cv_bridge::Exception& e)
          {
               ROS_ERROR("cv_bridge exception: %s", e.what());
          }
   
            // swap R and B color channels
            //if (!cv_out)
                //cv_out = cv_bridge::toCvCopy(msg, enc::BGR8);
            //cv::cvtColor(cv_in->image, cv_in->image, CV_RGB2BGR);
            //image_pub_.publish(cv_in->toImageMsg());
            image_pub_.publish(cv_out->toImageMsg());
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
