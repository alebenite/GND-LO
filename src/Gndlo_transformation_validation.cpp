// ROS
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
//#include <transport_hints.h>
//#include <sensor_msgs/point_cloud2.hpp>
//#include "/opt/ros/humble/include/sensor_msgs/sensor_msgs/msg/point_cloud2.hpp"
//#include "/opt/ros/humble/include/sensor_msgs/sensor_msgs/point_cloud2_iterator.hpp"
//#include "/opt/ros/humble/include/sensor_msgs/sensor_msgs/msg/laser_scan.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
//#include <pcl/search/kdtree.h>
//#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl_ros/point_cloud.hpp>

#include <pcl_conversions/pcl_conversions.h>
//#include "/opt/ros/humble/include/pcl_conversions/pcl_conversions.h"
//
/// Reconfigure params
#include <memory>
#include <regex>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

//DEBUGGER: ros2 run --prefix 'xterm -e gdb --args' gndlo Gndlo_image_formation 
//catch throw para pillar excepciones que salen antes de que se ejecute del todo

//using namespace Eigen;
using namespace std;

class Verification_Node : public rclcpp::Node
{
  public:
        // Constructor
    Verification_Node()
    : Node("verification_node")
	{	
		this->declare_parameter("subs_topic", "/ouster");

		//Creation of the QoS Polices
		rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
		qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
		rclcpp::QoS qos(rclcpp::KeepLast(10));
    	qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
		
		// Create subscription
		topic = this->get_parameter("subs_topic").get_parameter_value().get<string>();    	
		cloud_ori_sub_.subscribe(this,topic + "/points", qos_profile);
		cloud_tra_sub_.subscribe(this,topic + "/point_cloud", qos_profile);
		depth_ori_sub_.subscribe(this,topic + "/range_image", qos_profile);
		depth_tra_sub_.subscribe(this,topic + "/range/image", qos_profile);
		
		//auto i = sensor_msgs::
		//info_sub_.subscribe(this, topic + "/range/sensor_info");
    	
		// Synchronize subscribers
		image_sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(depth_ori_sub_, depth_tra_sub_, 10);
		cloud_sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>>(cloud_ori_sub_, cloud_tra_sub_, 10);
		image_sync_->registerCallback(std::bind(&Verification_Node::depth_callback, this, std::placeholders::_1, std::placeholders::_2));
		cloud_sync_->registerCallback(std::bind(&Verification_Node::cloud_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  private:
	
	void depth_callback (const sensor_msgs::msg::Image::ConstSharedPtr& original, const sensor_msgs::msg::Image::ConstSharedPtr& result){
		cout << "imanes de profundidad entrantes" << endl;
		cv_bridge::CvImagePtr cv_original = cv_bridge::toCvCopy(*original, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_bridge::CvImagePtr cv_transformed = cv_bridge::toCvCopy(*result, sensor_msgs::image_encodings::TYPE_32FC1);
		cv::Mat original_8bit;
		double min,max,mi,ma=255.0;
		cv::minMaxLoc(cv_transformed->image,&min,&max);
		cv::minMaxLoc(cv_original->image,&mi,&ma);
        cv_original->image.convertTo(original_8bit, CV_32F, max / ma);

		cv::Mat difference;
		cv::absdiff(original_8bit, cv_transformed->image, difference);
		double mean_diff, median_diff, max_diff, min_diff, max_o, min_o, max_t, min_t;
		cv::Scalar mean, std_dev, mean_o, mean_t, std_dev_o, std_dev_t;
		cv::meanStdDev(difference, mean, std_dev);
		cv::meanStdDev(cv_original->image, mean_o, std_dev_o);
		cv::meanStdDev(cv_transformed->image, mean_t, std_dev_t);
		mean_diff = mean[0];
		cv::minMaxLoc(difference, &min_diff, &max_diff);
		cv::minMaxLoc(cv_original->image, &min_o, &max_o);
		cv::minMaxLoc(cv_transformed->image, &min_t, &max_t);
		double sum_diff = cv::sum(difference)[0];
		//RCLCPP_INFO(this->get_logger(), "Mayor distancia original: %f, mayor distancia transformada: %f, media original: %f, media transformada: %f", max_o, max_t, mean_o, mean_t);
		RCLCPP_INFO(this->get_logger(), "Suma de la diferencia de píxeles: %f\nLa media por pixel: %f\nLa mediana: %f\nEl máximo: %f\nEl mínimo: %f\nSuma total de la original: %f, Suma total de la transformada: %f", sum_diff, /*sum_diff/result.data.size()*/ mean_diff, std_dev[0], max_diff, min_diff, cv::sum(original_8bit)[0], cv::sum(cv_transformed->image)[0]);

		cv::imshow("Original Depth Image", original_8bit);
        cv::imshow("Transformed Depth Image", cv_transformed->image);
        cv::imshow("Depth Image Difference", difference);
        cv::waitKey(1); 
	}
	void cloud_callback (const sensor_msgs::msg::PointCloud2::ConstSharedPtr& original, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& result){
		cout << "Nubes de puntos entrantes" << endl;
		// Convert sensor_msgs::PointCloud2 to PCL PointCloud
		/*pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*original, *original_cloud);
		

		pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*result, *result_cloud);*/

		/*pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		kdtree->setInputCloud(result_cloud);*/


		// From sensor_msgs::msg::PointCloud2::ConstSharedPtr to pcl::PointCloud<pcl::PointXYZ>::Ptr
		pcl::PCLPointCloud2::Ptr original_aux (new pcl::PCLPointCloud2 ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud (new pcl::PointCloud<pcl::PointXYZ>());
		pcl_conversions::toPCL(*original, *original_aux);
		cout << original_aux->data.size() << endl;
		pcl::fromPCLPointCloud2(*original_aux, *original_cloud);
		//cout << "Transformación hecha" << endl;
		// Create KDTree for the result cloud
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud (original_cloud);
		//cout << "Arbol creado" << endl;
		// K nearest neighbor search
		int K = 1;
		std::vector<int> pointIdxKNNSearch(K);
		std::vector<float> pointKNNSquaredDistance(K);

		// Iterate all points from the result point cloud
		auto result_cloud = sensor_msgs::msg::PointCloud2();
		result_cloud = *result;
		sensor_msgs::PointCloud2Iterator<float> iter_x(result_cloud, "x");
		//cout << "Iterador creado" << endl;
		/// Mirar de la doc el radiusSearch a ver como va
		std::vector<float> errors(result_cloud.height*result_cloud.width); ///Imagino que esto es mas eficiente que pushback
		for(int i = 0; iter_x != iter_x.end(); ++iter_x, i++){
			pcl::PointXYZ point (iter_x[0], iter_x[1], iter_x[2]);
			//cout << "Busco" << endl;
			if ( kdtree.nearestKSearch(point, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 ) {
				errors[i] = pointKNNSquaredDistance[0];

				/*for (std::size_t i = 0; i < pointIdxKNNSearch.size (); ++i)
					std::cout << "    "  <<   (*original_cloud)[ pointIdxKNNSearch[i] ].x 
							<< " " << (*original_cloud)[ pointIdxKNNSearch[i] ].y 
							<< " " << (*original_cloud)[ pointIdxKNNSearch[i] ].z 
							<< " (squared distance: " << pointKNNSquaredDistance[i] << ")" << std::endl;*/
			}
		}
		float MSE = std::accumulate(errors.begin(), errors.end(),0.0f)/errors.size();
		float median_SE = errors[errors.size()/2];
		cout << "Error cuadrático medio: " << MSE << ", mediana de los errores cuadráticos: " << median_SE << endl;
		for (int i = 0; i<errors.size(); i++){
			errors[i] = std::sqrt(errors[i]);
		}
		cout << "Error medio: " << std::accumulate(errors.begin(), errors.end(),0.0f)/errors.size() << ", mediana: " << errors[errors.size()/2] << endl;
		/*ChatGPT

		// Variables to store the total error
		double total_error = 0.0;
		int total_points = 0;

		// Iterate through points in the original cloud
		for (const auto &point_original : original_cloud->points) {
			// Find the nearest point in the result cloud
			pcl::PointXYZ point_nearest;
			int index_nearest;
			float squared_distance;
			kdtree->nearestKSearch(point_original, 1, index_nearest, squared_distance);
			point_nearest = result_cloud->points[index_nearest];

			// Compute the error (squared Euclidean distance)
			double error = pcl::squaredEuclideanDistance(point_original, point_nearest);

			// Accumulate the error
			total_error += error;
			total_points++;
		}

		// Compute the average error
		double average_error = total_error / total_points;

		// Output the result or use it as needed
		std::cout << "Total error: " << total_error << std::endl;
		std::cout << "Total points: " << total_points << std::endl;
		std::cout << "Average error: " << average_error << std::endl;*/
	}

	// Declaration of subscribers, synchronizers and variables
	string topic = "/ouster";
	message_filters::Subscriber<sensor_msgs::msg::Image> depth_ori_sub_;
	message_filters::Subscriber<sensor_msgs::msg::Image> depth_tra_sub_;
	message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_ori_sub_;
	message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_tra_sub_;
	std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>> image_sync_;
	std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>> cloud_sync_;
};

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------


int main(int num_arg, char *argv[])
{
	//						Start
	//----------------------------------------------------------------------
	try
	{
		// Start ROS
		//----------------------------------------------------------------------
		//cout << "GNDLO Odometry node: READY." << endl;
	    rclcpp::init(num_arg, argv);
		rclcpp::spin(std::make_shared<Verification_Node>());
		rclcpp::shutdown();

		//cout << "GNDLO Odometry node: SHUTTING DOWN" << endl;

		return 0;

	}
	catch (std::exception &e)
	{
		cout << "Exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}