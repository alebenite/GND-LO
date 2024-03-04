// ROS
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
//#include <transport_hints.h>
//#include <sensor_msgs/point_cloud2.hpp>
#include "/opt/ros/humble/include/sensor_msgs/sensor_msgs/msg/point_cloud2.hpp"
#include "/opt/ros/humble/include/sensor_msgs/sensor_msgs/point_cloud2_iterator.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "gndlo/msg/lidar3d_sensor_info.hpp"

/// Reconfigure params
#include <memory>
#include <regex>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>


//using namespace Eigen;
using namespace std;


// ------------------------------------------------------
//						CLASS
// ------------------------------------------------------
class Cloud2Depth_Node : public rclcpp::Node//, public GNDLO_Lidar
{
  public:
	// Constructor
    Cloud2Depth_Node()
    : Node("cloud2depth")
	{
		// Declare parameters
		this->declare_all_parameters();

		// Save parameters in options
		this->get_all_parameters();
		
		// Create publisher
		topic = this->get_parameter("subs_topic").get_parameter_value().get<string>();
		depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic + "/range/image", 10);
		info_pub_ = this->create_publisher<gndlo::msg::Lidar3dSensorInfo>(topic + "/range/sensor_info", 10);

		//Creation of the QoS Polices
		rclcpp::QoS qos(rclcpp::KeepLast(10));
    	qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
		rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
		qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

		// Create subscription to the cloud point
		//cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic + "/points", qos, std::bind(&Cloud2Depth_Node::cloud_callback, this, std::placeholders::_1));
    	cloud_sub_.subscribe(this,topic + "/points", qos_profile);
		depth_sub_.subscribe(this,topic + "/range_image", qos_profile);
		
		//auto i = sensor_msgs::
		//info_sub_.subscribe(this, topic + "/range/sensor_info");
    	
		// Synchronize subscribers
		sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>>(cloud_sub_, depth_sub_, 10);
    	sync_->registerCallback(std::bind(&Cloud2Depth_Node::cloud_callback, this, std::placeholders::_1, std::placeholders::_2));

    }

  private:
	

	void declare_all_parameters()
	{
		///Declaro el descriptor y el rango que voy a usar para los parametros
		rcl_interfaces::msg::ParameterDescriptor descriptor;
		rcl_interfaces::msg::IntegerRange range;
		rcl_interfaces::msg::FloatingPointRange frange;
		
		// Declare parameters
			// ROS topic to subscribe
		this->declare_parameter("subs_topic", "/ouster");
		
		range.set__from_value(1).set__to_value(5000).set__step(1);
		descriptor.integer_range= {range};
		this->declare_parameter("width", 1024, descriptor);
		range.set__from_value(1).set__to_value(150).set__step(1);
		descriptor.integer_range= {range};
		this->declare_parameter("height", 32, descriptor);
	}

	// Get parameters
	void get_all_parameters()
	{
		// Set options
		width = this->get_parameter("width").get_parameter_value().get<int>();
		height = this->get_parameter("height").get_parameter_value().get<int>();
	}

	//------------------------------------
	// Point cloud callback
	//------------------------------------
    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg){
		cout << "Nube de puntos recibido" << endl;
    		if (true){ //Futuro flag creo
				// Declaration
    			header = cloud_msg->header;
				auto depth_msg = sensor_msgs::msg::Image();
				auto info_msg = gndlo::msg::Lidar3dSensorInfo();
				cv_bridge::CvImage cv_depth;

				cout << "1111111111111111111" << endl;
				//Inicialize de depth image as a matrix of 0's
				cv::Mat algo (height, width, CV_32FC1, cv::Scalar(0.0));
				algo.copyTo(cv_depth.image); //= cv::Mat(height, width, CV_32FC1, cv::Scalar(0.0));
				//Iterate over all the points.
				sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");///Quizas se podria pasar todo a float64 si queremos mas precision
				std::vector<float> ranges; 
				std::vector<float> alphas;			
				std::vector<_Float64> betas;
				//
				std::vector<int> pixels_y;
				//std::vector<int> pixel_x;
				//std::vector<int> pixel_y;
				cout << "222222222222222" << endl;
				//Se puede quitar un for calculando primero left right... sabiendo que posiciones mirar directamente... o no
				for(; iter_x != iter_x.end(); ++iter_x){
					float x = iter_x[0];
					float y = iter_x[1];
					float z = iter_x[2];
					//cout << "(" << x << ", " << y << ", " << z << ")" << endl;

					float denominator = std::sqrt(x*x + y*y + z*z); //If denominator equals 0 means that x and y and z are 0 which means that is a non usefull observation
					if (denominator != 0){
						ranges.push_back(std::sqrt(x*x + y*y + z*z)); 					//√x²+y²+z²
						alphas.push_back(std::atan2(y, x)); 							//arctan(y/x)
						if(std::abs(z/denominator) <= 1.0){
							betas.push_back(std::acos(z/(std::sqrt(x*x + y*y + z*z))));	//arccos(z/range)
						} else {
							betas.push_back(0.0); ///No se que control hay que hacer aquí
						}
					}
					///cout << std::asin(z/(std::sqrt(x*x + y*y + z*z))) << endl;
				}
				cout << "Calculado rango, y angulos" << endl;
				//posible control de tamaño de vector
				left = *std::min_element(alphas.begin(), alphas.end());//alphas[0];
				right = *std::max_element(alphas.begin(), alphas.end());//alphas[alphas.size() - 1];
				top = *std::min_element(betas.begin(), betas.end());//betas[0]; //mirar si es con primero y ultimo o con max y min
				down = *std::max_element(betas.begin(), betas.end());//betas[betas.size() - 1];

				float range_horizontal = right - left;//*std::max_element(alphas.begin(), alphas.end()) - *std::min_element(alphas.begin(), alphas.end());	//right - left;
				float range_vertical = down - top;//*std::max_element(betas.begin(), betas.end()) - *std::min_element(betas.begin(), betas.end()); 		//top-down;
				/*for (int i=0; i<betas.size();i++){
					cout << i << ": " << ranges[i] << ", " << alphas[i] << ", " << betas[i]<< endl;
				}*/
				//cout << "Horizontal: " << range_horizontal << ", (" << *std::min_element(alphas.begin(), alphas.end()) << ", " << *std::max_element(alphas.begin(), alphas.end()) << ")" << endl;
				//cout << "Vertical: " <<  range_vertical << ", (" << *std::min_element(betas.begin(), betas.end()) << ", " << *std::max_element(betas.begin(), betas.end()) << ")" << endl;
				for (int i = 0; i < ranges.size(); i++){
					//int pixel_x = std::round((alphas[i]-left+M_PI)/range_horizontal * width)-1;///Cambiar M_PI a una variable///static_cast<int> para truncar
					//int pixel_y = std::round((betas[i]-top)/range_vertical * height);
					//Version adaptada del code andres
					int pixel_x = std::round((alphas[i]-left+M_PI)/range_horizontal * (width)-1);
					int pixel_y = std::round((betas[i]-top)/range_vertical * (height-1));
					/*if (alphas[i]-left == 0) {
						pixel_x = 0;
					}
					if (betas[i]-top == 0) {
						pixel_y = 0;
					}*/
					if (pixel_x >= width){pixel_x = pixel_x-width;}
					//if (pixel_y >= height){pixel_y = height-1;}
					//if (pixel_x >= width){pixel_x = width-1;}
					//cv_depth.image(pixel_x, pixel_y) = ranges[i];
					/*cout << "Iteracion " << i << " de " << ranges.size() << endl;
					cout << "El pixel_x es " <<pixel_x << ", " << alphas[i]-left << " " << (alphas[i]-left)/range_horizontal << " " << (alphas[i]-left)/range_horizontal * width << endl;
					cout << "El pixel_y es " <<pixel_y << ", " << betas[i]-top << " " << (betas[i]-top)/range_vertical << " " << (betas[i]-top)/range_vertical * height << endl;
					cout << "Se le asigna: " << ranges[i] << endl;*/
					//cout << cv_depth.image.rows << " " << cv_depth.image.cols << endl;
					if(pixel_y<0 || pixel_y>=height || pixel_x <0 || pixel_x >= width){cout << "height: " << height << ", width: " << width << ", pixel y: " << pixel_y << ", pixel x_1: " << pixel_x << ", pixel x_2: " << width-pixel_x-1 << endl; }
					if(cv_depth.image.at<float>(pixel_y, width-pixel_x-1) != 0) {cout << "Se está modificando un pixel ya modificado: (" << width-pixel_x-1 << ", " << pixel_y << ")" << endl;}
					cv_depth.image.at<float>(pixel_y, width-pixel_x-1) = ranges[i];
					pixels_y.push_back(pixel_y);
					//cv_depth.image.at<float>(pixel_y, pixel_x) = ranges[i];
					//cout << "final" << endl;
				}
				std::sort(pixels_y.begin(), pixels_y.end());
				auto unique_pixels_y = std::unique(pixels_y.begin(),pixels_y.end());
				pixels_y.erase(unique_pixels_y,pixels_y.end());
				cout << pixels_y.size() << endl;
				for(int i=0;i<pixels_y.size();i++){
					//cout << "Profundidad: " << cv_depth.image.at<float>(pixels_y[i], 100) << ", en el pixel: " << pixels_y[i] << endl;
				}
				//cv::imshow("Imagen recibida", cv_depth.image);
       			//cv::waitKey(1);


				cout << "cambio imagen OpenCV a ROS" << endl;
				if (!cv_depth.image.data)
				{
					RCLCPP_ERROR_STREAM(rclcpp::get_logger("cv_bridge"), "La imagen en cv_depth es nula.");
					return;
				}
				cout << "Pre puntero" << endl;
				auto image_msg_ptr = cv_depth.toImageMsg();
				cout << "Post puntero" << endl;
				if (!image_msg_ptr)
				{
					RCLCPP_ERROR_STREAM(rclcpp::get_logger("cv_bridge"), "Error al convertir la imagen a mensaje de imagen.");
					return;
				}

				depth_msg = *image_msg_ptr;
				//depth_msg = *(cv_depth.toImageMsg());
				depth_msg.header = header;
				depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
				cout << "Mensaje: " << depth_msg.height << ", " << depth_msg.width << ", " << depth_msg.encoding << ", " << endl;
				//rclcpp::sleep_for(std::chrono::seconds(1));
				cout << "Conversion hecha, ahora a calcular la info" << endl;

				info_msg.header = header;
				info_msg.rows = height;
				info_msg.cols = width;
				info_msg.phi_start = left;
				info_msg.phi_end = right;
				info_msg.theta_start = top;
				info_msg.theta_end = down;
				info_msg.theta_vec = betas;
				info_msg.is_theta_linear = true;
				

    			depth_pub_->publish(depth_msg);
				info_pub_->publish(info_msg);
				//rclcpp::sleep_for(std::chrono::seconds(1));
				cout << "Imagen to wapa subia" << endl;
				validation(depth_msg, image_msg);
    		}
    }
	void validation (const sensor_msgs::msg::Image& result, const sensor_msgs::msg::Image::ConstSharedPtr original){
		float error = 0.0;
		for (size_t i = 0; i < result.data.size(); ++i) {
        	error += std::abs(result.data[i] - original->data[i]);
			//cout << "pixel " << i << " original: " << static_cast<int>(original->data[i]) << ", calculado: " << static_cast<int>(result.data[i]) << endl;
    	}
		cout << "Error absoluto total: " << error << ", error medio por pixel: " << error/result.data.size() << endl;
		cv_bridge::CvImagePtr cv_original = cv_bridge::toCvCopy(*original, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_bridge::CvImagePtr cv_transformed = cv_bridge::toCvCopy(result, sensor_msgs::image_encodings::TYPE_32FC1);
		cv::Mat original_8bit;
        cv_original->image.convertTo(original_8bit, CV_32F, 255.0 / 65535.0);

		cv::Mat difference;
        cv::absdiff(original_8bit, cv_transformed->image, difference);
		cv::imshow("Original Depth Image", original_8bit);
        cv::imshow("Transformed Depth Image", cv_transformed->image);
        cv::imshow("Depth Image Difference", difference);
        cv::waitKey(1); 

		double sum_diff = cv::sum(difference)[0];
		RCLCPP_INFO(this->get_logger(), "Suma de la diferencia de píxeles: %f, La media por pixel: %f, Suma total de la original: %f, Suma total de la transformada: %f", sum_diff, sum_diff/result.data.size(), cv::sum(original_8bit)[0], cv::sum(cv_transformed->image)[0]);

	}
	//------------------------------------
	// VARIABLES
	//------------------------------------
	// Variables
	string topic = "/ouster";
	//bool first_data = true;
	sensor_msgs::msg::PointCloud2 cloud;
	int width;
	int height;
	// Size image
	float left;
	float right;
	float top;
	float down;
	
	// Output results file
	//ofstream results_file;

	// Declare publishers
	std_msgs::msg::Header header;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
	rclcpp::Publisher<gndlo::msg::Lidar3dSensorInfo>::SharedPtr info_pub_;

	// Declare subscriptions and synchronizer
	//rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;///Esto es sin sync
	//rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
	message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
	message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
	std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>> sync_;
	
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
		rclcpp::spin(std::make_shared<Cloud2Depth_Node>());
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