// ROS
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
//#include <sensor_msgs/pointcloud2.hpp>
#include "/opt/ros/humble/include/sensor_msgs/sensor_msgs/msg/point_cloud2.hpp"
#include "/opt/ros/humble/include/sensor_msgs/sensor_msgs/point_cloud2_iterator.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

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

		//Creation of the QoS Polices
		rclcpp::QoS qos(rclcpp::KeepLast(10));
    	qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

		// Create subscription to the cloud point
		///cloud_sub_.subscribe(this, topic + "/cloud");
		cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic + "/points", qos, std::bind(&Cloud2Depth_Node::cloud_callback, this, std::placeholders::_1));
    	//cloud_sub_.subscribe(this, "/cloud");
		//cloud_sub_->registerCallback(std::bind(&Cloud2Depth_Node::cloud_callback, this, std::placeholders::_1, std::placeholders::_2));
		//cloud_sub_.subscribe("/cloud", 10)
		//info_sub_.subscribe(this, topic + "/range/sensor_info");
		
    	
		// Synchronize subscribers
		//sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::LaserScan>>(image_sub_, info_sub_, 20);
    	//sync_->registerCallback(std::bind(&GNDLO_Node::image_callback, this, std::placeholders::_1, std::placeholders::_2));

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
    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg){
		cout << "Nube de puntos recibido" << endl;
    		if (true){ //Futuro flag creo
    			header = cloud_msg->header;

				auto depth_msg = sensor_msgs::msg::Image();
				cv_bridge::CvImage cv_depth;
				cout << "1111111111111111111" << endl;
				//Inicialize de depth image as a matrix of 0's
				cv::Mat algo (height, width, CV_32FC1, cv::Scalar(0.0));
				algo.copyTo(cv_depth.image); //= cv::Mat(height, width, CV_32FC1, cv::Scalar(0.0));
				//Iterate over all the points.
				sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
				std::vector<float> ranges; 
				std::vector<float> alphas;			
				std::vector<float> betas;
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
				float left = *std::min_element(alphas.begin(), alphas.end());//alphas[0];
				float right = *std::max_element(alphas.begin(), alphas.end());//alphas[alphas.size() - 1];
				float top = *std::min_element(betas.begin(), betas.end());//betas[0]; //mirar si es con primero y ultimo o con max y min
				float down = *std::max_element(betas.begin(), betas.end());//betas[betas.size() - 1];

				float range_horizontal = right - left;//*std::max_element(alphas.begin(), alphas.end()) - *std::min_element(alphas.begin(), alphas.end());	//right - left;
				float range_vertical = down - top;//*std::max_element(betas.begin(), betas.end()) - *std::min_element(betas.begin(), betas.end()); 		//top-down;
				for (int i=0; i<betas.size();i++){
					//cout << i << ": " << ranges[i] << ", " << alphas[i] << ", " << betas[i]<< endl;
				}
				//cout << "Horizontal: " << range_horizontal << ", (" << *std::min_element(alphas.begin(), alphas.end()) << ", " << *std::max_element(alphas.begin(), alphas.end()) << ")" << endl;
				//cout << "Vertical: " <<  range_vertical << ", (" << *std::min_element(betas.begin(), betas.end()) << ", " << *std::max_element(betas.begin(), betas.end()) << ")" << endl;
				for (int i = 0; i < ranges.size(); i++){
					int pixel_x = static_cast<int>((alphas[i]-left+M_PI)/range_horizontal * width)-1;
					int pixel_y = static_cast<int>((betas[i]-top)/range_vertical * height);
					if (alphas[i]-left == 0) {
						pixel_x = 0;
					}
					if (betas[i]-top == 0) {
						pixel_y = 0;
					}
					if (pixel_x > width){pixel_x = pixel_x-width;}
					if (pixel_y >= height){pixel_y = height-1;}
					if (pixel_x >= width){pixel_x = width-1;}
					//cv_depth.image(pixel_x, pixel_y) = ranges[i];
					/*cout << "Iteracion " << i << " de " << ranges.size() << endl;
					cout << "El pixel_x es " <<pixel_x << ", " << alphas[i]-left << " " << (alphas[i]-left)/range_horizontal << " " << (alphas[i]-left)/range_horizontal * width << endl;
					cout << "El pixel_y es " <<pixel_y << ", " << betas[i]-top << " " << (betas[i]-top)/range_vertical << " " << (betas[i]-top)/range_vertical * height << endl;
					cout << "Se le asigna: " << ranges[i] << endl;*/
					//cout << cv_depth.image.rows << " " << cv_depth.image.cols << endl;
					if(pixel_y<0 || pixel_y>31 || pixel_x <0 || pixel_x > 1031){cout << "height: " << height << ", width: " << width << ", pixel y: " << pixel_y << ", pixel x_1: " << pixel_x << ", pixel x_2: " << width-pixel_x-1 << endl; }
					cv_depth.image.at<float>(pixel_y, width-pixel_x-1) = ranges[i];
					//cv_depth.image.at<float>(pixel_y, pixel_x) = ranges[i];
					//cout << "final" << endl;
				}
				//cv::imshow("Imagen recibida", cv_depth.image);
       			//cv::waitKey(1);

				//////TRASPUESTA
				// Obtener la matriz de la imagen de entrada
				/*Mat input_mat = cv_depth.image;

				// Obtener la traspuesta de la imagen
				Mat transposed_mat = input_mat.t();

				// Crear una nueva imagen de OpenCV con la traspuesta
				cv_bridge::CvImage transposed_image = std::make_shared<cv_bridge::CvImage>();
				transposed_image->header = cv_depth->header;
				transposed_image->encoding = cv_depth->encoding;
				transposed_image->image = transposed_mat;
				*/

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
///////
				depth_msg = *image_msg_ptr;
				//depth_msg = *(cv_depth.toImageMsg());
				depth_msg.header = header;
				depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
				cout << "Mensaje: " << depth_msg.height << ", " << depth_msg.width << ", " << depth_msg.encoding << ", " << endl;
				//rclcpp::sleep_for(std::chrono::seconds(1));
				cout << "Conversion hecha, ahora a publicar" << endl;
    			depth_pub_->publish(depth_msg);
				//rclcpp::sleep_for(std::chrono::seconds(1));
				cout << "Imagen to wapa subia" << endl;
    		}
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
	
	// Output results file
	//ofstream results_file;

	// Declare publishers
	std_msgs::msg::Header header;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;

	// Declare subscriptions and synchronizer
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
	//std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::LaserScan>> sync_;
	
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