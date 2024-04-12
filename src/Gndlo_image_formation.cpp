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
#include "transformer/msg/lidar3d_sensor_info.hpp"

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

//No sincronizar con sensor info, este actualiza parametros y los parametros tengo que mirarlos o por la imagen que he sacado o por mi metodo viendo el sensor info que publico yo
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
		
		//Creation of the QoS Polices
		rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
		qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
		rclcpp::QoS qos(rclcpp::KeepLast(10));
    	qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
		
		// Create publisher
		topic = this->get_parameter("subs_topic").get_parameter_value().get<string>();
		depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic + "/range/image", qos);
		info_pub_ = this->create_publisher<transformer::msg::Lidar3dSensorInfo>(topic + "/range/sensor_info", qos);
		cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic + "/point_cloud", 10);

		// Create subscription to the cloud point
    	cloud_sub_.subscribe(this,topic + "/points", qos_profile);
		depth_sub_.subscribe(this,topic + "/range_image", qos_profile);
		info_sub_.subscribe(this,topic + "/range_sensor_info", qos_profile);
		info_sub_.registerCallback(std::bind(&Cloud2Depth_Node::sensor_info_callback, this, std::placeholders::_1));
		
		// Synchronize subscribers
		sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>>(cloud_sub_, depth_sub_, 10);
		// INPUT CLOUD POINT
    	//sync_->registerCallback(std::bind(&Cloud2Depth_Node::cloud_callback, this, std::placeholders::_1, std::placeholders::_2));
		// INPUT DEPTH IMAGE
		sync_->registerCallback(std::bind(&Cloud2Depth_Node::depth_callback, this, std::placeholders::_1, std::placeholders::_2));
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
		frange.set__from_value(-M_PI).set__to_value(M_PI).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("phi_start", -M_PI, descriptor);
		this->declare_parameter("phi_end", M_PI, descriptor);
		this->declare_parameter("theta_start", 0.35116025, descriptor);
		this->declare_parameter("theta_end", -0.37751472, descriptor);
		frange.set__from_value(0).set__to_value(1000).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("max_range", 130.8, descriptor);
	}

	// Get parameters
	void get_all_parameters()
	{
		// Set options
		width = this->get_parameter("width").get_parameter_value().get<int>();
		height = this->get_parameter("height").get_parameter_value().get<int>();
		phi_start = this->get_parameter("phi_start").get_parameter_value().get<float>();
		phi_end = this->get_parameter("phi_end").get_parameter_value().get<float>();
		theta_start = this->get_parameter("theta_start").get_parameter_value().get<float>();
		theta_end = this->get_parameter("theta_end").get_parameter_value().get<float>();
		max_range = this->get_parameter("max_range").get_parameter_value().get<float>();
	}

	//------------------------------------
	// sensor info callback
	//------------------------------------
	void sensor_info_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& info_msg){
		height = info_msg->range_min;
		width = info_msg->range_max;
		phi_start = info_msg->angle_min;
		phi_end = info_msg->angle_max;
		if (info_msg->ranges.size() != 0){ // If the sensor has no vector of thetas means that the angle theta changes linearly
			theta_linear = false;
			theta_ranges = info_msg->ranges;
		} else{
			theta_linear = true;
		}
	}

	//------------------------------------
	// depth image callback
	//------------------------------------
	void depth_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg){
		cout << "Imagen recibida" << endl;
		// Declarations
		header = image_msg->header;
		auto point_msg = sensor_msgs::msg::PointCloud2();
		cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
		sensor_msgs::PointCloud2Modifier point_field(point_msg);

		cout << "Variables declaradas" << endl;
		// Iterate over all pixels
		std::vector<float> x, y, z;
		int counter=0;
		float prop = max_range/std::pow(2,15);
		///Se podria mejorar creo estableciendo que el tamaño de la point cloud tiene que ser respecto a la imagen pero quitando los valores con 0 que esos son donde no se ha detectado nada
		for (float row=0; row < cv_depth->image.rows; row++){
			for (float col=0; col < cv_depth->image.cols; col++){
				//Obtain range and angles for every pixel to calculate x,y,z
				float range = prop*cv_depth->image.at<float>(row,col); ///Aquí habría que cambiar cosas si la imagen tiene que ir de 0 a 255
				if (range == 0) {continue;} //

				float phi, theta;
				phi = -(phi_end-phi_start)*(col/(cv_depth->image.cols))-phi_start-desfase; //Range of angles * horizontal percentaje of the pixel + the start angle
				if (theta_linear){
					theta = (theta_end-theta_start)*(row/(cv_depth->image.rows-1))+theta_start; //Range of angles * vertical percentaje of the pixel + the start angle
				}else{
					theta = theta_ranges[static_cast<int>(row)];
				}

				x.push_back(range * std::cos(theta) * std::cos(phi));
				y.push_back(range * std::cos(theta) * std::sin(phi));
				z.push_back(range * std::sin(theta));
			}
		}
		// Set the Point Fields
		// Define the fields
		point_field.setPointCloud2Fields(3,
			"x", 1, sensor_msgs::msg::PointField::FLOAT32,
			"y", 1, sensor_msgs::msg::PointField::FLOAT32,
			"z", 1, sensor_msgs::msg::PointField::FLOAT32
		);
		///Se podria mejorar creo estableciendo que el tamaño de la point cloud tiene que ser respecto a la imagen pero quitando los valores con 0 que esos son donde no se ha detectado nada
		// Set the message values
		point_msg.header = header;
		point_msg.is_dense = true;
		point_msg.is_bigendian = false;
		
		point_msg.height = 1;
		point_msg.width = x.size();
		// Total number of bytes per point
		point_msg.point_step = 12;
		point_msg.row_step = point_msg.point_step * point_msg.width;
		point_msg.data.resize(point_msg.row_step * point_msg.height);
		// Data
		cout << "Mensaje casi creado, falta meterle los datos" << endl;
		sensor_msgs::PointCloud2Iterator<float> iter_x(point_msg, "x");
		
		for(int i=0; iter_x != iter_x.end(); ++iter_x, i++){
			iter_x[0] = x[i]; //x
			iter_x[1] = y[i]; //y
			iter_x[2] = z[i]; //z
			
		}
		
		cout << "Falta publicar" << endl;
		cout << "Tamaño de fields: " << point_msg.fields.size() << ", tamaño de data: " << point_msg.data.size() << ", proporcion: " << point_msg.data.size()/point_msg.fields.size() << ", cantidad que se supone que hay: " << width*height << endl;
		cloud_pub_->publish(point_msg);
	}
	/// Posible opcion para cambiar de float a int en la imagen ya que le tengo metidos metros, se podrian poner a valores de los pixeles
	//------------------------------------
	// Point cloud callback
	//------------------------------------
    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg, const sensor_msgs::msg::Image::ConstSharedPtr& image_msg){
		cout << "Nube de puntos recibido" << endl;
    		if (true){ //Futuro flag creo
				// Declarations
    			header = cloud_msg->header;
				auto depth_msg = sensor_msgs::msg::Image();
				auto info_msg = transformer::msg::Lidar3dSensorInfo();
				cv_bridge::CvImage cv_depth;
				///Aquie creoq ue se puede mejorar que como la nube de puntos tiene su height y su width imagino que podria actualizar el param desde ahi 

				//Inicialize de depth image as a matrix of 0's
				cv::Mat algo (height, width, CV_32FC1, cv::Scalar(0.0));
				algo.copyTo(cv_depth.image);
				//Iterate over all the points.
				sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");///Quizas se podria pasar todo a float64 si queremos mas precision
				std::vector<float> ranges; 
				std::vector<float> alphas;			
				std::vector<_Float64> betas;
				std::vector<_Float64> range_vec;
				//
				std::vector<int> pixels_y;
				
				int aux=0;
				//Se puede quitar un for calculando primero left right... sabiendo que posiciones mirar directamente... o no
				for(int i = 0; iter_x != iter_x.end(); ++iter_x, i++){
					float x = iter_x[0];
					float y = iter_x[1];
					float z = iter_x[2];
					//cout << "(" << x << ", " << y << ", " << z << ")" << endl;

					float denominator = std::sqrt(x*x + y*y + z*z); //If denominator equals 0 means that x and y and z are 0 which means that is a non usefull observation
					if (denominator != 0){
						ranges.push_back(std::sqrt(x*x + y*y + z*z)); 					//√x²+y²+z²
						float angle = std::atan2(y, x);
						if (angle < 0.0) {angle = 2*M_PI+angle;}
						alphas.push_back(angle); 										//arctan(y/x)
						if(std::abs(z/denominator) <= 1.0){
							betas.push_back(std::asin(z/(std::sqrt(x*x + y*y + z*z))));	//arccos(z/range)
							if(i/width-aux >= 0){
								range_vec.push_back(std::asin(z/(std::sqrt(x*x + y*y + z*z))));
								aux++;
							}
						} else {
							betas.push_back(0.0); ///No se que control hay que hacer aquí
						}
					}
				}
				///Se puede en el futuro poner opcion para elegir entre ambos
				/*left = *std::min_element(alphas.begin(), alphas.end());//alphas[0];
				right = *std::max_element(alphas.begin(), alphas.end());//alphas[alphas.size() - 1];
				down = *std::min_element(betas.begin(), betas.end());//betas[0]; //mirar si es con primero y ultimo o con max y min
				top = *std::max_element(betas.begin(), betas.end());//betas[betas.size() - 1];*/

				float range_horizontal = right - left;
				float range_vertical = top - down;

				for (int i = 0; i < ranges.size(); i++){
					int pixel_x = std::round((alphas[i]-left/*+desfase*/)/range_horizontal * (width));///raro
					int pixel_y = std::round((-betas[i]+top)/range_vertical * (height-1));
					
					if(pixel_y<0 || pixel_y>=height || pixel_x <0 || pixel_x >= width)
					{cout << "Iteración: " << i << " alpha: " << alphas[i] << " " << (alphas[i] - left) << " " << range_horizontal  << ", beta: " << betas[i] << ", pixel y: " << pixel_y << ", pixel x_1: " << pixel_x << /*", pixel x_2: " << width-pixel_x-1 <<*/ endl; }
					cv_depth.image.at<float>(pixel_y, pixel_x) = ranges[i]; ///AQUI, en caso de que la de andres sea con ranges y el resto sea con puntos de 0-255 habría que meter un flag
					pixels_y.push_back(pixel_y);
				}

				// From image OpenCV to image ROS
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

				// Building the messages
				depth_msg = *image_msg_ptr;
				depth_msg.header = header;
				depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

				info_msg.header = header;
				info_msg.rows = height;
				info_msg.cols = width;
				info_msg.phi_start = left;
				info_msg.phi_end = right;
				info_msg.theta_start = top;
				info_msg.theta_end = down;
				info_msg.theta_vec = range_vec;
				info_msg.is_theta_linear = true;

				// Publishing the messages
    			depth_pub_->publish(depth_msg);
				info_pub_->publish(info_msg);

    		}
    }
	void validation (const sensor_msgs::msg::Image& result, const sensor_msgs::msg::Image::ConstSharedPtr original){

		cv_bridge::CvImagePtr cv_original = cv_bridge::toCvCopy(*original, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_bridge::CvImagePtr cv_transformed = cv_bridge::toCvCopy(result, sensor_msgs::image_encodings::TYPE_32FC1);
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

	//------------------------------------
	// VARIABLES
	//------------------------------------
	// Variables
	string topic = "/ouster";
	sensor_msgs::msg::PointCloud2 cloud;
	int width;
	int height;
	float desfase=M_PI;
	// Sensor info
	float phi_start, phi_end;
	float theta_start, theta_end;
	float max_range;
	float max_image=0.0;
	bool theta_linear = false;
	std::vector<float> theta_ranges {0.3511602455, 0.3291690969, 0.3070034154, 0.2844886681, 0.2614503219, 0.2382374429, 0.2148500309, 0.1909390202, 
									0.1675516082, 0.1436405974, 0.1192059879, 0.0951204442, 0.0706858347, 0.0459021593, 0.0214675498, -0.0029670597, -0.0275762022, 
									-0.0520108117, -0.0764454212, -0.1012290966, -0.1253146403, -0.149225651, -0.1731366618, -0.1968731396, -0.2204350845, 
									-0.2439970294, -0.2668608426, -0.2895501229, -0.3120648703, -0.3342305518, -0.3560471674, -0.3775147172};
	// Size image
	float left = 2*M_PI;
	float right = 0;
	float top = 0.35116025;
	float down = -0.37751472;
	
	// Declare publishers
	std_msgs::msg::Header header;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
	rclcpp::Publisher<transformer::msg::Lidar3dSensorInfo>::SharedPtr info_pub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

	// Declare subscriptions and synchronizer
	message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
	message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
	message_filters::Subscriber<sensor_msgs::msg::LaserScan> info_sub_;
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
		cout << "GNDLO transformation node: READY." << endl;
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