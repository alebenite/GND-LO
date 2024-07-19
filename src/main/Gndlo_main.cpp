#include "Gndlo.h"
#include "Gndlo_Lidar.h"

// ROS
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>

/// Reconfigure params
#include <memory>
#include <regex>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

/// Patches
#include "gndlo/msg/patches.hpp"

/// Lidar3dSensorInfo
#include "transformer/msg/lidar3d_sensor_info.hpp"

using namespace Eigen;
using namespace std;


// ------------------------------------------------------
//						CLASS
// ------------------------------------------------------
class GNDLO_Node : public rclcpp::Node, public GNDLO_Lidar
{
  public:
	// Constructor
    GNDLO_Node()
    : Node("gndlo_node")
	{
		// Initialize variables
		odom_pose = Matrix4f::Identity();
		odom_cov = MatrixXf::Zero(6,6);

		// Declare parameters
		this->declare_all_parameters();

		// Save parameters in options
		this->get_all_parameters(); ///Por que se hace esta linea. Hacer el get no devuelve nada creo, no?
		
		/// Parameter event handler to reconfigure
		param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
		
		auto cb = [this](const rcl_interfaces::msg::ParameterEvent & event) { 
		// Look for any updates to parameters in "/a_namespace" as well as any parameter changes 
		// to our own node ("this_node") 
			std::regex re("(/gndlo*)"); 
			///cout << "Entra evento" << event.node << endl;
			if (regex_match(event.node, re)) {
			        // You can also use 'get_parameters_from_event' to enumerate all changes that came
			        // in on this event
			        auto params = rclcpp::ParameterEventHandler::get_parameters_from_event(event);
			        for (auto & p : params) {
				  RCLCPP_INFO(
				    this->get_logger(),
				    "cb3: Received an update to parameter \"%s\" of type: %s: \"%s\"",
				    p.get_name().c_str(),
				    p.get_type_name().c_str(),
				    p.value_to_string().c_str());
	    			    ///options.*(p.get_name().c_str()) = this->get_parameter(p.get_name().c_str()).get_parameter_value().get<p.get_type()>();
	    			    ///cout << optionsMap<double Options::*>.find("count_goal") << endl;
				    ///auto it = optionsMap<double GNDLO::Options::*>::find("count_goal");
				    ///if (it != optionsMap.end()){ options.*(it->second) = this->get_parameter(p.get_name()).get_parameter_value().get<p.get_type()>(); }
				    ///std::invoke(p.get_name().c_str(), options) = this->get_parameter(p.get_name().c_str()).get_parameter_value().get<p.get_type()>();
			        }
			        get_all_parameters();
  			}
		};
		handle = param_handler_->add_parameter_event_callback(cb);

		// Create publisher
		pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("gndlo/odom_pose_cov", 10);
		odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("gndlo/odom", 10);
		///Que diferencia hay entre hacerlo asi y hacerlo como viene en la documentación? param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
		ptch_pub_ = this->create_publisher<gndlo::msg::Patches>("gndlo/patches", 10);

		// Create subscription to image and info
		topic = this->get_parameter("subs_topic").get_parameter_value().get<string>();
		image_sub_.subscribe(this, topic + "/range/image");
    	info_sub_.subscribe(this, topic + "/range/sensor_info");
    	
		// Synchronize subscribers
		sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, transformer::msg::Lidar3dSensorInfo>>(image_sub_, info_sub_, 20);
    	sync_->registerCallback(std::bind(&GNDLO_Node::image_callback, this, std::placeholders::_1, std::placeholders::_2));

		// Open file to save results
		if (options.flag_save_results)///////Esto hay que ajustarlo para que se pueda modificar con el flag. No funcionaba bien
		{
			if (options.results_file_name.size() < 5)
			{
				RCLCPP_WARN(this->get_logger(), "Invalid name for results file: ");
				RCLCPP_WARN(this->get_logger(), options.results_file_name.c_str());
				options.flag_save_results = false;
			}
			else
			{
				if (options.flag_verbose)
					RCLCPP_INFO(this->get_logger(), "Saving results to ");
					RCLCPP_INFO(this->get_logger(), options.results_file_name.c_str());
				//results_file.open(options.results_file_name);
				results_file.open(options.results_file_name, std::ios::out | std::ios::app);
			}
		}
    }

	// Destructor
	~GNDLO_Node()
	{
		// Ensure results file is closed
		if (results_file.is_open())
			results_file.close();
	}

  private:
	//------------------------------------
	// PARAMETERS
	//------------------------------------
  	// Declare parameters
	void declare_all_parameters()
	{
		///Declaro el descriptor y el rango que voy a usar para los parametros
		rcl_interfaces::msg::ParameterDescriptor descriptor;
		rcl_interfaces::msg::IntegerRange range;
		rcl_interfaces::msg::FloatingPointRange frange;
		
		// Declare parameters
			// ROS topic to subscribe
		descriptor.description = "Where the node should subscribe for the input images and the sensor information. It should have this structure: /subs_topic/range/image, /subs_topic/range/sensor_info, where the first topic is sensor_msgs/Image and the second is sensor_msgs/LaserScan.";
		this->declare_parameter("subs_topic", "/kitti", descriptor);
		//this->declare_parameter("qos_reliability", "/kitti", descriptor);
		//this->declare_parameter("qos_", "/kitti", descriptor);

			// Flags
		descriptor.description = "True to output in terminal some information about timing.";
		this->declare_parameter("flag_verbose", true);
		descriptor.description = "Wether or not to blur the flatness image (recommended: true).";
		this->declare_parameter("flag_flat_blur", true);
		descriptor.description = "Wether or not to use patches from both input scans. More accurate, but needs more time.";
		this->declare_parameter("flag_solve_backforth", true);
		descriptor.description = "Wether or not to filter the resulting motion estimation using previous instances of the movement.";
		this->declare_parameter("flag_filter", true);
			// Output
		descriptor.description = "Wether the resulting poses should be saved in a Freiburg style .txt file.";
		this->declare_parameter("flag_save_results", true);
		descriptor.description = "File name of the results if the flag_save_results is set to true.";
		this->declare_parameter("results_file_name", "/home/alex/ros2_ws/results/testData.dat");
			// General
		descriptor.description = "Number of threads used by Ceres solver";
		range.set__from_value(1).set__to_value(64).set__step(1);
		descriptor.integer_range= {range};
		this->declare_parameter("num_threads", 8, descriptor);
		descriptor.description = "Ratio (from 0 to 1) of valid pixels in a neighborhood to consider it a valid group of points.";
		frange.set__from_value(0.0).set__to_value(1.0).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("valid_ratio", 0.8, descriptor);
			// Gaussian filtering
		descriptor.description = "Radius used in kernels to calculate the curvature and smooth the flatness image. radius = 1 means 3x3 blocks.";
		range.set__from_value(0).set__to_value(15).set__step(1);
		descriptor.integer_range= {range};
		this->declare_parameter("select_radius", 6, descriptor);
		descriptor.description = "Sigma used in gaussian kernels to blur flatness and obtain points. Set to -1 to make it based on the radius.";
		frange.set__from_value(-1.0).set__to_value(4.0).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("gaussian_sigma", 0.5, descriptor);
			// Quadtree selection
		descriptor.description = "Threshold on the average of the quadtree blocks to select them as a patch. Higher means more (less flat) patches are selected).";
		frange.set__from_value(0.0).set__to_value(2.0).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("quadtrees_avg", 0.1, descriptor);
		descriptor.description = "Threshold on the standard deviation of the quadtree blocks to divide them. Higher means blocks are divided less often.";
		frange.set__from_value(0.0).set__to_value(2.0).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("quadtrees_std", 0.015, descriptor);
		descriptor.description = "Minimum level when performing quadtrees = 2^quadtrees_min_lvl. 1 means 2x2 blocks are also tested for selection.";
		range.set__from_value(1).set__to_value(5).set__step(1);
		descriptor.integer_range= {range};
		this->declare_parameter("quadtrees_min_lvl", 2, descriptor);
		descriptor.description = "Maximum level when performing quadtrees = 2^quadtrees_max_lvl. 5 means testing starts at 32x32 blocks.";
		range.set__from_value(2).set__to_value(6).set__step(1);
		descriptor.integer_range= {range};
		this->declare_parameter("quadtrees_max_lvl", 5, descriptor);
			// Orthog Culling
		descriptor.description = "Goal of number of patches contributing to each principal direction. Higher means more patches are included for each direction.";
		frange.set__from_value(0.0).set__to_value(200.0).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("count_goal", 50., descriptor);
		descriptor.description = "Starting block size when culling. 4 means blocks of 4x4 are included as the initial set, then smaller blocks are tested.";
		range.set__from_value(0).set__to_value(100).set__step(1);
		descriptor.integer_range= {range};
		this->declare_parameter("starting_size", 4, descriptor);
			// Ground clustering
		descriptor.description = "Threshold on the angular difference (between patch and previous ground plane) to consider patch as ground, in degrees.";
		frange.set__from_value(0.0).set__to_value(90.0).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("ground_threshold_deg", 10., descriptor);
		descriptor.description = "Threshold on the angular difference (between patch and previous ground plane) to consider patch as wall, in degrees.";
		frange.set__from_value(0.0).set__to_value(90.0).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("wall_threshold_deg", 60., descriptor);
			// Solution
		descriptor.description = "Maximum iterations on the update of the correspondences. It averages around 7 iterations per estimation.";
		range.set__from_value(1).set__to_value(30).set__step(1);
		descriptor.integer_range= {range};
		this->declare_parameter("iterations", 5, descriptor);
		descriptor.description = "Starting value of the huber loss function parameter. Gets substituted with MAD of residuals later.";
		frange.set__from_value(0.0).set__to_value(5.0).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("huber_loss", 3e-5, descriptor);
		descriptor.description = "Maximum translation in any direction. Set to very high to ignore it.";
		frange.set__from_value(0.0).set__to_value(200.0).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("trans_bound", 1., descriptor);
			// Convergence
		descriptor.description = "Pixel difference threshold when updating correspondences to check convergence.";
		frange.set__from_value(0.0).set__to_value(50.0).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("pix_threshold", 5., descriptor);
		descriptor.description = "Convergence threshold on the norm of the difference in translation.";
		frange.set__from_value(0.0).set__to_value(0.5).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("trans_threshold", 0.002, descriptor);
		descriptor.description = "Convergence threshold on the norm of the difference in translation.";
		frange.set__from_value(0.0).set__to_value(M_PI/90.0).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("rot_threshold", 0.5*(M_PI/180.), descriptor);
			// Filter
		descriptor.description = "";
		frange.set__from_value(0.0).set__to_value(2000.0).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("filter_kd", 100., descriptor);
		descriptor.description = "";
		frange.set__from_value(0.0).set__to_value(12.0).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("filter_pd", 0., descriptor);
		descriptor.description = "";
		frange.set__from_value(0.0).set__to_value(8.0).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("filter_kf", 2., descriptor);
		descriptor.description = "";
		frange.set__from_value(0.0).set__to_value(4.0).set__step(0);
		descriptor.floating_point_range= {frange};
		this->declare_parameter("filter_pf", 1., descriptor);
	}

	// Get parameters
	void get_all_parameters()
	{
		// Set options
			// General
		options.num_threads = this->get_parameter("num_threads").get_parameter_value().get<int>();
		options.valid_ratio = this->get_parameter("valid_ratio").get_parameter_value().get<double>();
			// Flags
		options.flag_verbose = this->get_parameter("flag_verbose").get_parameter_value().get<bool>();
		options.flag_flat_blur = this->get_parameter("flag_flat_blur").get_parameter_value().get<bool>();
		options.flag_solve_backforth = this->get_parameter("flag_solve_backforth").get_parameter_value().get<bool>();
		options.flag_filter = this->get_parameter("flag_filter").get_parameter_value().get<bool>();
			// Gaussian filtering
		options.select_radius = this->get_parameter("select_radius").get_parameter_value().get<int>();
		options.gaussian_sigma = this->get_parameter("gaussian_sigma").get_parameter_value().get<double>();
			// Quadtree selection
		options.quadtrees_avg = this->get_parameter("quadtrees_avg").get_parameter_value().get<double>();
		options.quadtrees_std = this->get_parameter("quadtrees_std").get_parameter_value().get<double>();
		options.quadtrees_min_lvl = this->get_parameter("quadtrees_min_lvl").get_parameter_value().get<int>();
		options.quadtrees_max_lvl = this->get_parameter("quadtrees_max_lvl").get_parameter_value().get<int>();
			// Orthog Culling
		options.count_goal = this->get_parameter("count_goal").get_parameter_value().get<double>();
		options.starting_size = this->get_parameter("starting_size").get_parameter_value().get<int>();
			// Ground clustering
		options.ground_threshold_deg = this->get_parameter("ground_threshold_deg").get_parameter_value().get<double>();
		options.wall_threshold_deg = this->get_parameter("wall_threshold_deg").get_parameter_value().get<double>();
			// Solution
		options.iterations = this->get_parameter("iterations").get_parameter_value().get<int>();
		options.huber_loss = this->get_parameter("huber_loss").get_parameter_value().get<double>();
		options.trans_bound = this->get_parameter("trans_bound").get_parameter_value().get<double>();
			// Convergence
		options.pix_threshold = this->get_parameter("pix_threshold").get_parameter_value().get<double>();
		options.trans_threshold = this->get_parameter("trans_threshold").get_parameter_value().get<double>();
		options.rot_threshold = this->get_parameter("rot_threshold").get_parameter_value().get<double>();
			// Filter
		options.filter_kd = this->get_parameter("filter_kd").get_parameter_value().get<double>();
		options.filter_pd = this->get_parameter("filter_pd").get_parameter_value().get<double>();
		options.filter_kf = this->get_parameter("filter_kf").get_parameter_value().get<double>();
		options.filter_pf = this->get_parameter("filter_pf").get_parameter_value().get<double>();
			// Output
		options.flag_save_results = this->get_parameter("flag_save_results").get_parameter_value().get<bool>();
		options.results_file_name = this->get_parameter("results_file_name").get_parameter_value().get<string>();
	}


	//------------------------------------
	// PUBLISHER
	//------------------------------------
	// Publish Pose with Covariance
	void publish_odometry(const std_msgs::msg::Header & header, const Eigen::Matrix4f & pose, const Eigen::MatrixXf & cov)
	{
		// Create message PoseWithCovarianceStamped
		auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
			// Set header
		message.header = header;
		message.header.frame_id = "world";
			// Set pose
		Eigen::Quaternionf quat(pose.block<3,3>(0,0));
		message.pose.pose.position.x = pose(0,3);
		message.pose.pose.position.y = pose(1,3);
		message.pose.pose.position.z = pose(2,3);
		message.pose.pose.orientation.x = quat.x();
		message.pose.pose.orientation.y = quat.y();
		message.pose.pose.orientation.z = quat.z();
		message.pose.pose.orientation.w = quat.w();
			// Set covariance
		Map<Matrix<double, 6, 6, RowMajor>>(begin(message.pose.covariance)) = cov.cast<double>();

		// Create odometry message
		auto odom_msg = nav_msgs::msg::Odometry();
			// Set header
		odom_msg.header = header;
		odom_msg.header.frame_id = "world";
		odom_msg.child_frame_id = "lidar";
			// Set pose as above
		odom_msg.pose = message.pose;

		// Publish
		pose_pub_->publish(message);
		odom_pub_->publish(odom_msg);
	}

	/// Publish Patch
	void publish_patches(const std_msgs::msg::Header & header, const SizedData & szdata){ ///Se puede mejorar inicializando los vectores con tamaño conocido e insertando
		/// Create the patch message
		auto patch = gndlo::msg::Patches();
			/// Set header
		patch.header = header;
		patch.header.frame_id = "world";
			/// Set centers
		std::vector<float> centers;
		for (const auto& center : szdata.centers){
			centers.push_back(center(0));
			centers.push_back(center(1));
			centers.push_back(center(2));
		}
		patch.centers = centers;
			/// Set normals
		std::vector<float> normals;
		for (const auto& normal : szdata.normals){
			normals.push_back(normal(0));
			normals.push_back(normal(1));
			normals.push_back(normal(2));
		}
		patch.normals = normals;
			/// Set covars
		std::vector<float> covars;
		for (const auto& covar : szdata.covars) {
			for (int i = 0; i < covar.rows(); ++i) {
				for (int j = 0; j < covar.cols(); ++j) {
					covars.push_back(covar(i, j));
				}
			}
    	}
		patch.covars = covars;
			/// Set weights
		patch.weights = szdata.fitnesses;
			/// Set labels
		patch.labels = szdata.labels;
			/// Set sizes
		patch.sizes = szdata.sizes;
			/// Set pixels
		std::vector<int> pixels;
		for (const auto& pixel : szdata.px0){
			pixels.push_back(pixel(0));
			pixels.push_back(pixel(1));
		}
		patch.pixels = pixels;

		///Publish the patch
		ptch_pub_->publish(patch);
	}

	//------------------------------------
	// IMAGE+INFO CALLBACK
	//------------------------------------
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg, const transformer::msg::Lidar3dSensorInfo::ConstSharedPtr& info_msg)
    {
		if (options.flag_verbose)
			RCLCPP_INFO(this->get_logger(), "Pair of messages (image - info) received:");

		// Save header
		header = img_msg->header;

		// Save image
        cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img_msg);
        cv::cv2eigen(cv_img->image, input_img);
		

		// Save Sensor information
		sensor.flag_linear_vert = info_msg->is_theta_linear;
		sensor.rows = info_msg->rows;
		sensor.cols = info_msg->cols;
	    sensor.hor_min = info_msg->phi_start;
		sensor.hor_max = info_msg->phi_end;
		sensor.ver_min = info_msg->theta_start;
		sensor.ver_max = info_msg->theta_end;
		for (int i=0; i<sensor.rows; ++i)
			sensor.ver_angles.push_back(info_msg->theta_vec[i]);

		// Initialize when first data arrives
		if (first_data)
		{
			// Initialize
			first_data = false;
			this->setSensorParameters(sensor);
			this->setDepthInput(input_img);
			this->initialize();

			// Create header in results file
			if (options.flag_save_results)
				results_file << "#time \ttx \tty \ttz \tqx \tqy \tqz \tqw\n";

		}
		// Do the normal calculations
		else
		{
			// Set input frame
			this->setDepthInput(input_img);

			// Odometry
			this->runOdometry();

			// Display execution time
			if (options.flag_verbose)
			{
				RCLCPP_INFO(this->get_logger(), "Frame: %li", this->number_frames);
				RCLCPP_INFO(this->get_logger(), "Execution time: %f ms", this->execution_time);
				RCLCPP_INFO(this->get_logger(), "Average execution time: %f ms:", this->avg_exec_time);
				RCLCPP_INFO(this->get_logger(), "\t-Flatness: %f ms", this->avg_time_bd(0));
				RCLCPP_INFO(this->get_logger(), "\t-Quadtree: %f ms", this->avg_time_bd(1));
				RCLCPP_INFO(this->get_logger(), "\t-Plane fitting: %f ms", this->avg_time_bd(2));
				RCLCPP_INFO(this->get_logger(), "\t-Labeling: %f ms", this->avg_time_bd(3));
				RCLCPP_INFO(this->get_logger(), "\t-Culling: %f ms", this->avg_time_bd(4));
				RCLCPP_INFO(this->get_logger(), "\t-Ground clustering: %f ms", this->avg_time_bd(5));
				RCLCPP_INFO(this->get_logger(), "\t-Ground alignment: %f ms", this->avg_time_bd(6));
				RCLCPP_INFO(this->get_logger(), "\t-Point matching: %f ms", this->avg_time_bd(7));
				RCLCPP_INFO(this->get_logger(), "\t-Motion estimation: %f ms", this->avg_time_bd(8));
				RCLCPP_INFO(this->get_logger(), "\t-Iterations: %f ms", this->avg_time_bd(9));
				RCLCPP_INFO(this->get_logger(), "\t-Motion filter: %f ms\n", this->avg_time_bd(10));
			}
		}

		// Get results
		this->getPose(odom_pose);
		this->getCovariance(odom_cov);
		this->getPatches(szdata);

		// Publish the results
		publish_odometry(header, odom_pose, odom_cov);

		/// Publish the patches
		publish_patches(header, szdata);

		// Save results to file
		if (options.flag_save_results)
		{
			Quaternionf q(odom_pose.block<3,3>(0,0));
			rclcpp::Time timestamp(header.stamp);
			char timestr[20];
			snprintf(timestr, sizeof(timestr), "%.9f", timestamp.seconds());
			results_file << timestr << " "
			 			 << odom_pose(0,3) << " " << odom_pose(1,3) << " " << odom_pose(2,3) << " "
			 			 << q.vec()(0) << " " << q.vec()(1) << " " << q.vec()(2) << " " << q.w()
						 << endl;
		}
    }


	//------------------------------------
	// VARIABLES
	//------------------------------------
	// Variables
	string topic = "/ouster";
	bool first_data = true;
	Eigen::MatrixXf input_img;
	Eigen::Matrix4f odom_pose;
	Eigen::MatrixXf odom_cov;
	GNDLO_Lidar::SensorInfo sensor;
	///
	SizedData szdata; // This is the actual patch
	
	// Output results file
	ofstream results_file;

	/// Reconfigure parameters
	std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
  	std::shared_ptr<rclcpp::ParameterEventCallbackHandle> handle;
  	///
	gndlo::msg::Patches parche = gndlo::msg::Patches();

	// Declare publishers
	std_msgs::msg::Header header;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
	rclcpp::Publisher<gndlo::msg::Patches>::SharedPtr ptch_pub_;

	// Declare subscriptions and synchronizer
	message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
	message_filters::Subscriber<transformer::msg::Lidar3dSensorInfo> info_sub_;
	std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, transformer::msg::Lidar3dSensorInfo>> sync_;
	
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
		cout << "GNDLO Odometry node: READY." << endl;
	    rclcpp::init(num_arg, argv);
		rclcpp::spin(std::make_shared<GNDLO_Node>());
		rclcpp::shutdown();

		cout << "GNDLO Odometry node: SHUTTING DOWN" << endl;

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

