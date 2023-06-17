#include <memory>
#include <functional>
#include <cmath>
#include <robot_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <algorithm>

using std::placeholders::_1;

class ObsDetect: public rclcpp::Node
{
	private:
		rclcpp::Publisher<robot_msgs::msg::PoseArray>::SharedPtr pub_pose;

		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subs;

		void laserScanCallback(const sensor_msgs::msg::LaserScan& msg);
		
	public:
		ObsDetect(std::string, std::string);
};
ObsDetect::ObsDetect(std::string node_name="ObsDetect", 
		     std::string laserscan_topic="/scan"): 
			Node(node_name)
{
	pub_pose = this -> create_publisher<robot_msgs::msg::PoseArray>("obtaclePose", 10);

	subs = this -> create_subscription<sensor_msgs::msg::LaserScan>(
			laserscan_topic,
			10,
			std::bind(&ObsDetect::laserScanCallback, this, _1)
			);

	RCLCPP_INFO( this -> get_logger(), "Demarrage du noeud detection d'obstacke");
}

void ObsDetect::laserScanCallback(const sensor_msgs::msg::LaserScan& scan)
{
	auto begin { std::begin(scan.ranges)  };
	
	auto end { std::end(scan.ranges)};
	std::cout << "[\n";

	auto inf = std::numeric_limits<float>::infinity();

	
	for( int i = 1; begin != end; ++i )
	{
		begin++;
		//auto debut = std::find(begin, end, inf);
		if( *begin == inf )
		{
			int debut = i , fin = i;
			for ( ; *begin == inf; fin++, begin++);
			std::cout << debut << " " << fin << std::endl;

		}

	}
	
	std::cout << "]\n";
	

}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin( std::make_shared<ObsDetect>()) ;
	rclcpp::shutdown();


	return 0;
}
