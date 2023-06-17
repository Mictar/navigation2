#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;

class Nav: public rclcpp::Node
{
	private:
		rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub;

		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subs_rviz;

		void rvizCallback(const geometry_msgs::msg::PoseStamped&);

	public:
		Nav(std::string name);
};

Nav::Nav(std::string name="NavRviz"): Node(name)
{
	RCLCPP_INFO( this -> get_logger(), "Demarrage du navigateur par rviz...");

	goal_pub = this -> create_publisher<geometry_msgs::msg::Point>(
			"goal",
			10);
	subs_rviz = this -> create_subscription<geometry_msgs::msg::PoseStamped>(
			"goal_pose",
			10,
			std::bind(&Nav::rvizCallback, this, _1));
}

void Nav::rvizCallback(const geometry_msgs::msg::PoseStamped& msg)
{
	auto pose_goal = geometry_msgs::msg::Point();

	pose_goal.x = msg.pose.position.x;
	pose_goal.y = msg.pose.position.y;

	goal_pub -> publish(pose_goal);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin( std::make_shared<Nav>() );
	rclcpp::shutdown();

	return 0;
}
