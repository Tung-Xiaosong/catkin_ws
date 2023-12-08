#ifndef PROJECT_PATH_RVIZ_PLUGIN_H
#define PROJECT_PATH_RVIZ_PLUGIN_H

# include <ros/ros.h>
# include <rviz/panel.h>

#include <QHBoxLayout>
#include <QPushButton>
#include <QString>

#include <random_numbers/random_numbers.h>

#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>

namespace follow_plugin {
    class PathRvizPlugin : public rviz::Panel {
    Q_OBJECT
    public:
        PathRvizPlugin(QWidget *parent = 0);

		~PathRvizPlugin();

        virtual void load(const rviz::Config &config);

        virtual void save(rviz::Config config) const;

    protected Q_SLOTS:

        void start_record_callback();

        void stop_record_callback();

        void start_task_callback();

        void stop_task_callback();

		void cancel_task_callback();

    private:
        random_numbers::RandomNumberGenerator generator_;
        ros::NodeHandle nh_;
        ros::ServiceClient start_record_client_, stop_record_client_,load_path_client_;
        ros::Publisher start_task_pub_, cancel_task_pub_;

		ros::Subscriber path_subscribe_,status_subscribe_;

		geometry_msgs::PoseStamped start_goal_,end_goal_;

		void receive_path_callback(const nav_msgs::Path::ConstPtr &msg);
		void result_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg);

		bool isStart_;

		bool task_,cancelTask_;

		nav_msgs::Path path_;
 		

        QPushButton *start_record_button_;
        QPushButton *stop_record_button_;
        QPushButton *load_task_button_;
        QPushButton *start_task_button_;

        QPushButton *cancel_task_button_;
    };
}


#endif //PROJECT_PATH_RVIZ_PLUGIN_H
