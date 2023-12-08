#include <tf/transform_datatypes.h>

#include <std_srvs/Empty.h>
#include <actionlib_msgs/GoalID.h>
#include <path_server/SetPathName.h>
#include <geometry_msgs/PoseStamped.h>
#include <path_rviz_plugin/path_rviz_plugin.h>
#include <nav_msgs/GetPlan.h>


namespace follow_plugin {

    PathRvizPlugin::PathRvizPlugin(QWidget *parent) : Panel(parent) {
        auto *button_layout = new QHBoxLayout;
        start_record_button_ = new QPushButton(tr("录制路径"), this);
        button_layout->addWidget(start_record_button_);
        stop_record_button_ = new QPushButton(tr("保存路径"), this);
        button_layout->addWidget(stop_record_button_);

        load_task_button_ = new QPushButton(tr("载入路径"), this);
        button_layout->addWidget(load_task_button_);

        start_task_button_ = new QPushButton(tr("开始任务"), this);
		button_layout->addWidget(start_task_button_);

		cancel_task_button_ = new QPushButton(tr("取消任务"), this);
		button_layout->addWidget(cancel_task_button_);
        
		

        setLayout(button_layout);

        stop_record_button_->setEnabled(false);
		start_task_button_->setEnabled(false);
		cancel_task_button_->setEnabled(false);

        connect(start_record_button_, SIGNAL(clicked()), this, SLOT(start_record_callback()));
        connect(stop_record_button_, SIGNAL(clicked()), this, SLOT(stop_record_callback()));
        connect(load_task_button_, SIGNAL(clicked()), this, SLOT(start_task_callback()));
        connect(start_task_button_, SIGNAL(clicked()), this, SLOT(stop_task_callback()));
		connect(cancel_task_button_, SIGNAL(clicked()), this, SLOT(cancel_task_callback()));

        start_record_client_ = nh_.serviceClient<path_server::SetPathName>("start_record_path");
        stop_record_client_ = nh_.serviceClient<std_srvs::Empty>("stop_record_path");
		load_path_client_ = nh_.serviceClient<path_server::SetPathName>("load_path_server");

        start_task_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
        cancel_task_pub_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);

		path_subscribe_ = nh_.subscribe<nav_msgs::Path>("recorded_path", 10, &PathRvizPlugin::receive_path_callback, this);

		status_subscribe_ = nh_.subscribe<move_base_msgs::MoveBaseActionResult>("move_base/result", 10, &PathRvizPlugin::result_callback, this);

        isStart_ = false;
		task_ = false;


		actionlib_msgs::GoalID msg;
        msg.stamp = ros::Time::now();
        cancel_task_pub_.publish(msg);
    }

	PathRvizPlugin::~PathRvizPlugin() {
		actionlib_msgs::GoalID msg;
        msg.stamp = ros::Time::now();
        cancel_task_pub_.publish(msg);
		ROS_INFO("goodbye");
    }

    void PathRvizPlugin::load(const rviz::Config &config) {
        Panel::load(config);
    }

    void PathRvizPlugin::save(rviz::Config config) const {
        Panel::save(config);
    }

	void PathRvizPlugin::receive_path_callback(const nav_msgs::Path::ConstPtr &msg) {

		load_task_button_->setEnabled(false);
		start_task_button_->setEnabled(true);
		path_ = *msg;
		start_goal_ = msg->poses[0];

		end_goal_ = msg->poses[msg->poses.size() - 1];
    }

	void PathRvizPlugin::result_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg) {

	static int n = 0;

	if(!task_) return;

	if(msg->status.status == 3)
	{
		if(isStart_)
		{
			start_task_pub_.publish(end_goal_);
			isStart_ = false;
        }
		else
		{
       		start_task_pub_.publish(start_goal_);
			isStart_ = true;
		}
	}
	if(msg->status.status == 2)
	{
		cancelTask_ = true;
		isStart_ = false;
		task_ = false;

		cancel_task_button_->setEnabled(false);
		start_task_button_->setEnabled(true);
	}
	
    }

	
    void PathRvizPlugin::start_record_callback() {
        path_server::SetPathName msg;
        msg.request.path_name = "default_path";
        if (!start_record_client_.call(msg)) {
            ROS_ERROR_STREAM("记录路径发生错误，请重试！");
        } else {
            stop_record_button_->setEnabled(true);
//            load_task_button_->setEnabled(false);
        }
    }

    void PathRvizPlugin::stop_record_callback() {
        std_srvs::Empty msg;
        if (!stop_record_client_.call(msg)) {
            ROS_ERROR_STREAM("保存路径发生错误，请重试！");
        } else {
            load_task_button_->setEnabled(true);
        }
    }

    void PathRvizPlugin::start_task_callback() {
        path_server::SetPathName msg;
        msg.request.path_name = "default_path";
        if (!load_path_client_.call(msg)) {
            ROS_ERROR_STREAM("载入路径发生错误，请重试！");
        } else {
              
              start_goal_ = msg.response.plan.poses[0];

 			  std::cout << start_goal_ << std::endl;

		      end_goal_ = msg.response.plan.poses[msg.response.plan.poses.size() - 1];
        }
    }

    void PathRvizPlugin::stop_task_callback() {

		if(path_.poses.size() == 0)
		{
			load_task_button_->setEnabled(true);
			return;
		}

		std::cout << start_goal_ << std::endl;		

		start_task_pub_.publish(start_goal_);
		isStart_ = true;
		task_ = true;
		start_task_button_->setEnabled(false);
		cancel_task_button_->setEnabled(true);

/*
        actionlib_msgs::GoalID msg;
        msg.stamp = ros::Time::now();
        cancel_task_pub_.publish(msg); */
    }

	void PathRvizPlugin::cancel_task_callback() {

		std::cout << "cancel task" << std::endl;		

		cancelTask_ = true;
		isStart_ = false;
		task_ = false;

        actionlib_msgs::GoalID msg;
        msg.stamp = ros::Time::now();
        cancel_task_pub_.publish(msg);

		cancel_task_button_->setEnabled(false);
		start_task_button_->setEnabled(true);
    }
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(follow_plugin::PathRvizPlugin, rviz::Panel)
