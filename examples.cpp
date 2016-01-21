//Broadcaster

tf::TransformBroadcaster();

void sendTransform(const StampedTransform & transform);
void sendTransform(const geometry_msgs::TransformStamped & transform);


//Listener

bool tf::TransformListener::canTransform (
  const std::string &target_frame,
  const std::string &source_frame,
  const ros::Time &time,
  std::string *error_msg=NULL) const;

bool tf::TransformListener::waitForTransform (
  const std::string &target_frame,
  const std::string &source_frame,
  const ros::Time &time,
  const ros::Duration &timeout,
  const ros::Duration &polling_sleep_duration=ros::Duration(0.01),
  std::string *error_msg=NULL) const;

void tf::TransformListener::lookupTransform (
  const std::string &target_frame,
  const std::string &source_frame,
  const ros::Time &time,
  StampedTransform &transform) const;

void tf::TransformListener::lookupTransform (
  const std::string &target_frame,
  const ros::Time &target_time,
  const std::string &source_frame,
  const ros::Time &source_time,
  const std::string &fixed_frame,
  StampedTransform &transform) const;


//Examples from swarm0

//tf_broadcaster_node.cpp
  // ...
  static tf::TransformBroadcaster br;
  ros::Time const t = ros::Time::now();
  {
    geometry_msgs::TransformStamped base_footprint_tf;
    base_footprint_tf.header.stamp = t;
    base_footprint_tf.header.frame_id = "base_footprint";
    base_footprint_tf.child_frame_id = "base_link";
    base_footprint_tf.transform.translation.x = 0.0;
    base_footprint_tf.transform.translation.y = 0.0;
    base_footprint_tf.transform.translation.z = 0.235;
    base_footprint_tf.transform.rotation.x = 0.0;
    base_footprint_tf.transform.rotation.y = 0.0;
    base_footprint_tf.transform.rotation.z = 0.0;
    base_footprint_tf.transform.rotation.w = 1.0;
    
    br.sendTransform(base_footprint_tf);
  }
  // ...
  //sonar3
  {
    geometry_msgs::Quaternion quat;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0., 0., M_PI/4.), quat);
    geometry_msgs::TransformStamped base_link_tf;
    base_link_tf.header.stamp = t;
    base_link_tf.header.frame_id = "base_link";
    base_link_tf.child_frame_id = "sonar3";
    base_link_tf.transform.translation.x = 0.17;
    base_link_tf.transform.translation.y = 0.09;
    base_link_tf.transform.translation.z = -0.13;
    base_link_tf.transform.rotation = quat;

    br.sendTransform(base_link_tf);
  }
  // ...

//grid_mapping/sonar.cpp
  bool SonarState::update_pose()
  {
    tf::StampedTransform transform;
    try
    {
      tf_listener_.lookupTransform(g_frame_, s_frame_,
        ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN_STREAM("No transform between "<<g_frame_<<" and "<<s_frame_);
      return false;
    }
    //update poseition of sonar
    cv::Point2d pose(transform.getOrigin().x(), transform.getOrigin().y());
    double heading = tf::getYaw(transform.getRotation());
    sonar_ptr_->update_heading_and_pose(heading, pose);
    return true;
  }

//grid_mapping/sonar.h
class SonarState
{
public:
  SonarState(
    std::shared_ptr<Sonar> sonar_ptr,
    std::string s_topic,
    std::string g_frame,
    std::string s_frame):
  sonar_ptr_(sonar_ptr),
  s_frame_(s_frame),
  g_frame_(g_frame),
  upd_flg_(false)
  {
    s_sub_.subscribe(nh_, s_topic, 20);
    tf_filter_ptr_ = std::shared_ptr<tf::MessageFilter<sensor_msgs::Range> >(
      new tf::MessageFilter<sensor_msgs::Range>(s_sub_, tf_listener_, g_frame, 20)
      );
    tf_filter_ptr_->registerCallback( boost::bind(&SonarState::update_state, this, _1) );
  }

  // ...

private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::Range> s_sub_;
  tf::TransformListener tf_listener_;
  std::shared_ptr<tf::MessageFilter<sensor_msgs::Range> > tf_filter_ptr_;
  // ...
};