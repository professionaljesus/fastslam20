#include "ros/ros.h"
#include "ros/package.h"
#include "ros/console.h"
#include "particle.h"


//Message Deps
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "lfs_msgs/cone.h"
#include "lfs_msgs/cone_classes.h"
#include "lfs_msgs/cone_array.h"
#include "lfs_msgs/velocity_estimation.h"

#include "lfs_msgs/world_map.h"
#include "nav_msgs/Odometry.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#include "sbg_driver/SbgEkfNav.h"

//Visualizers and debugging
#include "visualizer.cpp"

using namespace std;
using namespace Eigen;

#define NUM_PARTICLES 1
#define NUM_RESAMPLE 1
#define DEBUG_MESSAGE 1


//Publishers
ros::Publisher world_map_publisher;
ros::Publisher local_map_publisher;


vector<Particle> particles;

Vector3d latest_vel; //debug purposes

bool loop_closing;

Matrix3d Particle::Pt;
Matrix3d Particle::Ptinv;
Vector3d Particle::transform_std;

Matrix2d Particle::Rt;
double Particle::association_threshold;
double Particle::observation_quote_threshold;
vector<double> Particle::angle_range;
vector<double> Particle::range_range;
int Landmark::id_counter;

random_device rd{};
mt19937 gen{rd()};
//TODO: is this a good range for the added noise to additional particles?
normal_distribution<double> gauss{0, 1};

list<double> timestamps_prediction;
list<double> timestamps_observation;

/*
   Function publishing map to the /slam/world_map topic
 */
void publish_map(){

    lfs_msgs::world_map map;
    Particle p = particles.front();
    Vector3d transform = p.poses.front();

    map.pose_x = transform.x();
    map.pose_y = transform.y();
    map.pose_phi = transform.z();

    list<Landmark*> *cones = &p.local_landmarks;

    if(p.loop_closed)
    {
        cones = &p.global_landmarks;
    }
    
    for(Landmark* cone : *cones)
    {
        if(cone->Sigma.sum() > 0.01 || (cone->mu - transform.head(2)).norm() > 15)
        {
            continue;
        }

        map.cone_x.push_back(cone->mu.x());
        map.cone_y.push_back(cone->mu.y());
        Vector4f classes_vec = cone->color_probability.cast<float>();
        lfs_msgs::cone_classes classes;
        classes.cone_classes = {classes_vec[0], classes_vec[1], classes_vec[2], classes_vec[3]};
        map.cone_class_probs.push_back(classes);
    }

    world_map_publisher.publish(map);

}



/*
   function resampling the particles based on weight
   @return vector with the resampled particles
 */
vector<Particle> resample_particles()
{
    sort(particles.begin(), particles.end(), greater<Particle>());
    vector<Particle> new_particles;

    double total_weight = 0.0;
    for(int i = 0; i < NUM_RESAMPLE; i++)
    {
        total_weight += particles[i].weight;
        new_particles.push_back(particles[i]);
    }

    for(int i = 0; i < NUM_RESAMPLE; i++)
    {
        double quota = 1.0/(double)NUM_RESAMPLE;
        if(total_weight > 0)
        {
            quota = particles[i].weight/total_weight;
        }

        int to_create = ceil(quota*(NUM_PARTICLES - NUM_RESAMPLE));
        to_create = min(NUM_PARTICLES - (int)new_particles.size(), to_create);

        for(int j = 0; j < to_create; j++) {
            new_particles.emplace_back(particles[i]);
        }

    }

    return new_particles;
}


/*
 * Main callback for all observations made by the sensors. This will update the everything the sensor is allowed, and call resampling of the particles.
 * @param valid_cones vector of the observations made by the sensor
 @param color_observation the probability of each of the 4 colors observed for each cone
 @param timestamp timestamp the observation was made
 @param sensor the sensor which made the observation
 */

chrono::time_point<chrono::high_resolution_clock> start;
void process_observations(const vector<Vector2d>& valid_cones, const vector<Vector4d>& color_observations, const double& timestamp)
{

    int downtime = 0;
    if(DEBUG_MESSAGE)
    {
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        downtime = duration.count();
        start = stop;
    }

    double delta_t = 0;
    if(timestamps_observation.size() > 0)
    {
        delta_t = timestamp - timestamps_observation.front();
    }
    timestamps_observation.push_front(timestamp);

    // Loop through all particles and process measurement
    for(size_t i = 0; i < particles.size(); i++)
    {
        Vector3d pose(0,0,0);
        if(particles[i].poses.size() > 0)
        {
            pose = particles[i].poses.front();
            if(i > 0)
            {
                //Add sprinkle of noise to the additional particles
                Vector3d gauss_samples(gauss(gen), gauss(gen), gauss(gen));
                pose += Particle::transform_std.cwiseProduct(gauss_samples);
                pose[2] = remainder(pose[2], 2 * M_PI);
            }
        }
        particles[i].process_observations(valid_cones, color_observations, pose);
    }


    particles = resample_particles();

    if(loop_closing && !particles.front().loop_closed && particles.front().search_for_loop_closure)
    {
        bool closed = particles.front().pattern_matching();
        if(closed)
        {
            particles.front().loop_closing();
            particles.front().search_for_loop_closure = false;
        }
    }

    publish_map();
    publish_visualization(particles);

    if(DEBUG_MESSAGE){
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        list<Vector3d>::iterator it = particles.front().poses.begin();
        Vector2d pos = it->head(2);
        it++;
        Vector2d old_pos = it->head(2);

        double est_speed = 0;
        if(delta_t > 0)
        {
            est_speed = (pos - old_pos).norm()/delta_t;
        }

        string debug_string = "Downtime: " + to_string(downtime) + " \u03BCs\n" +
            "Execution time: " + to_string(duration.count()) + " \u03BCs\n" +
            "Particles: " + to_string(particles.size()) + "\n" +
            " Left Home: " + to_string(particles.front().left_home) + "\n" +
            " Pattern Matching: " + to_string(particles.front().search_for_loop_closure)  + "\n" +
            " Loop Closed: " + to_string(particles.front().loop_closed)  + "\n" +
            "Local Landmarks: " + to_string(particles.front().local_landmarks.size()) + "\n" +
            "Global Landmarks: " + to_string(particles.front().global_landmarks.size()) + "\n" +
            "Total Landmarks: " + to_string(particles.front().global_landmarks.size() + particles.front().local_landmarks.size()) + "\n" +
            "Observations: " + to_string(valid_cones.size()) + "\n" +
            "Latest Input Velocities:\n \t magnitude: "+to_string(sqrt(pow(latest_vel[0],2)+pow(latest_vel[1],2)))+" m/s,\n \t x:" + to_string(latest_vel[0]) + " m/s,\n \t y: " + to_string(latest_vel[1]) + " m/s,\n \t angular: "+to_string(latest_vel[2]) + "rad/s \n" +
            "Estimated Velocity (based on position): " + to_string(est_speed) + " m/s (" + to_string(est_speed*3.6) + " km/h)" + "\n" +
            "delta_t: " + to_string(delta_t) + "\n";

        cout << debug_string << endl;

        Particle p = particles.front();


        string json_output = "{\"timestamps\":  [";
        for(double timestamp : timestamps_observation)
        {
            json_output += to_string(timestamp) + ",";
        }
        json_output.pop_back();
        json_output += "], \n \"poses\" : [";


        for(Vector3d pose : p.poses)
        {
           json_output += "[";
           json_output += to_string(pose.x()) + ",";
           json_output += to_string(pose.y()) + ",";
           json_output += to_string(pose.z()) + "],";

        }

        json_output.pop_back();
        json_output += "], \n \"cones\" : [";

        for(Landmark* cone : p.global_landmarks)
        {
            json_output += "[";
            json_output += to_string(cone->mu.x()) + ",";
            json_output += to_string(cone->mu.y()) + ",";
            json_output += to_string(cone->color) + "],";
        }

        for(Landmark* cone : p.local_landmarks)
        {
            json_output += "[";
            json_output += to_string(cone->mu.x()) + ",";
            json_output += to_string(cone->mu.y()) + ",";
            json_output += to_string(cone->color) + "],";
        }

        json_output.pop_back();
        json_output += "]\n}";
        //cout << json_output << endl;

        start = stop;
    }
}

/*
 * Publish local map/observation as slam message (word_map.msg) to /slam/local_map topic
 */
void publish_valid_cones_as_map(vector<Vector2d> valid_cones, vector<Vector4d> color_observations, double timestamp){
    lfs_msgs::world_map local_map;
    Particle p = particles.front();
    Vector3d transform = p.poses.front();

    local_map.pose_x = transform.x();
    local_map.pose_y = transform.y();
    local_map.pose_phi = transform.z();

    for(size_t i = 0; i < valid_cones.size(); i++)
    {

        // local_map.cone_x.push_back(valid_cones[i].x());
        // local_map.cone_y.push_back(valid_cones[i].y());
        auto cone_global = to_global(valid_cones[i], transform);
        local_map.cone_x.push_back(cone_global.x());
        local_map.cone_y.push_back(cone_global.y());

        lfs_msgs::cone_classes classes;
        classes.cone_classes = {(float) color_observations[i][0], (float) color_observations[i][1], (float) color_observations[i][2], (float) color_observations[i][3]};
        local_map.cone_class_probs.push_back(classes);
    }

    local_map_publisher.publish(local_map);
}

/*
 * Fusion calllback
 */
void fusion_callback(const lfs_msgs::cone_array::ConstPtr& msg){
    // Get measurement timestamp
    double timestamp = msg->header.stamp.toSec();
    vector<lfs_msgs::cone> cones = msg->cones;

    // Check if measurements are in lidar range.
    vector<Vector2d> valid_cones;

    // color_observations keep track of the likelihood that a cone is a certain color.
    vector<Vector4d> color_observations;

    for(size_t i = 0; i < cones.size(); i++)
    {

        double x = cones[i].point.x;
        double y = cones[i].point.y;
        Vector2d cone(x,y);

        double angle = atan2(cone.y(), cone.x());

        if(cone.norm() < Particle::range_range[0] || cone.norm() > Particle::range_range[1] + 2 || angle < Particle::angle_range[0] || angle > Particle::angle_range[1])
        {
            continue;
        }

        //points_to_visualize.push_back(to_global(cone, particles.front().poses.front()));

        valid_cones.push_back(cone);

        Vector4d color_probs;
        lfs_msgs::cone_classes classes = cones[i].cone_class_prob;
        color_probs << classes.cone_classes[0], classes.cone_classes[1], classes.cone_classes[2], classes.cone_classes[3];
        color_observations.push_back(color_probs);
    }

    process_observations(valid_cones, color_observations, timestamp);
    
    // Also publish LOCAL/OBSERVATION as map msg
    //publish_valid_cones_as_map(valid_cones, color_observations, timestamp);
}
/*
 * Handle the message from the velocity estimation
 * make a small prediction
 */
void vel_callback(const lfs_msgs::velocity_estimation msg){
        //get the time since last prediction
        double timestamp = msg.header.stamp.toSec();
        double delta_t = 0;
        if(timestamps_prediction.size() > 0){
                delta_t = timestamp - timestamps_prediction.front();
        }
        timestamps_prediction.push_front(timestamp);

        Vector3d vel(msg.x_dot, msg.y_dot, msg.phi_dot);
        latest_vel = vel;

        // Loop through all particles and make small prediction
        for(size_t i = 0; i < particles.size(); i++){
            if(particles[i].poses.size() > 0){
                // velocities from velocity estimation are with respect to the car's refferance frame. We need to convert to global coordinate system
                // vel[0] is therefore v*sin(phi), we need v*sin(phi+theta)
                // note: addition rules for trigonometry
                Vector3d& pose = particles[i].poses.front();
                float th = pose.z();
                pose[0] += (vel[0]*cos(th)-vel[1]*sin(th))*delta_t;
                pose[1] += (vel[0]*sin(th)-vel[1]*cos(th))*delta_t;
                pose[2] += (vel[2])*delta_t;
                pose[2] = remainder(pose[2], 2 * M_PI);
            }
        }
}

bool load_param(string param, double &value, ros::NodeHandle &n)
{
    bool result = n.getParam(param, value);
    if(result)
    {
        //ROS_INFO("%s loaded from parameter server to %f",param, value);
    }
    else
    {
        //ROS_INFO(param + " using default value of %f",param, value);
    }

    return result;
}

bool load_param_vector(string param, vector<double> &value, ros::NodeHandle &n)
{
    bool result = n.getParam(param, value);
    if(result)
    {
        //ROS_INFO("%s loaded from parameter server" ,param);
    }
    else
    {
        //ROS_INFO("%s using default value" ,param);
    }

    return result;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "slamming2");
    ros::NodeHandle n;

    Landmark::id_counter = 0;

    vector<double> P_temp = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
    load_param_vector("/slam/P", P_temp, n);
    Particle::Pt = Matrix3d(P_temp.data());
    cout << Particle::Pt << endl;

    Particle::transform_std = Particle::Pt.diagonal().cwiseSqrt();
    Particle::Ptinv = Particle::Pt.inverse();

    loop_closing = true;
    n.getParam("/slam/loop_closing", loop_closing);

    Particle::association_threshold = 0.01;
    load_param("/slam/association_threshold", Particle::association_threshold, n);

    Particle::observation_quote_threshold = 0.3;
    load_param("/slam/observation_quote_threshold", Particle::observation_quote_threshold, n);

    vector<double> R_temp = {0.03, 0.0, 0.0, 0.03};
    load_param_vector("/slam/R", R_temp, n);
    Particle::Rt = Matrix2d(R_temp.data());
    cout << Particle::Rt << endl;

    Particle::angle_range = {-1.5, 1.5};
    Particle::range_range = {0.3, 10};
    load_param_vector("/slam/angle_range", Particle::angle_range, n);
    load_param_vector("/slam/range_range", Particle::range_range, n);


    for (size_t i = 0; i < NUM_PARTICLES; i++)
    {
        Particle temp_p;
        particles.push_back(temp_p);
    }

    world_map_publisher = n.advertise<lfs_msgs::world_map>("/slam/world_map", 1);
    local_map_publisher = n.advertise<lfs_msgs::world_map>("/slam/local_map", 1);

    vis_pub = n.advertise<visualization_msgs::MarkerArray>("/slam/visualization", 1);

    ros::Subscriber fusion_subscriber = n.subscribe("/sensor_fusion/cones", 1, fusion_callback);

    ros::Subscriber vel_subscriber = n.subscribe("/velocity_estimation/state", 1, vel_callback);
    ros::spin();
    return 0;
}
