#ifndef VIZ
#define VIZ
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "utils.h"

using namespace std;
using namespace Eigen;


ros::Publisher vis_pub;
list<pair<Vector3d, Vector3f>> points_to_visualize;
int marker_id = 0;

visualization_msgs::Marker line_list_marker(vector<pair<Vector2d, Vector2d>> lines, vector<float> rgb)
{

	visualization_msgs::Marker line_list;
	line_list.ns = "debug_lines";
	line_list.header.frame_id = "map";
	line_list.id = marker_id++;
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.1;
  	
	// list is red
	line_list.color.r = rgb[0];
	line_list.color.g = rgb[1];
	line_list.color.b = rgb[2];
	line_list.color.a = 1.0;


	for(auto line : lines) {

		geometry_msgs::Point p;

		p.x = line.first[0];
		p.y = line.first[1];
		p.z = 0;
		line_list.points.push_back(p);

		p.x = line.second[0];
		p.y = line.second[1];

		line_list.points.push_back(p);

	}


	return line_list;

}

visualization_msgs::Marker arrow_marker(float x, float y, float theta, float green = 0.0)
{
	visualization_msgs::Marker arrow;
	arrow.header.frame_id = "map";
	arrow.header.stamp = ros::Time();
	arrow.ns = "cars";
	arrow.id = marker_id++;
	arrow.type = visualization_msgs::Marker::ARROW;
	arrow.action = visualization_msgs::Marker::ADD;
	arrow.pose.position.x = x;
	arrow.pose.position.y = y;
	arrow.pose.position.z = 0;
	Quaternionf q = AngleAxisf(0, Vector3f::UnitX()) * AngleAxisf(0, Vector3f::UnitY()) * AngleAxisf(theta, Vector3f::UnitZ());
	arrow.pose.orientation.x = q.x();
	arrow.pose.orientation.y = q.y();
	arrow.pose.orientation.z = q.z();
	arrow.pose.orientation.w = q.w();

	arrow.scale.x = 0.2;
	arrow.scale.y = 0.05;
	arrow.scale.z = 0.05;
	arrow.color.a = 0.8; // Don't forget to set the alpha!
	arrow.color.r = 1.0;
	arrow.color.g = green;
	arrow.color.b = 1.0;

	return arrow;
}

visualization_msgs::Marker sphere_marker(float x, float y, int cone_color, Vector2d scale, float alpha = 0.5, string ns = "landmarks", float z = 0.0)
{

	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = ns;
	marker.id = marker_id++;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = scale.x();
	marker.scale.y = scale.y();
	marker.scale.z = 0.2;
	marker.color.a = alpha; // Don't forget to set the alpha!

	if(cone_color == -1){
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
	}else if(cone_color == 0){
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
	}else if(cone_color == 1){
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
	}else if(cone_color == 2){
		marker.color.r = 224.0/255.0;
		marker.color.g = 45.0/255.0;
		marker.color.b = 18.0/255.0;
	}else if(cone_color == 3){
		marker.color.r = 224.0/255.0;
		marker.color.g = 45.0/255.0;
		marker.color.b = 18.0/255.0;
	}

	return marker;
}

visualization_msgs::Marker debug_marker(Vector3d p, Vector3f rgb)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "debug";
	marker.id = marker_id++;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = p.x();
	marker.pose.position.y = p.y();
	marker.pose.position.z = p.z();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.2;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = rgb[0];
    marker.color.g = rgb[1];
    marker.color.b = rgb[2];
	return marker;

}

visualization_msgs::Marker text_marker(double x, double y, double z, string text, double scale, string ns)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.text = text;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0;
    marker.scale.y = 0;
    marker.scale.z = scale;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);
    
    return marker;
}



void publish_visualization(const vector<Particle> &particles)
{
	
	visualization_msgs::MarkerArray del_markers;
	visualization_msgs::Marker del_marker;
	del_marker.header.frame_id = "map";
	del_marker.header.stamp = ros::Time();
	del_marker.action = 3;
	del_markers.markers.push_back(del_marker);

	vis_pub.publish(del_markers);
	
	marker_id = 0;
	visualization_msgs::MarkerArray markers;
	Particle m = particles.front();
	vector<pair<Vector2d, Vector2d>> lines = m.associations;

	Vector3d mpos = m.poses.front();

	visualization_msgs::Marker main_car = arrow_marker(mpos[0], mpos[1], mpos[2]);

	//only if using a MESH_RESOURCE marker type:
	markers.markers.push_back(main_car);

	for(const Particle p : particles) 
	{
		Vector3d pos = p.poses.front();
		visualization_msgs::Marker car = arrow_marker(pos[0], pos[1], pos[2], 1.0);
		markers.markers.push_back(car);
	}

	Landmark* cone;
	if(m.global_landmarks.size() > 0)
	{
		std::list<Landmark*> &cones = m.global_landmarks;

		for(auto cone_iter = cones.begin(); cone_iter != cones.end(); cone_iter++) 
        {
			cone = *cone_iter;
			int cone_color = cone->color;

			Vector2d scale = 2*(5.991464547107983*cone->Sigma.diagonal()).cwiseSqrt();
			visualization_msgs::Marker marker = sphere_marker(cone->mu.x(), cone->mu.y(), cone_color, scale, 0.5, "global_landmarks");
			markers.markers.push_back(marker);
            if(cone_color == -1)
            {
			    visualization_msgs::Marker est_marker = sphere_marker(cone->mu.x(), cone->mu.y(), cone->color_est, scale, 0.5, "global_landmarks", 0.5);
			    markers.markers.push_back(est_marker);
            }
			visualization_msgs::Marker txt_marker = text_marker(cone->mu.x(), cone->mu.y(), 0.2, to_string(cone->Sigma.sum()), 0.2, "global_det(Sigma)");
			markers.markers.push_back(txt_marker);
			txt_marker = text_marker(cone->mu.x(), cone->mu.y(), 0.3, to_string(cone->id), 0.2, "ids");
			markers.markers.push_back(txt_marker);
		}
	}
    vector<pair<Vector2d, Vector2d>> loop_close_assoc;
	if(m.local_landmarks.size() > 0)
	{
		std::list<Landmark*> &cones = m.local_landmarks;

		for(auto cone_iter = cones.begin(); cone_iter != cones.end(); cone_iter++) 
        {
			cone = *cone_iter;
            
			visualization_msgs::Marker txt_marker = text_marker(cone->mu.x(), cone->mu.y(), 0.2, to_string(cone->Sigma.sum()), 0.2, "local_det(Sigma)");
			markers.markers.push_back(txt_marker);
			txt_marker = text_marker(cone->mu.x(), cone->mu.y(), 0.3, to_string(cone->id), 0.2, "ids");
			markers.markers.push_back(txt_marker);
            if(cone->assoc_landmark != NULL)
            {
                loop_close_assoc.emplace_back(cone->mu, (cone->assoc_landmark)->mu);
			    txt_marker = text_marker(cone->mu.x(), cone->mu.y(), 0.5, "assoc: " + to_string((cone->assoc_landmark)->id), 0.2, "assoc_ids");
			    markers.markers.push_back(txt_marker);
            }
            
			int cone_color = cone->color;

			Vector2d scale = 2*(5.991464547107983*cone->Sigma.diagonal()).cwiseSqrt();
			visualization_msgs::Marker marker = sphere_marker(cone->mu.x(), cone->mu.y(), cone_color, scale, 0.8, "local_landmarks");
			markers.markers.push_back(marker);
            if(cone_color == -1)
            {
			    visualization_msgs::Marker est_marker = sphere_marker(cone->mu.x(), cone->mu.y(), cone->color_est, scale, 0.5, "local_landmarks", 0.5);
			    markers.markers.push_back(est_marker);
            }
		}
	}

    vector<float> rgb = {1.0, 0.0, 0.0};
    if (loop_close_assoc.size() > 0)
    {
        markers.markers.push_back(line_list_marker(loop_close_assoc, rgb));
    }

	if (lines.size() > 0)
	{
        rgb[0] = 0.0;
        rgb[2] = 1.0;
		markers.markers.push_back(line_list_marker(lines,rgb));
	}

	for (const pair<Vector3d, Vector3f> &m : points_to_visualize)
	{
		markers.markers.push_back(debug_marker(m.first, m.second));
	}
	points_to_visualize.clear();

	vis_pub.publish(markers);
}
#endif
