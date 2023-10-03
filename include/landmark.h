#ifndef LANDMARK_H
#define LANDMARK_H

#include "utils.h"

class Landmark{

	public:
        static int id_counter;
        int id;
        bool removed;
        int color;
        int color_est; 
		Eigen::Vector2d mu;	// Position mean
		Eigen::Matrix2d Sigma;	// Position covariance matrix
		Eigen::Vector4d color_probability;	// 4 possible cone colors, these are probabilties of each
        std::map<Landmark*, int> landmark_buddies;

		int n_obs;// Number of times the landmarks been observed.
        int n_not_obs; // Number of times the landmark should have been observered but is not
        size_t pose_index;
        Landmark* assoc_landmark;
		
		/**
			Constructor 
			@param particle_pose pose of the particle that saw the landmark
			@param z the observation in polar coordinates
			@param Rt the sensor error covariance

		*/
		Landmark(const Eigen::Vector3d& particle_pose, const Eigen::Vector2d& z, const Eigen::Matrix2d& Rt, const size_t& pose_index);

		/**
			The EKF update for the landmark.
			@param zdiff error between predicted and real observation
			@param Qinv precalculated inverse of the Q matrix.
			@param G_theta jacobian with respect to the landmarks position

		*/
		void ekf_update(const Eigen::Vector2d& zdiff, const Eigen::Matrix2d& Qinv,  const Eigen::Matrix2d& G_theta);
	
		/**
			The color update for the landmark
			@param color_prob probability vector of the landmark color
		*/	
		void color_update(const Eigen::Vector4d& color_prob);
		
		/**
			Does the landmark have a color?
			@return does it?

		*/	
		bool has_color();
        
};

#endif
