#ifndef PARTICLE_H
#define PARTICLE_H
#include "landmark.h"


#define NEW_LANDMARK_WEIGHT 0 // TODO TODO TODO

// DEBUG

class Particle{
	public:
        std::list<Eigen::Vector3d> poses;
		std::list<Landmark*> global_landmarks; // All landmarks seen by this particle
		std::list<Landmark*> local_landmarks; // Local landmarks seen by this particle

        bool loop_closed = false;
        bool left_home = false;
        bool search_for_loop_closure = false;

		double weight = 0; // Weight of the particle

		//DEBUG
		std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> associations;

		//Motion model variances
		static Eigen::Matrix3d Pt; // Motion model covariance matrix
		static Eigen::Matrix3d Ptinv;// Inverse of motion model covariance matrix
		static Eigen::Vector3d transform_std;

        //Calib
        static Eigen::Matrix2d Rt; //Sensor covariance matrix
        static double association_threshold; 
        static double observation_quote_threshold; 
        static std::vector<double> angle_range;
        static std::vector<double> range_range;

		
		bool operator>(const Particle& other) const;

		bool operator<(const Particle& other) const;

        void loop_closing();

        static double cost_function(const std::vector<double> &sin_angles, std::vector<double> &grad, void* f_data);

        bool pattern_matching();
		/**
			Main part of the SLAM. Data assocation between observation and landmarks. Updating the landmarks and creating new ones.	
			@param observation_positions The observations in polar coordinates
			@param color_prob all color probabilites for every obseravtion
			@param pose where was these observations
		*/
		void process_observations(
				const std::vector<Eigen::Vector2d>& observation_positions,
				const std::vector<Eigen::Vector4d>& color_prob,
				Eigen::Vector3d pose);



};
#endif
