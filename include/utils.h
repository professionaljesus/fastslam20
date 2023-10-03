#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include <nlopt.hpp>

#include <math.h>  
#include <algorithm>
#include <iostream>  
#include <iterator>
#include <stdlib.h>
#include <cmath>
#include <random>
#include <cfloat>
#include <list>
#include <vector>
#include <cfloat>
#include <tuple>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <thread>

#define p(x) cout << x << endl << std::flush;

#define set_debug 1
#if set_debug == 1
#define DBG(x) cout << x << endl << std::flush;

#define DBGEIG( var ) \
       std::cout << fixed << #var << " = " << var.format(Eigen::IOFormat(StreamPrecision, DontAlignCols, ", ", "; ", "", "", "[ ", " ]")) << endl << std::flush;

#define DBGVAR( var ) \
       std::cout << fixed << #var << " = " << (var) << endl << std::flush;

#define DBGAR( var, len ) \
		std::cout << #var << ": "; \
		for(size_t i = 0; i < len; i++) { \
       		std::cout << fixed << var[i] << ", "; \
		} \
		std::cout << endl << std::flush;

#define DBGIT( var ) \
		std::cout << #var << " length = " << (var.size()) << ": "; \
		for(auto a : var) { \
       		std::cout << fixed << a << ", "; \
		} \
		std::cout << endl << std::flush;
#else
#define DBG(x) 
#define DBGEIG( var ) 
#define DBGVAR( var )
#define DBGAR( var, len ) 
#define DBGIT( var ) 
#endif

Eigen::Vector2d to_global(const Eigen::Vector2d& z, const Eigen::Vector3d& pose); 


//To poseations local polar system
Eigen::Vector2d to_local(const Eigen::Vector2d& lm_pos, const Eigen::Vector3d& pose);


//Calculate measurement jacobian H
Eigen::Matrix2d measurement_jacobian_landmark(const Eigen::Vector2d& landmark_pos, const Eigen::Vector3d& pose); 

Eigen::Matrix<double, 2, 3> measurement_jacobian_pose(const Eigen::Vector2d& landmark_pos, const Eigen::Vector3d& pose); 
#endif
