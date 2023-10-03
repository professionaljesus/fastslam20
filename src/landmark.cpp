#include "landmark.h"

using namespace std;
using namespace Eigen;

Landmark::Landmark(const Vector3d& pose, const Vector2d& z, const Matrix2d& Rt, const size_t& pose_index)
{	
    this->n_obs = 0;
    this->n_not_obs = 0;
    this->pose_index = pose_index;
	
	this->color_probability = Vector4d(0.25, 0.25, 0.25, 0.25);
	this->mu = to_global(z, pose);

	Matrix2d H = measurement_jacobian_landmark(mu, pose);
	Matrix2d Hinv = H.inverse();
	
	this->Sigma =  Hinv.transpose()*Rt*Hinv;	
    
    this->id = id_counter++;

    this->removed = false;
    this->color = -1;
    this->color_est = -1;
    this->assoc_landmark = NULL;

}

//Landmark update
void Landmark::ekf_update(const Vector2d& zdiff, const Matrix2d& Qinv, const Matrix2d& G_theta)
{
	//EKF Update
	Matrix2d K = Sigma * G_theta.transpose() * Qinv;
	mu = mu + K * zdiff;
	//Sigma = (Matrix2d::Identity(2,2) - K*G_theta)*Sigma;
	Sigma -= K*G_theta*Sigma;

}

void Landmark::color_update(const Vector4d& color_prob)
{

	color_probability = color_probability.cwiseProduct(color_prob);
	color_probability /= color_probability.sum();
    
    double best_prob = 0.25;
    for(int i = 0; i < 4; i++)
    {
        if(color_probability[i] > best_prob)
        {
            color = i;
            best_prob = color_probability[i];
        }
    }

}

bool Landmark::has_color()
{
	return color > -1;

}
