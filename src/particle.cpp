#include "particle.h"

using namespace std;
using namespace Eigen;

extern list<pair<Vector3d, Vector3f>> points_to_visualize;

void Particle::process_observations(
        const vector<Vector2d> &observation_positions,
        const std::vector<Eigen::Vector4d> &color_prob,
        Vector3d pose)
{

    associations.clear();

    weight = 0;
    vector<int> found_matches(observation_positions.size(), 0);

    list<Landmark*> associated_landmarks;
    Landmark* landmark;

    list<Landmark*> *landmarks = &local_landmarks;
    if(loop_closed)
    {
        landmarks = &global_landmarks;
    }

    //Finding and updating corresponding landmarks
    for (list<Landmark*>::iterator landmark_iterator = landmarks->begin(); landmark_iterator != landmarks->end(); landmark_iterator++)
    {
        landmark = *landmark_iterator;
        
        Vector2d z_hat = to_local(landmark->mu, pose);

        if(z_hat.norm() > range_range[1] + 5)
        {

            if(!loop_closed)
            {
                if(landmark->Sigma.determinant() < 0.001)
                {
                    global_landmarks.push_back(landmark);    
                }
    
                landmark_iterator = landmarks->erase(landmark_iterator);
                landmark_iterator--;
            }

            continue;
        }

        Matrix2d G_theta = measurement_jacobian_landmark(landmark->mu, pose);
        Matrix2d Q = (G_theta * landmark->Sigma * G_theta.transpose() + Rt);
        Matrix2d Qinv = Q.inverse();

        //Weight
        double den = sqrt(2 * M_PI * Q.determinant());

        //Finding best corresponding observation
        double best_weight = 0;
        int best_index = 0;

        for (size_t i = 0; i < observation_positions.size(); i++)
        {

            if (found_matches[i] == 1)
            {
                continue;
            }

            Vector2d z = observation_positions[i];
            Vector2d zdiff = z - z_hat;

            double num = exp(-0.5 * zdiff.transpose() * Qinv * zdiff);
            double weight = num / den;

            if (weight > best_weight)
            {
                best_weight = weight;
                best_index = i;
            }
        }

        if (best_weight > association_threshold)
        {
            Vector2d z = observation_positions[best_index];
            Vector2d zdiff = z - z_hat;
            found_matches[best_index] = 1;

            landmark->n_obs += 1;
            associated_landmarks.push_back(landmark);

            //Fastslam2.0 pose update
            Matrix<double, 2, 3> G_s = measurement_jacobian_pose(landmark->mu, pose);

            Matrix3d Sigma_s = (G_s.transpose() * Qinv * G_s + Particle::Ptinv).inverse();
            pose = Sigma_s * G_s.transpose() * Qinv * zdiff + pose;
            pose[2] = remainder(pose[2], 2 * M_PI);

            //I AM VERY UNSURE ABOUT THIS STEP. IS NOT IN THE PSEUDOCODE BUT IT MAKES SENSE
            z_hat = to_local(landmark->mu, pose);
            zdiff = z - z_hat;

            Matrix2d L = G_s * Particle::Pt * G_s.transpose() + Q;
            double den = sqrt(2 * M_PI * L.determinant());
            double num = exp(-0.5 * zdiff.transpose() * L.inverse() * zdiff);
            best_weight = num / den;
            weight += best_weight;

            if(!loop_closed)
            {
                //Update landmark
                landmark->ekf_update(zdiff, Qinv, G_theta);
            }
            landmark->color_update(color_prob[best_index]);

            associations.emplace_back(landmark->mu, to_global(z, pose));
        }
        else if(!loop_closed && z_hat.norm() > range_range[0] + 0.2 && z_hat.norm() < range_range[1] - 4 && abs(atan2(z_hat.y(), z_hat.x())) < angle_range[1] - 0.2)
        {
            //Landmark should have been seen but is not
            landmark->n_not_obs += 1;
            float obs_quote = (float)landmark->n_obs / ((float)(landmark->n_obs + landmark->n_not_obs));
            if (obs_quote < observation_quote_threshold)
            {
                landmark->removed = true;
                landmark_iterator = landmarks->erase(landmark_iterator);
                //delete landmark;
                landmark_iterator--;
            }
        }
    }

    if(!loop_closed)
    {
        for(Landmark* landmark : associated_landmarks) 
        {

            for(Landmark* buddy : associated_landmarks)
            {
                if(landmark->id != buddy->id)
                {
                    landmark->landmark_buddies[buddy] += 1;
                }
            }

            //We've left the start and a new orange is found
            if(left_home && landmark->color > 1)
            {
                search_for_loop_closure = true;
            }


            if(!landmark->has_color())
            {
                Landmark* closest_landmark = NULL; 
                double closest_dist = DBL_MAX;
                for(Landmark* buddy : *landmarks)
                {

                    if(landmark->id == buddy->id)
                    {
                        continue;
                    }
                    if(buddy->removed)
                    {
                        //Removed from map?
                        continue;
                    }
            
                    double dist = (buddy->mu - landmark->mu).norm();

                    if(dist < closest_dist && (buddy->has_color() || buddy->color_est > -1))
                    {
                        closest_landmark = buddy; 
                        closest_dist = dist;
                    }
                }
                if(closest_landmark != NULL)
                {
                    if(closest_landmark->has_color())
                    {
                        landmark->color_est = closest_landmark->color;
                    }
                    else
                    {
                        landmark->color_est = closest_landmark->color_est;
                    }
                }
                
            }
        }
    }

    if(!loop_closed)
    {
        //Add all observations that were not matched as new landmarks
        for (size_t i = 0; i < observation_positions.size(); i++)
        {
            if (found_matches[i] == 0)
            {
                Vector2d z = observation_positions[i];
                Landmark* new_landmark = new Landmark(pose, z, Rt, poses.size());

                new_landmark->color_probability = color_prob[i];
                local_landmarks.push_back(new_landmark);
                //If you make alot of new landmarks, bad
                weight += NEW_LANDMARK_WEIGHT;
            }
        }
    }

    if(!left_home && pose.head(2).norm() > 15)
    {
        left_home = true;
    }

    poses.push_front(pose);
}

bool Particle::operator>(const Particle &other) const
{
    return this->weight > other.weight;
}

bool Particle::operator<(const Particle &other) const
{
    return this->weight < other.weight;
}
