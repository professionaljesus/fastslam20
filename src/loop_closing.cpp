#include "particle.h"

using namespace std;
using namespace Eigen;

extern list<pair<Vector3d, Vector3f>> points_to_visualize;

bool Particle::pattern_matching()
{
    
    //If there are more local_landmarks than global we wont be able to find a pattern
    if(global_landmarks.size() < local_landmarks.size())
    {
        cout << "too few globals" << endl;
        return false;
    }

    vector<Landmark*> pattern = vector<Landmark*>((3*local_landmarks.size())/4);
    
    size_t i = 0;
    for(Landmark* landmark : global_landmarks)
    {
        if(i < pattern.size())
        {
            pattern[i] = landmark;
        }
        else
        {
            break;
        }
        i++;
    }

    Landmark* mid_landmark = pattern[pattern.size()/2];
    Landmark* dir_landmark = NULL;

    double dist = 0;
    double max_dist = 0;
    
    //Finding the landmark furthest away to determine direction
    for(Landmark* landmark : pattern)
    {
        dist = (mid_landmark->mu - landmark->mu).norm();
        if(dist > max_dist)
        {
            max_dist = dist;
            dir_landmark = landmark;
        }
    }

    if(dir_landmark == NULL)
    {
        cout << "No dir_landmark was found in loop closing." << endl;
        return false;
    }

    Vector2d direction = dir_landmark->mu - mid_landmark->mu;
    double dir_norm = direction.norm();
    double dir_angle = atan2(direction.y(), direction.x());

    /* ------------ 
        originial pattern has now been created,
        mid_landmark is the middle of the pattern,
        dir_landmark is the landmark furthest away from mid_landmark

        we will now try to find the corresponding
        mid_landmark and dir_landmark at the end of the track

        When the possible mid_landmark and dir_landmark are found we will try overlaying
        the old pattern with the possible new one, counting the number over overlapping
        landmarks with the old one. 

        The mid_landmark-dir_landmark pair which creates the most overlaps has the highest
        probability of being the correct pair and there we have found our correspondence.
        The rest of the correspondences of the pattern are based on the overlaps.

    -------------- */


    vector<Vector2d> transformed_pattern(pattern.size());
    size_t max_inliers = 0;
    
    list<pair<Landmark*, Landmark*>> best_associations;

    // Looping through landmarks, trying to find the corresponding mid_landmark
    for(Landmark* landmark : local_landmarks)
    {
        list<pair<Landmark*, double>> possible_dir_landmarks;
        // Try finding to find the corresponding mid_landmark
        // We are sorting on the difference in distance
        for(Landmark* possible_dir_landmark : local_landmarks)
        {
            double diff = abs((possible_dir_landmark->mu - landmark->mu).norm() - dir_norm);
            possible_dir_landmarks.emplace_back(possible_dir_landmark, diff);
        }

        possible_dir_landmarks.sort( []( const pair<Landmark*,double> &a, const pair<Landmark*,double> &b ) { return a.second < b.second; } );
        possible_dir_landmarks.resize(3);

        Matrix2d rotation_matrix;
        for(pair<Landmark*, double> pp : possible_dir_landmarks)
        {

            Landmark* possible_dir_landmark = pp.first;
            Vector2d possible_dir = possible_dir_landmark->mu - landmark->mu;

            double rotation = atan2(possible_dir.y(), possible_dir.x()) - dir_angle;
            rotation_matrix << cos(rotation), -sin(rotation),
                               sin(rotation), cos(rotation);

            /*
                We transform all landmarks from the pattern in the beginning to 
                where they would end up in the new possible pattern.
            */

            for(size_t j = 0; j < pattern.size(); j++)
            {
                transformed_pattern[j] = rotation_matrix*(pattern[j]->mu - mid_landmark->mu) + landmark->mu;
            }

            // We check how many overlaps there are with this 
            // mid_landmark - dir_landmark combo

            list<pair<Landmark*, Landmark*>> temp_associations;
            size_t inliers = 0;
            for(size_t j = 0; j < transformed_pattern.size(); j++)
            {
                Vector2d v = transformed_pattern[j];
                double closest = FLT_MAX;
                Landmark* closest_landmark = NULL;
                for(Landmark* local_landmark : local_landmarks)
                {
                    double dist = (v - local_landmark->mu).norm();
                    if(dist < closest)
                    {
                        closest = dist;
                        closest_landmark = local_landmark;
                    }
                }

                temp_associations.emplace_back(closest_landmark, pattern[j]);

                if(closest < 0.2)
                {
                    inliers++;
                }
            }

            if(inliers > max_inliers)
            {
                max_inliers = inliers;
                best_associations = temp_associations;
            }

        }

    }

    bool success = false;
    
    if(max_inliers > 0.6 * (float) pattern.size())
    {
        success = true;
        for(pair<Landmark*, Landmark*> assoc : best_associations)
        {
            assoc.first->assoc_landmark = assoc.second;
            assoc.second->assoc_landmark = assoc.first;
        }
    }

    cout << "Max Inliers: " << to_string(max_inliers) << '\n';
    cout << "We require: " << to_string(0.6 * (float)pattern.size()) << endl;

    return success;
}

void Particle::loop_closing()
{
    Landmark* first_blue = NULL;
    for(Landmark* landmark : global_landmarks)
    {
        if(landmark->color == 0 && landmark->assoc_landmark != NULL)
        {
            first_blue = landmark;
            break;
        }
    }

    if(first_blue == NULL)
    {
        cout << "first blue not found" << endl;
        return;
    }

    Landmark* closest_landmark;
    double min_dist = DBL_MAX;
    double dist;

    size_t i = 0;

    //Find the closest blue thats infront of the first one
    for(Landmark* landmark : global_landmarks)
    {
        i++;
        if(i > global_landmarks.size()/2)
        {
            break;
        }

        if(landmark->id == first_blue->id || landmark->mu.x() < 0 || landmark->color > 0 || (landmark->color == -1 && landmark->color_est != 0))
        {
            continue;
        }

        dist = (first_blue->mu - landmark->mu).norm();
        if(dist < min_dist)
        {
            min_dist = dist;
            closest_landmark = landmark;
        }
    }

    Landmark* second_blue = closest_landmark;
    

    list<Landmark*> temp_landmarks = global_landmarks;
    for(Landmark* landmark : local_landmarks)
    {
        temp_landmarks.push_back(landmark);
    }
    
    set<int> used;
    used.insert(first_blue->id);
    used.insert(second_blue->id);
    vector<Landmark*> sorted_blue;
    sorted_blue.push_back(first_blue);
    sorted_blue.push_back(second_blue);
    vector<Vector2d> relative_blue;

    relative_blue.push_back(second_blue->mu - first_blue->mu);

    vector<double> weights;
    weights.push_back(first_blue->landmark_buddies[second_blue]);
    double total_weight = weights[0];
    double w = 0;
    Vector2d relative;
    Landmark* last_with_assoc = NULL;
    size_t last_with_assoc_idx = 0;
    map<size_t, Vector2d> to_cmp;
     
    cout << "stuck in infinity? " << endl << flush;
    while(last_with_assoc == NULL || sorted_blue.back() != last_with_assoc->assoc_landmark)
    {
        min_dist = DBL_MAX;
        closest_landmark = NULL;

        for(Landmark* landmark : temp_landmarks)
        {
            if(abs(sorted_blue.back()->id - landmark->id) > temp_landmarks.size()/2 || landmark->color > 0 || (landmark->color == -1 && landmark->color_est != 0) || used.find(landmark->id) != used.end())
            {
                continue;
            }

            dist = (sorted_blue.back()->mu - landmark->mu).norm();
            if(dist < min_dist)
            {
                min_dist = dist;
                closest_landmark = landmark;
            }
        }

        if(closest_landmark == NULL)
        {
            break;
        }


        if(closest_landmark->assoc_landmark != NULL)
        {
            if(closest_landmark->id < temp_landmarks.size()/2)
            {
                last_with_assoc = closest_landmark;
                last_with_assoc_idx = sorted_blue.size();
            }
            else
            {
                //bruh wat
                to_cmp[sorted_blue.size() - 1] = (closest_landmark->assoc_landmark)->mu - first_blue->mu;
            }
        }

        relative = closest_landmark->mu - sorted_blue.back()->mu;

        //w = relative_blue.back().dot(relative)/(relative_blue.back().norm() * relative.norm());
        w = sorted_blue.back()->landmark_buddies[closest_landmark];
        weights.push_back(w);
        total_weight += w;

        associations.emplace_back(closest_landmark->mu, sorted_blue.back()->mu);
        relative_blue.push_back(relative);
        sorted_blue.push_back(closest_landmark);
        used.insert(closest_landmark->id);
    }
    cout << "no" << endl << flush;

    map<Landmark*, list<pair<Landmark*, Vector2d>>> to_move;
    Vector2d local_x, local_y, local;
    Landmark* blue;
    size_t closest_blue_idx = 0;
    
    for(Landmark* landmark : temp_landmarks)
    {
        if(used.find(landmark->id) != used.end())
        {
            continue;
        }
        min_dist = DBL_MAX;
        for(i = 0; i < sorted_blue.size(); i++)
        {
            blue = sorted_blue[i];
            //If the id is too far apart they come from different laps
            if(abs(blue->id - landmark->id) > temp_landmarks.size()/2)
            {
                continue;
            }

            dist = (blue->mu - landmark->mu).norm();
            if(dist < min_dist)
            {
                min_dist = dist;
                closest_blue_idx = i;
            }
        }
        
        blue = sorted_blue[closest_blue_idx];
        if(closest_blue_idx + 1 >= sorted_blue.size())
        {
            continue;
        }
        
        local_x = sorted_blue[closest_blue_idx + 1]->mu - blue->mu;
        local_x /= local_x.norm();
        local_y << -local_x.y(), local_x.x();

        relative = landmark->mu - blue->mu;
        local << relative.dot(local_x), relative.dot(local_y);

        to_move[blue].emplace_back(landmark, local);
    }


    // Normalize the weights
    for(i = 0; i < weights.size(); i++)
    {
        weights[i] = 1 - (weights[i] / total_weight);
    }
    
    
    tuple<
          map<size_t, Vector2d>*,
          vector<Vector2d>*,
          vector<double>*,
          size_t*,
          map<Landmark*, list<pair<Landmark*, Vector2d>>>*,
          vector<Landmark*>*
            > data_tuple = 

          make_tuple(&to_cmp, &relative_blue, &weights, &last_with_assoc_idx, &to_move, &sorted_blue);


    
    cout << "last_with_assoc_idx: " << last_with_assoc_idx << endl;
    
    for(const auto &kv : to_cmp)
    {

        cout << kv.first << " " << kv.second << endl;

    }
    
    //Setup optimizer
    size_t n = weights.size();
    nlopt::opt opt(nlopt::LN_NELDERMEAD, n);
    vector<double> lb(n, -0.2);
    opt.set_lower_bounds(lb);
    vector<double> ub(n, 0.2);
    opt.set_upper_bounds(ub);
    opt.set_min_objective(cost_function, &data_tuple);
    opt.set_xtol_rel(1e-2);
    vector<double> x(n, 0);
    double minf;

    try{
        nlopt::result result = opt.optimize(x, minf);
        cout << "found minimum f(";

        for(const double &xx : x)
        {
            cout << xx << ",";
        }
        cout << ")\n";
        cout << "minf: " << to_string(minf) << endl;
    }
    catch(std::exception &e) 
    {
        cout << "nlopt failed: " << e.what() << endl;
    }
   
    double cos_i = 0;
    double sin_i = 0;

    //MOVING THE CAR
    Landmark* l1 = NULL;
    Landmark* l2 = NULL;

    list<pair<Landmark*, double>> closest;
    for(Landmark* landmark : local_landmarks)
    {
        if(landmark->assoc_landmark != NULL)
        {
            dist = (landmark->mu - poses.front().head(2)).norm();
            closest.emplace_back(landmark, dist);
        }
    }
    closest.sort( []( const pair<Landmark*,double> &a, const pair<Landmark*,double> &b ) { return a.second < b.second; } );
    closest.resize(2);
    l1 = closest.front().first;
    l2 = closest.back().first;
    Vector2d car_rel = poses.front().head(2) - l1->mu;

    Matrix2d rotation_matrix;
    relative = l2->mu - l1->mu;
    Vector2d new_relative = l2->assoc_landmark->mu - l1->assoc_landmark->mu;
    double rotation = atan2(new_relative.y(), new_relative.x()) - atan2(relative.y(), relative.x());
    cos_i = cos(rotation);
    sin_i = sin(rotation);

    rotation_matrix << cos_i, -sin_i,
                        sin_i, cos_i;

    Vector2d translation = (l1->assoc_landmark->mu - l1->mu) + (rotation_matrix*car_rel) - car_rel;

    poses.front().head(2) += translation;
    poses.front().z() += rotation;
    poses.front().z() = remainder(poses.front().z(), 2 * M_PI);
    

    Vector2d most_recent = first_blue->mu;
    double accumulated_angle = 0;
    
    for(size_t i = 0; i < x.size(); i++)
    {
        if(i > last_with_assoc_idx && to_cmp.find(i) == to_cmp.end())
        {
            accumulated_angle += x[i];
        }

        sin_i = sin(accumulated_angle);
        cos_i = cos(accumulated_angle);
        rotation_matrix << cos_i, -sin_i,
                           sin_i, cos_i;

        most_recent += rotation_matrix*relative_blue[i];
        sorted_blue[i + 1]->mu = most_recent;
        local_x = sorted_blue[i + 1]->mu - sorted_blue[i]->mu;
        local_x /= local_x.norm();
        local_y << -local_x.y(), local_x.x();

        for(const pair<Landmark*, Vector2d> &p : to_move[sorted_blue[i]]) 
        {
            p.first->mu = sorted_blue[i]->mu + p.second.x()*local_x + p.second.y()*local_y;
        }
    }

    Landmark* landmark;
    for (list<Landmark*>::iterator landmark_iterator = local_landmarks.begin(); landmark_iterator != local_landmarks.end(); landmark_iterator++)
    {

        landmark = *landmark_iterator;
        if(landmark->assoc_landmark != NULL || landmark->Sigma.determinant() > 0.001)
        {
            landmark->removed = true;
        }
        else
        {
            global_landmarks.push_back(landmark);    
        }

        landmark_iterator = local_landmarks.erase(landmark_iterator);
        landmark_iterator--;
    }

    loop_closed = true;

}



// Cost function, read nlopt documentation
double Particle::cost_function(const vector<double> &angles, vector<double> &grad, void* f_data)
{
    double cos_i = 0;
    double sin_i = 0;
    double reg = 0;
    double cost = 0;

    Matrix2d rotation_matrix;
    
    tuple<

        map<size_t, Vector2d>*,
         vector<Vector2d>*,
         vector<double>*,
         size_t*,
        map<Landmark*, list<pair<Landmark*, Vector2d>>>*,
        vector<Landmark*>*

        > *data_tuple = 
        (tuple<

        map<size_t, Vector2d>*,
         vector<Vector2d>*,
         vector<double>*,
         size_t*,
        map<Landmark*, list<pair<Landmark*, Vector2d>>>*,
        vector<Landmark*>*

        >*) f_data; 

    map<size_t, Vector2d> &to_cmp = *(get<0>(*data_tuple));
    vector<Vector2d> &relative_blue = *(get<1>(*data_tuple));
    vector<double> &weights = *(get<2>(*data_tuple));
    size_t &last_with_assoc_idx = *(get<3>(*data_tuple));

    map<Landmark*, list<pair<Landmark*, Vector2d>>> &to_move = *(get<4>(*data_tuple));
    vector<Landmark*> &sorted_blue = *(get<5>(*data_tuple));

    Vector2d most_recent(0,0);
    Vector2d temp, local_x, local_y, v;
    double accumulated_angle = 0;

    for(size_t i = 0; i < angles.size(); i++)
    {
        
        if(i > last_with_assoc_idx && to_cmp.find(i) == to_cmp.end())
        {
            accumulated_angle += angles[i];
            reg += angles[i]*angles[i];
        }

        sin_i = sin(accumulated_angle);
        cos_i = cos(accumulated_angle);
        
        rotation_matrix << cos_i, -sin_i,
                            sin_i, cos_i;

        temp = most_recent;
        most_recent += rotation_matrix*relative_blue[i];

        if(false && i > angles.size()/2 && to_move[sorted_blue[i]].size() > 0)
        {
            local_x = most_recent - temp;
            local_x /= local_x.norm();
            local_y << -local_x.y(), local_x.x();

            for(const pair<Landmark*, Vector2d> &p : to_move[sorted_blue[i]]) 
            {
                if(p.first->assoc_landmark != NULL)
                {
                    v = temp + p.second.x()*local_x + p.second.y()*local_y;
                    cost += (v - ((p.first->assoc_landmark)->mu - sorted_blue[0]->mu)).norm();
                }
            }
        }
        

        if(to_cmp.find(i) != to_cmp.end())
        {
            cost += (to_cmp[i] - most_recent).norm();
        }
        
    }
    
    return cost + reg;
}

