#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <map>
#include <string>
#include <random>

using namespace std;

vector<vector<double>> dot_product(vector<vector<double>>& a, vector<vector<double>>& b) {
    /*
    inputs:
        a : 2d vector

        a : 2d vector
    */

    vector<vector<double>> c(a);
    int c_size = c.size();

    for (int i=0; i < c_size; i++) {
        for (int j=0; j < c_size; j++) {
            c[i][j] = 0; // set current c[i][j] value to zero, then compute inner prod

            for (int k=0; k < c_size; k++) {
                    c[i][j] = c[i][j] + (a[i][k] * b[k][j]);
            }
        }
    }

    return c;
}

vector<vector<double>> make_A_i(double a, double al, double d, double th) {
    /*
    inputs:
        a : link length

        al : link twist

        d : link offset

        th : joint angle
    */

    vector<vector<double>> A_i {
        {cos(th), -sin(th)*cos(al), sin(th)*sin(al), a*cos(th)},
        {sin(th), cos(th)*cos(al), -cos(th)*sin(al), a*sin(th)},
        {0, sin(al), cos(al), d},
        {0, 0, 0, 1}
    };

    return A_i;
}

void update_A_map(vector<vector<double>>& links, map<int, vector<vector<double>>>& A) {
    /*
    inputs:
        links : 2d vector describing the link parameters for each link

        A : map of 2d vectors containing the D-H representation of each link
    */
    
    for (int i=0; i<links.size(); i++) {
        double a = 0;
        double al = 0;
        double d = 0;
        double th = 0;

        vector<double> x = links[i];

        a = x[0];
        al = x[1];
        d = x[2];
        th = x[3];

        A[i] = make_A_i(a, al, d, th);
    }
}

void update_T_map(map<int, vector<vector<double>>>& A, map<int, vector<vector<double>>>& T) {
    /*
    inputs:
        A : map of 2d vectors containing the D-H representation of each link

        T : map of 2d vectors containing the translation matrix for each joint
    */

    for (int i=0; i<T.size(); i++) {
        if (i == 0) {
            T[i] = A[i];
        }
        else {
            T[i] = dot_product(T[i-1], A[i]);
        }
    }
}

vector<vector<double>> get_end_rot_mat(map<int, vector<vector<double>>>& T) {
    vector<vector<double>> rot_mat {
        {0,0},
        {0,0}
    };
    
    for (int i=0; i<2; i++) {
        for (int j=0; j<2; j++) {
            rot_mat[i][j] = T[T.size()-1][i][j];
        }
    }

    return rot_mat;
}

vector<double> get_end_coord(map<int, vector<vector<double>>>& T) {
    vector<double> end_coords;
    
    for (int i=0; i<T[T.size()-1].size()-1; i++) {
        end_coords.push_back(T[T.size()-1][i][3]);
    }

    return end_coords;
}
void update_links(vector<vector<double>>& links, vector<double>& params) {
    /*
    This function must be updated if the link parameters are changed
    */
    
    links[0][3] = params[0]; // update link parameter theta
    links[1][3] = params[1]; // update link parameter theta
    links[2][3] = params[2]; // update link parameter theta
    links[3][3] = params[3]; // update link parameter theta
    links[4][3] = params[4]; // update link parameter theta
    links[5][3] = params[5]; // update link parameter theta
}

double error_pos(map<int, vector<vector<double>>>& T, vector<double>& P_d) {
    /*
    Following from [1] calculate the positional error using (6)

    inputs:
        T : 

        P_d :

    returns:
        E_p :
    */

    vector<double> P_pso = get_end_coord(T);

    double top;
    double bot;

    double E_p;

    // take difference between estimated and desired position
    for (int i=0; i<3; i++) {
        P_pso[i] -= P_d[i];
    }

    // compute L2 norm of top and bottom
    for (int i=0; i<3; i++) {
        top += P_pso[i]*P_pso[i];
        bot += P_d[i]*P_d[i];
    }

    E_p = top/bot;

    return E_p;
}

double error_rot(map<int, vector<vector<double>>>& T, vector<vector<double>>& R_d) {
    /*
    Following from [1] calculate the rotational error using (7)

    inputs:
        T : 

        R_d :

    returns:
        E_r :
    */

    vector<vector<double>> R_pso = get_end_rot_mat(T);

    double top;
    double bot;

    double E_r;

    // take difference between estimated and desired rotation
    for (int i=0; i<2; i++) {
        for (int j=0; j<2; j++) {
            R_pso[i][j] -= R_d[i][j];
        }
    }

    // compute L2 norm of top and bottom
    for (int i=0; i<2; i++) {
        for (int j=0; j<2; j++) {
            top += R_pso[i][j]*R_pso[i][j];
            bot += R_d[i][j]*R_d[i][j];
        }
    }

    E_r = top/bot;

    return E_r;
}

double calc_fitness(map<int, vector<vector<double>>>& T, vector<double>& P_d, vector<vector<double>>& R_d, double& alpha, double& beta) {
    double E_pos = error_pos(T, P_d);
    double E_rot = error_rot(T, R_d);

    // cout << E_pos << ", " << E_rot << "\n";
    
    double rho = alpha*exp(-E_rot) + beta;

    double fit_value = rho*E_pos + (1-rho)*E_rot;

    return fit_value;
}

void perturb_particle(map<int, vector<vector<double>>>& swarm, float max_perturbation) {
    int n_particles = swarm.size();
    int n_v_p = swarm[0][0].size();

    for (int i=0; i<n_particles; i++) {
        for (int j=0; j<n_v_p; j++) {
            swarm[i][0][j] = swarm[i][0][j]*1;
        }
    }
}

vector<vector<double>> make_particle(int n_v_p,  vector<double>& init_pos) {
    /*
    Initialize a particle in the optimizer swarm

    inputs:
        n_v_p : the number of variable parameters for each particle; int

        init_pos : the inital values for each variable parameter for each particle; vector<double>

    returns:
        particle_n : the initialized particle; vector<vector<double>>
    */
    vector<vector<double>> particle_n(n_v_p, vector<double> (n_v_p, 0));

    particle_n[0] = init_pos; // initialize position 
    particle_n[1] = vector<double> (n_v_p, 0); // initialize velocity
    particle_n[2] = vector<double> (n_v_p, 0); // initialize acceleration

    return particle_n;
}

map<int, vector<vector<double>>> make_swarm(int n_particles, int n_v_p, vector<double>& init_pos) {
    /*
    Initialize a swarm of particles

    inputs:
        n_particles : the number of particles in the swarm; int

        n_v_p : the number of variable parameters for each particle; int

        init_pos : the inital values for each variable parameter for each particle; vector<double>

    returns:
        swarm : the initialized swarm, containing n_particles; map<int, vector<vector<double>>>
    */
    map<int, vector<vector<double>>> swarm;

    for (int i=0; i<n_particles; i++) {
        swarm[i] = make_particle(n_v_p, init_pos);
    }

    return swarm;
}