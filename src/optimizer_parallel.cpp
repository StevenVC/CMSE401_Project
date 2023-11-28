#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <map>
#include <string>
#include <random>
#include <chrono>

#include "omp.h"

#include "funcs_optimizer.h"

using namespace std;

int main(int argc, char **argv) {
    /*
    Optimization algorithm for a robotic arm. This algorithm takes an n-jointed robotic arm
    defined by the parameters below, and finds the optimal joint parameters such that the
    end effector is located in a desiered position and rotation.

        "v_p_pos" : the initial values for the paramters to be optimized

        "links" : a Denavit–Hartenberg matrix describing the robotic arm, with "v_p_pos"

        "P_d" : the desired final position of the end effector

        "R_d" : the desired final rotation matrix of the end effector

    The program can take the inputs defined below:

        input 1 : "n_particles" : the number of particles to use in the point swarm optimization

        input 2 : "iter_max" : the maximum number of iterations the program may make during 
                               the optimization
        
        input 3 : "pert_rate" : the maximum value that the inital position "v_p_pos" may be
                                randomly perturbed by when the swarm is created. This value also
                                defined the maximum value the inital velocity for each particle
                                may be 

        input 4 : "stop_cond" : defines wether the algorithm will run until the maximum iteration
                                count has been reached (stop_cond=0) or until the algorithm achieves
                                a sufficent accuracy (stop_cond=1)
    */
    // the log is used for various testing cases 

    string fname_log = "log.txt";
    if (argc > 9) {
        fname_log = argv[9];
    }

    ofstream log;
    log.open(fname_log, ios::app);

    auto t_start = chrono::high_resolution_clock::now();
    auto t_end = chrono::high_resolution_clock::now();

    int n_particles = 10;
    if (argc > 1) {
        n_particles = atoi(argv[1]);
    }

    int iter_max = 10;
    if (argc > 2) {
        iter_max = atoi(argv[2]);
    }

    double pert_rate = 0.2;
    if (argc > 3) {
        pert_rate = stod(argv[3]);
    }

    int stop_cond = 0;
    if (argc > 4) {
        stop_cond = atoi(argv[4]);
    }

    string fname_sol = "algorithm_solution.txt";
    if (argc > 5) {
        fname_sol = argv[5];
    }

    string fname_g_hist = "g_best_hist.txt";
    if (argc > 6) {
        fname_g_hist = argv[6];
    }

    int n_threads = 8;
    if (argc > 7) {
        n_threads = atoi(argv[7]);
    }

    double stop_acc = 1e-6;
    if (argc > 8) {
        stop_acc = stod(argv[8]);
    }
    
    // initialize the random number generator engine to use in this program
    default_random_engine rand_gen;

    // variable parameter "positions"
    vector<double> v_p_pos {
        0.1,0.2,0.3,0.1,0.2,0.3
    };

    int n_v_p = v_p_pos.size(); // number of variable parameters

    // initial link parameter values
    vector<vector<double>> links {
        {25, -M_PI/2, 400, v_p_pos[0]},
        {455, 0, 0, v_p_pos[1]},
        {35, M_PI/2, 0, v_p_pos[2]},
        {0, -M_PI/2, -420, v_p_pos[3]},
        {0, M_PI/2, 0, v_p_pos[4]},
        {0, M_PI, -80, v_p_pos[5]}
    };

    // desired position (P_d) and desired rotation (R_d)
    vector<double> P_d {0, 525, 890};

    vector<vector<double>> R_d {
        {0, 1},
        {0, 0}
    };

    // initialize point swarm
    map<int, vector<vector<double>>> point_swarm = make_swarm(n_particles, n_v_p, v_p_pos);


    uniform_real_distribution<double> pert_dist(0.0, pert_rate);
    // randomly perturb each particles positions and velocities
    for (int i=0; i<n_particles; i++) {
        for (int j=0; j<n_v_p; j++) {
            point_swarm[i][0][j] = point_swarm[i][0][j]*pert_dist(rand_gen);
            point_swarm[i][1][j] = pert_dist(rand_gen);
        }
    }

    /*
    Begin optimizing steps
    */

    // personal best: lowest fitness valued particle for each iteration
    vector<double> P_best(n_v_p, 0);
    double P_fit_val = INFINITY;

    // global best: lowest fitness valued particle over the whole iteration history
    vector<double> g_best(n_v_p, 0);
    double g_best_fit = INFINITY;    

    // initialize both an A_map & T_map for use later, values are not important
    map<int, vector<vector<double>>> A_map;

    update_A_map(links, A_map);

    map<int, vector<vector<double>>> T_map(A_map);

    update_T_map(A_map, T_map);
    
    // initialize variables for use inside loops
    double fit_val = 0;
    double alpha = 0.7;
    double beta = 0.3;

    double omega;
    double omega_max = 1;
    double omega_min = 0;

    double phi;

    double k_fact;

    vector<vector<double>> g_best_fit_hist (iter_max, vector<double> (2, 0.0));

    // generate all random variables needed for the maxium number of iterations
    vector<vector<int>> vec_vec_int(n_particles, vector<int> (n_v_p,0));
    vector<vector<double>> vec_vec_double(n_particles, vector<double> (n_v_p,0.0));

    map<int, vector<vector<int>>> c_1;
    map<int, vector<vector<int>>> c_2;
    map<int, vector<vector<double>>> r_1;
    map<int, vector<vector<double>>> r_2;
    map<int, vector<vector<double>>> k_fact_rand;

    for (int i=0; i<iter_max; i++) {
        c_1[i] = vec_vec_int;
        c_2[i] = vec_vec_int;
        r_1[i] = vec_vec_double;
        r_2[i] = vec_vec_double;
        k_fact_rand[i] = vec_vec_double;
    }

    uniform_int_distribution<int> int_dist(2,4);
    uniform_real_distribution<double> real_dist(0.0, 1.0);

    for (int i=0; i<iter_max; i++) {
        for (int j=0; j<n_particles; j++) {
            for (int k=0; k<n_v_p; k++) {
                c_1[i][j][k] = int_dist(rand_gen);
                c_2[i][j][k] = int_dist(rand_gen);
                r_1[i][j][k] = real_dist(rand_gen);
                r_2[i][j][k] = real_dist(rand_gen);

                k_fact_rand[i][j][k] = real_dist(rand_gen);
            }
        }
    }

    // calculate the initial end effector position for each particle & evaluate fitness
    for (int i=0; i<n_particles; i++) {

        update_links(links, point_swarm[i][0]);
        update_A_map(links, A_map);
        update_T_map(A_map, T_map);

        fit_val = calc_fitness(T_map, P_d, R_d, alpha, beta);

        if (fit_val < P_fit_val) {
            P_fit_val = fit_val;
            P_best = point_swarm[i][0];
        }
        if (P_fit_val < g_best_fit) {
            g_best_fit = P_fit_val;
            g_best = point_swarm[i][0];
        }
    }

    // define variables used for parallel thread reconciliation
    vector<double> P_best_thread_fit (n_threads, P_fit_val);
    vector<vector<double>> P_best_thread (n_threads, P_best);


    t_end = chrono::high_resolution_clock::now();
     
    log << "Initial Serial: " << chrono::duration_cast<chrono::microseconds>(t_end - t_start).count() << "\n";
    
    bool break_flag = true; // flag to indicate a break condition has been met
    // begin main optimizer loop
    // begin parallel region
    #pragma omp parallel num_threads(n_threads)
    for (int u=0; u<iter_max && break_flag==true; u++) {

        #pragma omp for private(omega, phi, k_fact, fit_val) firstprivate(links, A_map, T_map, P_fit_val, g_best_fit, P_best, g_best, t_end)
        for (int i=0; i<n_particles; i++) {
            int thread_id = omp_get_thread_num();

            for (int j=0; j<n_v_p; j++) {
                // calculate the random variance introduced to this particle (omega) and its constriction factor (k_fact)
                omega = omega_max - ((omega_max - omega_min)/iter_max) * u;

                phi = c_1[u][i][j] * r_1[u][i][j] + c_2[u][i][j] * r_2[u][i][j];

                k_fact = 2*k_fact_rand[u][i][j] / abs(2-phi-pow(abs(phi*phi - 4*phi),0.5));

                // calculate the "kinematics"
                // update particle accelerations
                point_swarm[i][2][j] = c_1[u][i][j] * r_1[u][i][j] * (P_best_thread[thread_id][j] - point_swarm[i][0][j]) + \
                                    c_2[u][i][j] * r_2[u][i][j] * (g_best[j] - point_swarm[i][0][j]);

                // update particle velocities
                point_swarm[i][1][j] = k_fact * (omega * point_swarm[i][1][j] + point_swarm[i][2][j]);

                // update particle positions
                point_swarm[i][0][j] = point_swarm[i][0][j] + point_swarm[i][1][j];
            }

            update_links(links, point_swarm[i][0]);
            update_A_map(links, A_map);
            update_T_map(A_map, T_map);

            fit_val = calc_fitness(T_map, P_d, R_d, alpha, beta);

            // keep track of the particle with the best fitness in this thread
            if (fit_val < P_best_thread_fit[thread_id]) {
                P_best_thread_fit[thread_id] = fit_val;
                P_best_thread[thread_id] = point_swarm[i][0];
            }
        }

        // keep track of the thread with the particle with the best fitness
        // this should be thread safe
        for (int i=0; i<n_threads; i++) {
            if (P_best_thread_fit[i] < g_best_fit) {
                g_best_fit = P_best_thread_fit[i];
                g_best = P_best_thread[i];
            }
        }

        t_end = chrono::high_resolution_clock::now();

        g_best_fit_hist[u][0] = g_best_fit;
        g_best_fit_hist[u][1] = chrono::duration_cast<chrono::microseconds>(t_end - t_start).count();

        if ((stop_cond == 1) && (g_best_fit <= stop_acc)) {
            break_flag = false;
        }
    }
    // end parallel region

    /*
    The following code is related to writing to file all neccesary data for analysis
    */

    // write out the global best fit for each iteration
    ofstream out_file;
    out_file.open(fname_g_hist);

    for (int i=0; i<iter_max; i++) {
        out_file << g_best_fit_hist[i][0] << "," << g_best_fit_hist[i][1] << "\n";
    }

    out_file.close();

    // write out the final solution
    out_file.open(fname_sol); 

    // print out the best solution from the pso algorithm
    out_file << "The global best solution acoss " << iter_max << " iterations was the following: " << "\n";
    for (int i=0; i<n_v_p; i++) {
        out_file << g_best[i] << ", ";
    }
    out_file << "\n" << "This solution had a fitness value of " << g_best_fit << "\n";

    double opt_sol_time = 0;
    for (int i=0; i<iter_max; i++) {
        if (g_best_fit_hist[i][0] <= stop_acc) {
            opt_sol_time = g_best_fit_hist[i][1]*(1E-6);
            break;
        }
    }
    out_file << "\nThe algorithm attained an accuracy of " << stop_acc << " in " << opt_sol_time << " [s]";
    log << n_threads << ", " << stop_acc << ", " << opt_sol_time << "\n";

    update_links(links, g_best);
    update_A_map(links, A_map);
    update_T_map(A_map, T_map);

    vector<double> final_pos = get_end_coord(T_map);
    vector<vector<double>> final_rot = get_end_rot_mat(T_map);

    // print desired and best pso solution for the end effector positions
    out_file << "\n" << "\n" << "The desired position of the end effector is the following:" << "\n";
    for (auto x : P_d) {
        out_file << x << ", ";
    }
    out_file << "\n" << "\n" << "The best pso solution resulted in an end effector position of:" << "\n";
    for (auto x : final_pos) {
        out_file << x << ", ";
    }

    // print desired and best pso solution for the end effector rotation matrix
    out_file << "\n" << "\n" << "The desired rotation matrix of the end effector is the following:" << "\n";
    for (auto x : R_d) {
        for (auto y : x) {
            out_file << y << ", ";
        }
        out_file << "\n";
    }
    out_file << "\n" << "The best pso solution resulted in an end effector rotation of:" << "\n";
    for (auto x : final_rot) {
        for (auto y : x) {
            out_file << y << ", ";
        }
        out_file << "\n";
    }

    out_file.close();
    log.close();

    return 0;
}

// auto start = chrono::high_resolution_clock::now();
// stuff to time
// auto end = chrono::high_resolution_clock::now();
// auto dur = chrono::duration_cast<chrono::microseconds>(end - start);
// cout << dur.count() << "\n";