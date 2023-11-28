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

void print_2d_vec(vector<vector<double>>& a) {
    /*
    inputs:
        a : 2d vector
    */

    for (auto x : a) {
        for (auto y : x) {
            cout << y << ", ";
        }

        cout << "\n";
    }
}

void write_2d_vec(vector<vector<double>>& a, ofstream& out_file) {
    /*
    inputs:
        a : 2d vector

        outfile : ofstream object used to write to a file
    */

    for (auto x : a) {
        for (auto y : x) {
            out_file << y << ", ";
        }

        out_file << "\n";
    }
}

void write_A_T_map(map<int, vector<vector<double>>>& A, map<int, vector<vector<double>>>& T, ofstream& out_file) {
    /*
    inputs:
        A : map of 2d vectors containing the D-H representation of each link

        T : map of 2d vectors containing the translation matrix for each joint

        outfile : ofstream object used to write to a file
    */

    out_file.open("A_map.txt");
    out_file << "A map :\n";
    for (int i=0; i<A.size(); i++) {
        write_2d_vec(A[i], out_file);
        out_file << "\n";
    }
    out_file.close();

    out_file.open("T_map.txt");
    out_file << "T map :\n";
    for (int i=0; i<T.size(); i++) {
        write_2d_vec(T[i], out_file);
        out_file << "\n";
    }
    out_file.close();
}

void print_joint_coords(map<int, vector<vector<double>>>& T) {
    /*
    inputs:
        T : map of 2d vectors containing the translation matrix for each joint
    */
    
    int mat_width = T[0].size(); // y size of the translation matrix, should be 4

    for (int i=0; i<T.size(); i++) {
        cout << i << ", " ;

        for (int j=0; j<mat_width-1; j++) { // write the last element in each row of the translation matrix, excluding the last row
            // cout << T[i][j][mat_length] << ", ";
            cout << T[i][j][mat_width-1] << ", ";
        }

        cout << "\n";
    }
}

void write_joint_coords(map<int, vector<vector<double>>>& T, ofstream& out_file) {
    /*
    inputs:
        T : map of 2d vectors containing the translation matrix for each joint

        outfile : ofstream object used to write to a file
    */
    
    int mat_width = T[0].size(); // y size of the translation matrix, should be 4
    
    out_file.open("coords_joint.txt");

    out_file << "j#, x, y, z" << "\n";
    
    for (int i=0; i<T.size(); i++) {
        out_file << i << ", " ;

        for (int j=0; j<mat_width-1; j++) { // write the last element in each row of the translation matrix, excluding the last row
            out_file << T[i][j][mat_width-1] << ", ";
        }

        out_file << "\n";
    }

    out_file.close();
}

void print_end_coord(map<int, vector<vector<double>>>& T) {
    for (int i=0; i<T[T.size()-1].size()-1; i++) {
        cout << T[T.size()-1][i][3] << ", ";
    }

    cout << "\n";
}

void write_end_coord(map<int, vector<vector<double>>>& T, ofstream& out_file) {
    out_file.open("coords_end.txt");

    for (int i=0; i<T[T.size()-1].size()-1; i++) {
        out_file << T[T.size()-1][i][3] << ", ";
    }

    out_file.close();
}

void write_end_coord_noClose(map<int, vector<vector<double>>>& T, ofstream& out_file) {
    for (int i=0; i<T[T.size()-1].size()-1; i++) {
        out_file << T[T.size()-1][i][3] << ", ";
    }

    out_file << "\n";
}