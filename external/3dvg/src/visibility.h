/*
* visibility.h
*
* ---------------------------------------------------------------------
* Copyright (C) 2023 Matthew (matthewoots at gmail.com)
*
*  This program is free software; you can redistribute it and/or
*  modify it under the terms of the GNU General Public License
*  as published by the Free Software Foundation; either version 2
*  of the License, or (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* ---------------------------------------------------------------------
*/

#ifndef VISIBILITY_H
#define VISIBILITY_H

#include "visilibity.hpp"
#include <Eigen/Dense>

#include <string>
#include <cfloat>
// #include <iostream>
#include <math.h>
#include <map>
#include <chrono>

using namespace Eigen;
using namespace VisiLibity;

using namespace std::chrono;

typedef time_point<std::chrono::system_clock> t_p_sc; // giving a typename

namespace visibility_graph
{

    /** @brief Obstacle structure
     * @param v Flattened vertices of the obstacle
     * @param h Height that the obstacle is extruded to
     * @param c Flattened centroid of the obstacle
    **/
    struct obstacle
    {
        std::vector<Eigen::Vector2d> v; // Base vertices
        std::pair<double, double> h; // Height that is it extruded to
        Eigen::Vector2d c; // Centroid
    };

    /** @brief Global map structure
     * @param start_end Start and end point pair (3D)
     * @param obs Obstacles in the map
     * @param inflation Safety margin that is used to expand the map
     * @param t Transform and oriented to start-end vector
     * @param rpy RPY that is recorded for the transform
    **/
    struct global_map
    {
        std::pair<Eigen::Vector3d, Eigen::Vector3d> start_end; // Start and end pair
        std::vector<obstacle> obs; // Obstacles
        double inflation; // Safety margin
        Eigen::Affine3d t; // Transform and oriented to start-end vector
        Eigen::Vector3d rpy; // RPY that is recorded for the transform
    };

    /** 
     * @brief get_centroid_2d
     * @param vect Vector of points used to get centroid
     * @param (Return) Centroid
    **/
    Eigen::Vector2d get_centroid_2d(
        std::vector<Eigen::Vector2d> vect);

    /** 
     * @brief get_rotation
     * @param rpy Euler angles in Eigen::Vector3d
     * @param frame Coordinate frame used
     * @param (Return) 3x3 Rotation matrix
    **/
    Eigen::Matrix3d get_rotation(
        Eigen::Vector3d rpy, std::string frame);

    /** 
     * @brief graham_scan, sorts the polygon clockwise https://stackoverflow.com/a/57454410
     * Holes listed in clockwise or counterclockwise up to the direction
     * @param points_in Vector of points in
     * @param centroid Centroid of the flattened polygon
     * @param dir Clockwise or counterclockwise direction sorting of points_out
     * @param points_out (Return) Vector of points out
    **/
    void graham_scan(
        std::vector<Eigen::Vector2d> points_in, Eigen::Vector2d centroid,
        std::string dir, std::vector<Eigen::Vector2d> &points_out);

    /** 
     * @brief gift_wrapping algorithm
     * @param points_in points that are passed into the algorithm
     * @param (Return) Vector of points that are at the edge (convex hull)
    **/
    std::vector<Eigen::Vector2d> gift_wrapping(
        std::vector<Eigen::Vector2d> points_in);

    // https://stackoverflow.com/a/43896965
    // This uses the ray-casting algorithm to decide whether the point is inside
    // the given polygon. See https://en.wikipedia.org/wiki/Point_in_polygon#Ray_casting_algorithm
    bool point_in_polygon(obstacle &poly, Eigen::Vector2d point);

    // https://stackoverflow.com/a/12132746
    void get_line(Eigen::Vector2d l1, Eigen::Vector2d l2, double &a, double &b, double &c);
    
    void get_point_to_line(
        Eigen::Vector2d p, Eigen::Vector2d l1, Eigen::Vector2d l2,
        double &distance, Eigen::Vector2d &closest_point);

    /** 
     * @brief get_line_plane_intersection
     * @param s_e Start and end pair
     * @param normal Normal of the plane
     * @param pop Point on plane
     * @param p (Return) Point of intersection
    **/
    bool get_line_plane_intersection(
        std::pair<Eigen::Vector3d, Eigen::Vector3d> s_e, 
        Eigen::Vector3d normal, Eigen::Vector3d pop, Eigen::Vector3d &p);

    /** 
     * @brief get_polygons_on_plane
     * @param g_m Pass in the global map
     * @param normal Normal of the plane
     * @param polygons (Return) Return the vector of flattened polygons
     * @param v (Return) Return the vertices in 3d (not transformed)
    **/
    void get_polygons_on_plane(
        global_map g_m, Eigen::Vector3d normal, 
        std::vector<obstacle> &polygons, std::vector<Eigen::Vector3d> &v,
        bool sorted);

    /**
     * @brief get_affine_transform
     * @param pos Translational position
     * @param rpy Euler angles
     * @param frame Coordinate frame used
     * @param (Return) Affine3d matrix
    **/
    Eigen::Affine3d get_affine_transform(
        Eigen::Vector3d pos, Eigen::Vector3d rpy, 
        std::string frame);

    class visibility
    {
        public:

            visibility(global_map _map, std::string _frame, int _div_angle) : 
                map(_map), frame(_frame), div_angle(_div_angle)
            {
                // no plane and height constrain
                constrain_type = 0;
                // std::cout << &frame << " " << &map << std::endl;
                // std::cout << &_frame << std::endl;
            }

            visibility(global_map _map, std::string _frame,
                std::pair<double, double> _height_constrain, int _div_angle) : 
                map(_map), frame(_frame), height_constrain(_height_constrain), 
                div_angle(_div_angle)
            {
                // no plane constrain
                constrain_type = 1;
            }

            visibility(global_map _map, std::string _frame,
                std::pair<double, double> _height_constrain,
                std::pair<Eigen::Vector2d, Eigen::Vector2d> _plane_constrain, 
                int _div_angle) : 
                map(_map), frame(_frame), height_constrain(_height_constrain),
                plane_constrain(_plane_constrain), div_angle(_div_angle)
            {
                // no plane and height constrain
                constrain_type = 2;
            }

            ~visibility(){}

            /** @brief Main loop **/
            void calculate_path(bool is_expanded_and_sorted);

            /** @brief Get the rotated polygon (obstacles) **/
            std::vector<obstacle> get_rotated_poly();

            /** @brief Get the calculated path **/
            std::vector<Eigen::Vector3d> get_path();

            /** @brief Get updated map **/
            global_map get_map();

        private:

            global_map map;

            int div_angle;

            std::vector<Eigen::Vector3d> path;
            std::vector<Eigen::Vector3d> debug_point_vertices;

            std::vector<obstacle> rot_polygons;
            
            std::string frame;

            uint8_t constrain_type;

            std::pair<double, double> height_constrain;
            std::pair<Eigen::Vector2d, Eigen::Vector2d> plane_constrain; 

            /** 
             * @brief Sum up from a range of unsigned int values
             * @param s start of range
             * @param e end of the range
            **/
            size_t sum_of_range(size_t s, size_t e);

            /** 
             * @brief find_nearest_distance_2d_polygons_and_fuse
             * Find out whether the nearest distance between the 2 polygons are within the threshold
             * If it is within, the 2 polygons will be fused
             * Uses a shortcut to derive the closest vertex pair, use the vector of the centroids and dot product
             * @param o1 Obstacle 1 input
             * @param o2 Obstacle 2 input
             * @param points_out (Return) Pair of points that are the closest 2 points between the 2 polygons
             * (Only will return points out if the edges are not parallel)
             * @param nearest_distance (Return) Nearest distance between the 2 polygons
             * @param o3 (Return) The fused obstacle
             * (Only will return points out if the edges are not parallel)
             * @param (Return) A boolean representing whether there is a fused polygon
            **/
            bool find_nearest_distance_2d_polygons_and_fuse(
                obstacle o1, obstacle o2,
                std::pair<Eigen::Vector2d, Eigen::Vector2d> &points_out, 
                double &nearest_distance, obstacle &o3);

            /** 
             * @brief closest_points_between_lines
             * @param a0 First point of line 1
             * @param a1 Second point of line 1
             * @param b0 First point of line 2
             * @param b1 Second point of line 2
             * @param c_p The pair of points that are the closest points
             * (Only will return points out if the edges are not parallel)
             * @param distance Nearest distance between the 2 lines
             * @param (Return) A boolean representing whether the line is not parallel
            **/
            bool closest_points_between_lines(
                Eigen::Vector2d a0, Eigen::Vector2d a1,
                Eigen::Vector2d b0, Eigen::Vector2d b1,
                std::pair<Eigen::Vector2d, Eigen::Vector2d> &c_p,
                double &distance);
            
            /** 
             * @brief set_2d_min_max_boundary
             * @param obstacles Vector of flattened obstacles
             * @param start_end Start and end flattened points
             * @param boundary (Return) The minimum and maximum of the inputs
            **/
            void set_2d_min_max_boundary(
                std::vector<obstacle> obstacles, std::pair<Eigen::Vector2d, Eigen::Vector2d> start_end, 
                std::pair<Eigen::Vector2d, Eigen::Vector2d> &boundary);

            /** 
             * @brief boundary_to_polygon_vertices
             * @param min_max The minimum and maximum of the inputs
             * @param dir Pass the direction into the graham search to sort the vertices
             * @param (Return) The AABB of the inputs represented in the 4 vertices
            **/
            std::vector<Eigen::Vector2d> boundary_to_polygon_vertices(
                std::pair<Eigen::Vector2d, Eigen::Vector2d> min_max, std::string dir);
        
            /** 
             * @brief get_expansion_of_obs
             * @param obs Obstacle as input and output 
             * @param inflation Inflation amount
            **/
            void get_expanded_obs(
                obstacle &obs, double inflation);

            void check_and_fuse_obstacles();

            void check_simple_obstacle_vertices(
                obstacle obs, double eps, size_t &valid_vert_size);            
            
    };
}

#endif