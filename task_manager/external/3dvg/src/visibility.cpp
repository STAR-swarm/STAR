/*
* visibility.cpp
*
* ---------------------------------------------------------------------
* Copyright (C) 2022 Matthew (matthewoots at gmail.com)
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

#include "visibility.h"

namespace visibility_graph
{
    /** 
     * @brief Sum up from a range of unsigned int values
     * @param s start of range
     * @param e end of the range
    **/
    size_t visibility::sum_of_range(size_t s, size_t e)
    {
        return e*(e+1)/2 - s*(s+1)/2 + s;
    }

    /** 
     * @brief gift_wrapping algorithm
     * @param points_in points that are passed into the algorithm
     * @param (Return) Vector of points that are at the edge (convex hull)
    **/
    std::vector<Eigen::Vector2d> gift_wrapping(
        std::vector<Eigen::Vector2d> points_in)
    {
        static const auto INVALID_VALUE = std::numeric_limits<size_t>::max();

        std::vector<Eigen::Vector2d> vect;
        size_t n = points_in.size();
        std::vector<size_t> next(n);
        size_t l = 0; //leftmost point
        for (size_t i = 0; i < n; i++)
        {
            next[i] = INVALID_VALUE;
            // Find the leftmost point
            if (points_in[i].x() < points_in[l].x())
                l = i;
        }
        size_t p = l;
        size_t q = 0; //Should be initialized when do loop below first runs
        size_t total_possible_attempts = (size_t)pow(n,2);
        size_t tries = 0;
        do
        {
            q = (p+1) % n;
            for (size_t i = 0; i < n; i++)
            {
                double val =
                    (points_in[i].y() - points_in[p].y()) * (points_in[q].x() - points_in[i].x()) -
                    (points_in[i].x() - points_in[p].x()) * (points_in[q].y() - points_in[i].y());
                // clockwise direction
                if (val > 0)
                    q = i;
            }
            next[p] = q;
            p = q;
            tries++;
        }
        while (p != l && tries < total_possible_attempts);

        for (size_t i = 0; i < n; i++)
        {
            if (next[i] != INVALID_VALUE)
                vect.push_back(points_in[i]);
        }

        return vect;

    }

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
        std::string dir, std::vector<Eigen::Vector2d> &points_out)
    {
        std::vector<std::pair<double, size_t>> point_angle_pair;
        size_t point_size = points_in.size();
        // Begin sorting by ordering using angles
        for (size_t i = 0; i < point_size; i++)
        {
            double angle;

            // check to make sure the angle won't be "0"
            if (points_in[i].x() == centroid.x())
                angle = 0.0;
            else
                angle = atan2((points_in[i].y() - centroid.y()), (points_in[i].x() - centroid.x()));

            point_angle_pair.push_back(
                std::make_pair(angle, i));
        }

        // Using simple sort() function to sort
        // By default it is 
        sort(point_angle_pair.begin(), point_angle_pair.end());

        points_out.clear();
        if (strcmp(dir.c_str(), "ccw") == 0)
        {
            for (int i = 0; i < point_size; i++) 
                points_out.push_back(points_in[point_angle_pair[i].second]);
        }
        else if (strcmp(dir.c_str(), "cw") == 0)
        {
            for (int i = point_size-1; i >= 0; i--) 
                points_out.push_back(points_in[point_angle_pair[i].second]);
        }

        // for (int i = 0; i < point_size; i++)
        //     printf("%lf %ld\n", point_angle_pair[i].first,
        //         point_angle_pair[i].second); 
    }

        // https://stackoverflow.com/a/43896965
    // This uses the ray-casting algorithm to decide whether the point is inside
    // the given polygon. See https://en.wikipedia.org/wiki/Point_in_polygon#Ray_casting_algorithm
    bool point_in_polygon(obstacle &poly, Eigen::Vector2d point)
    {
        // If we never cross any lines we're inside.
        bool inside = false;

        // Loop through all the edges.
        for (size_t i = 0; i < poly.v.size(); ++i)
        {
            // i is the index of the first vertex, j is the next one.
            // The original code uses a too-clever trick for this.
            int j = (i + 1) % (poly.v.size());

            // The vertices of the edge we are checking.
            double xp0 = poly.v[i].x();
            double yp0 = poly.v[i].y();
            double xp1 = poly.v[j].x();
            double yp1 = poly.v[j].y();

            // Check whether the edge intersects a line from (-inf,y) to (x,y).

            // First check if the line crosses the horizontal line at y in either direction.
            if ((yp0 <= point.y()) && (yp1 > point.y()) || (yp1 <= point.y()) && (yp0 > point.y()))
            {
                // If so, get the point where it crosses that line. This is a simple solution
                // to a linear equation. Note that we can't get a division by zero here -
                // if yp1 == yp0 then the above if will be false.
                double cross = (xp1 - xp0) * (point.y() - yp0) / (yp1 - yp0) + xp0;

                // Finally check if it crosses to the left of our test point. You could equally
                // do right and it should give the same result.
                if (cross < point.x())
                    inside = !inside;
            }
        }

        return inside;
    }

    // https://stackoverflow.com/a/12132746
    void get_line(Eigen::Vector2d l1, Eigen::Vector2d l2, double &a, double &b, double &c)
    {
        // (x- p1X) / (p2X - p1X) = (y - p1Y) / (p2Y - p1Y) 
        a = l1.y() - l2.y(); // Note: this was incorrectly "y2 - y1" in the original answer
        b = l2.x() - l1.x();
        c = l1.x() * l2.y() - l2.x() * l1.y();
    }
    
    void get_point_to_line(
        Eigen::Vector2d p, Eigen::Vector2d l1, Eigen::Vector2d l2,
        double &distance, Eigen::Vector2d &closest_point)
    {
        double a, b, c;
        get_line(l1, l2, a, b, c);
        // https://www.geeksforgeeks.org/find-foot-of-perpendicular-from-a-point-in-2-d-plane-to-a-line/?ref=rp
        double temp = -1 * (a * p.x() + b * p.y() + c) / (a * a + b * b);
        Eigen::Vector2d pp = Eigen::Vector2d(temp * a + p.x(), temp * b + p.y());

        Eigen::Vector2d dir1 = (pp - l1);
        Eigen::Vector2d dir2 = (pp - l2);

        // same direction, we should clamp at one of the end points
        if ((dir1.normalized()-dir2.normalized()).norm() < 0.0000001)
        {
            if (dir1.norm() < dir2.norm())
            {
                distance = dir1.norm();
                closest_point = l1;
            }
            else
            {
                distance = dir2.norm();
                closest_point = l2;
            }
        }
        else
        {
            distance = (pp - p).norm();
            closest_point = pp;
        }
    }

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
    bool visibility::find_nearest_distance_2d_polygons_and_fuse(
        obstacle o1, obstacle o2,
        std::pair<Eigen::Vector2d, Eigen::Vector2d> &points_out, 
        double &nearest_distance, obstacle &o3)
    {
        // The polygon vertices should be sorted
        // Use the euclidean distance to estimate the closest vertices
        std::pair<Eigen::Vector2d, Eigen::Vector2d> direction_pair;
        direction_pair.first = (o2.c - o1.c).normalized(); // o1 -> o2
        direction_pair.second = (o1.c - o2.c).normalized(); // o2 -> o1

        std::vector<std::pair<size_t,size_t>> i1, i2;
        double dot1 = 0.0, dot2 = 0.0;
        for (int i = 0; i < (int)o1.v.size(); i++)
        {
            std::pair<size_t,size_t> p_i;
            Eigen::Vector2d v1 = o1.v[i] - o1.c;
            double d1 =
                direction_pair.first.x() * v1.x() + direction_pair.first.y() * v1.y();

            // Same direction
            if (d1 > dot1)
            {
                i1.clear();
                p_i = std::make_pair(
                    ((i-2) < 0) ? i-2+o1.v.size() : i-2, 
                    ((i-1) < 0) ? i-1+o1.v.size() : i-1);
                i1.push_back(p_i);
                p_i = std::make_pair(
                    ((i-1) < 0) ? i-1+o1.v.size() : i-1, i);
                i1.push_back(p_i);
                p_i = std::make_pair(
                    i, (i+1)%o1.v.size());
                i1.push_back(p_i);
                p_i = std::make_pair(
                    (i+1)%o1.v.size(), (i+2)%o1.v.size());
                i1.push_back(p_i);

                dot1 = d1;
            }

        }

        for (int i = 0; i < (int)o2.v.size(); i++)
        {
            std::pair<size_t,size_t> p_i;
            Eigen::Vector2d v2 = o2.v[i] - o2.c;
            double d2 =
                direction_pair.second.x() * v2.x() + direction_pair.second.y() * v2.y();

            // Same direction
            if (d2 > dot2)
            {
                i2.clear();
                p_i = std::make_pair(
                    ((i-2) < 0) ? i-2+o2.v.size() : i-2, 
                    ((i-1) < 0) ? i-1+o2.v.size() : i-1);
                i2.push_back(p_i);
                p_i = std::make_pair(
                    ((i-1) < 0) ? i-1+o2.v.size() : i-1, i);
                i2.push_back(p_i);
                p_i = std::make_pair(
                    i, (i+1)%o2.v.size());
                i2.push_back(p_i);
                p_i = std::make_pair(
                    (i+1)%o2.v.size(), (i+2)%o2.v.size());
                i2.push_back(p_i);

                dot2 = d2;
            }
        }
        
        bool fuse = false;
        nearest_distance = map.inflation;

        for (std::pair<size_t,size_t> &idx1 : i1)
        {
            for (std::pair<size_t,size_t> &idx2 : i2)
            {
                std::pair<Eigen::Vector2d, Eigen::Vector2d> c_p;
                double dist;

                closest_points_between_lines(
                    o1.v[idx1.first], o1.v[idx1.second],
                    o2.v[idx2.first], o2.v[idx2.second],
                    c_p, dist);
                /** @brief For debug purpose **/
                // std::cout << "closest dist = " << dist << std::endl;

                if (dist < nearest_distance)
                {
                    // printf("fuse [%lf %lf -- %lf %lf] [%lf %lf -- %lf %lf]\n",
                    //     o1.v[idx1.first].x(), o1.v[idx1.first].y(),
                    //     o1.v[idx1.second].x(), o1.v[idx1.second].y(),
                    //     o2.v[idx2.first].x(), o2.v[idx2.first].y(), 
                    //     o2.v[idx2.second].x(), o2.v[idx2.second].y());
                    // printf("[%ld %ld] [%ld %ld] dist %lf\n", 
                    //     idx1.first, idx1.second,
                    //     idx2.first, idx2.second,
                    //     dist);

                    fuse = true;
                    nearest_distance = dist;
                    points_out = c_p;
                }
            }
        }        

        if (!fuse)
            return false;

        o3.h = std::make_pair(0.0, 0.0);

        // Append the 2 vectors
        std::vector<Eigen::Vector2d> tmp_vertices, new_vertices;
        tmp_vertices = o1.v;
        tmp_vertices.insert(tmp_vertices.end(), o2.v.begin(), o2.v.end());
        /** @brief For debug purpose **/
        // std::cout << "tmp_vertices.size() = " << (int)tmp_vertices.size() << std::endl;
        
        new_vertices = gift_wrapping(tmp_vertices);
        /** @brief For debug purpose **/
        // std::cout << "gift_wrapping.size() = " << (int)new_vertices.size() << std::endl;
        o3.c = get_centroid_2d(new_vertices);
        graham_scan(new_vertices, o3.c, "cw", o3.v);

        return true;
    }

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
    bool visibility::closest_points_between_lines(
        Eigen::Vector2d a0, Eigen::Vector2d a1,
        Eigen::Vector2d b0, Eigen::Vector2d b1,
        std::pair<Eigen::Vector2d, Eigen::Vector2d> &c_p,
        double &distance)
    {
        // Converted from Python code from
        // https://stackoverflow.com/a/18994296

        double epsilon = 0.000001;

        Eigen::Vector2d a = a1 - a0;
        Eigen::Vector2d b = b1 - b0;
        double m_a = a.norm();
        double m_b = b.norm();

        Eigen::Vector2d a_v = a.normalized();
        Eigen::Vector2d b_v = b.normalized();

        double cross = a_v.x() * b_v.y() - a_v.y() * b_v.x();
        double denom = pow(cross, 2);

        // If lines are parallel (denom=0) test if lines overlap.
        // If they don't overlap then there is a closest point solution.
        // If they do overlap, there are infinite closest positions, but there is a closest distance
        if (denom < epsilon)
        {
            // Lines are parallel, there can be closest points
            double d0 = a_v.x() * (b0-a0).x() + a_v.y() * (b0-a0).y();
            double d1 = a_v.x() * (b1-a0).x() + a_v.y() * (b1-a0).y();

            // Is segment B before A?
            if (d0 <= 0 && d1 <= 0)
            {
                if (abs(d0) < abs(d1))
                {
                    c_p.first = a0; c_p.second = b0;
                    distance = (a0-b0).norm();
                    return true;
                }
                else
                {
                    c_p.first = a0; c_p.second = b1;
                    distance = (a0-b1).norm();
                    return true;
                }
            }
            // Is segment B after A?
            else if (d0 >= m_a && d1 >= m_a)
            {
                if (abs(d0) < abs(d1))
                {
                    c_p.first = a1; c_p.second = b0;
                    distance = (a1-b0).norm();
                    return true;
                }
                else
                {
                    c_p.first = a1; c_p.second = b1;
                    distance = (a1-b1).norm();
                    return true;
                }
            }
            // Segments overlap, return distance between parallel segments
            else
            {
                distance = (((d0 * a_v) + a0) - b0).norm();
                return false;
            }
        }
        
        // Lines criss-cross: Calculate the projected closest points
        Eigen::Vector2d t = b0 - a0;
        double det_a = t.x() * a_v.y() - t.y() * a_v.x();
        double det_b = t.x() * b_v.y() - t.y() * b_v.x();

        double t0 = det_a / denom;
        double t1 = det_b / denom;

        Eigen::Vector2d p_a = a0 + (a_v * t0); // Projected closest point on segment A
        Eigen::Vector2d p_b = b0 + (b_v * t1); // Projected closest point on segment B

        if (t0 < 0)
            p_a = a0;
        else if (t0 > m_a)
            p_a = a1;
        
        if (t1 < 0)
            p_b = b0;
        else if (t1 > m_b)
            p_b = b1;

        // Clamp projection A
        if (t0 < 0.0 || t0 > m_a)
        {            
            double dot = 
                b_v.x() * (p_a-b0).x() + b_v.y() * (p_a-b0).y();
            if (dot < 0.0)
                dot = 0;
            else if (dot > m_b)
                dot = m_b;
            p_b = b0 + (b_v * dot);
        }
        
        
        // Clamp projection B
        if (t1 < 0.0 || t1 > m_b)
        {
            double dot = 
                a_v.x() * (p_b-a0).x() + a_v.y() * (p_b-a0).y();
            if (dot < 0.0)
                dot = 0;
            else if (dot > m_a)
                dot = m_a;
            p_a = a0 + (a_v * dot);
        }

        c_p.first = p_a; c_p.second = p_b;
        distance = (p_a - p_b).norm();

        return true;
    }

    /** 
     * @brief set_2d_min_max_boundary
     * @param obstacles Vector of flattened obstacles
     * @param start_end Start and end flattened points
     * @param boundary (Return) The minimum and maximum of the inputs
    **/
    void visibility::set_2d_min_max_boundary(
        std::vector<obstacle> obstacles, std::pair<Eigen::Vector2d, Eigen::Vector2d> start_end, std::pair<Eigen::Vector2d, Eigen::Vector2d> &boundary)
    {
        // std::vector<Eigen::Vector2d> query_vector;
        // for (obstacle &obs : obstacles)
        // {
        //     for (size_t j = 0; j < obs.v.size(); j++)
        //         query_vector.push_back(obs.v[j]);
        // }
        // query_vector.push_back(start_end.first);
        // query_vector.push_back(start_end.second);

        Eigen::Vector2d max(-1000.0, -1000.0), min(1000.0, 1000.0);
        // Eigen::Vector2d max(-5.0, -10.0), min(5.0, 10.0);
        // for (Eigen::Vector2d &v : query_vector)
        // {
        //     if (v.x() < min.x())
        //         min.x() = v.x();
        //     if (v.x() > max.x())
        //         max.x() = v.x();
            
        //     if (v.y() < min.y())
        //         min.y() = v.y();
        //     if (v.y() > max.y())
        //         max.y() = v.y();
        // }

        boundary.first = min;
        boundary.second = max;
    }

    /** 
     * @brief boundary_to_polygon_vertices
     * @param min_max The minimum and maximum of the inputs
     * @param dir Pass the direction into the graham search to sort the vertices
     * @param (Return) The AABB of the inputs represented in the 4 vertices
    **/
    std::vector<Eigen::Vector2d> visibility::boundary_to_polygon_vertices(
        std::pair<Eigen::Vector2d, Eigen::Vector2d> min_max, std::string dir)
    {
        static const auto BOUNDARY_EXPANSION = 0.0;

        std::vector<Eigen::Vector2d> tmp, vect;
        // Establish 4 corners
        tmp.push_back(min_max.first - Eigen::Vector2d(BOUNDARY_EXPANSION,BOUNDARY_EXPANSION));
        tmp.push_back(min_max.second + Eigen::Vector2d(BOUNDARY_EXPANSION,BOUNDARY_EXPANSION));
        tmp.push_back(Eigen::Vector2d(min_max.first.x() - BOUNDARY_EXPANSION, min_max.second.y() + BOUNDARY_EXPANSION));
        tmp.push_back(Eigen::Vector2d(min_max.second.x() + BOUNDARY_EXPANSION, min_max.first.y() - BOUNDARY_EXPANSION));

        graham_scan(tmp, Eigen::Vector2d(
            (min_max.first.x() + min_max.second.x()) / 2, (min_max.first.y() + min_max.second.y()) / 2), 
            dir, vect);

        return vect;
    }

    /** 
     * @brief get_line_plane_intersection
     * @param s_e Start and end pair
     * @param normal Normal of the plane
     * @param pop Point on plane
     * @param p (Return) Point of intersection
    **/
    bool get_line_plane_intersection(
        std::pair<Eigen::Vector3d, Eigen::Vector3d> s_e, 
        Eigen::Vector3d normal, Eigen::Vector3d pop, Eigen::Vector3d &p)
    {
        // https://stackoverflow.com/a/23976134
        double epsilon = 0.0001;
        double t;

        Eigen::Vector3d ray_raw = s_e.second - s_e.first;
        Eigen::Vector3d ray = ray_raw.normalized();
        double ray_dist = ray_raw.norm(); 
        Eigen::Vector3d ray_to_p = pop - s_e.first; 
        double d_rp_n = normal.x() * ray_to_p.x() + normal.y() * ray_to_p.y() + normal.z() * ray_to_p.z();
        double d_n_r = normal.x() * ray.x() + normal.y() * ray.y() + normal.z() * ray.z();
        
        if (abs(d_n_r) > epsilon)
        {
            t = d_rp_n / d_n_r;
            if (t < 0 || t > ray_dist) 
                return false; 
        }
        else
            return false;
        
        p = s_e.first + ray * t; 
        /** @brief For debug purpose **/
        // std::cout << "p = " << p.transpose() << " t = " << t << std::endl;

        return true;
    }

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
        bool sorted)
    {        
        v.clear();
        
        // https://karlobermeyer.github.io/VisiLibity1/doxygen_html/class_visi_libity_1_1_environment.html
        // the outer boundary vertices must be listed ccw and the hole vertices cw 
        for (obstacle &obs : g_m.obs)
        {
            std::vector<Eigen::Vector2d> vert;             

            int obs_vert_size = obs.v.size();
            for (int i = 0; i < obs_vert_size; i++)
            {
                Eigen::Vector3d o_vert;
                std::pair<Eigen::Vector3d, Eigen::Vector3d> vert_pair;
                vert_pair.first = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), obs.h.first);
                vert_pair.second = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), obs.h.second);

                if (get_line_plane_intersection(vert_pair, normal, g_m.start_end.first, o_vert))
                {
                    // Transform original vertex into 2d, o_vert to t_vert
                    v.push_back(o_vert);

                    Eigen::Vector3d t_vert = g_m.t * o_vert;
                    /** @brief For debug purpose **/
                    // std::cout << i << " = " <<  t_vert.transpose() << std::endl;
                    
                    vert.push_back(Eigen::Vector2d(t_vert.x(), t_vert.y()));
                }
            }
            // Top and bottom plane
            for (int i = 0; i < obs_vert_size-1; i++)
            {
                Eigen::Vector3d o_vert;
                std::pair<Eigen::Vector3d, Eigen::Vector3d> vert_pair;
                vert_pair.first = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), obs.h.first);
                vert_pair.second = Eigen::Vector3d(obs.v[i+1].x(), obs.v[i+1].y(), obs.h.first);

                if (get_line_plane_intersection(vert_pair, normal, g_m.start_end.first, o_vert))
                {
                    // Transform original vertex into 2d, o_vert to t_vert
                    v.push_back(o_vert);

                    Eigen::Vector3d t_vert = g_m.t * o_vert;
                    /** @brief For debug purpose **/
                    // std::cout << t_vert.transpose() << std::endl;
                    
                    vert.push_back(Eigen::Vector2d(t_vert.x(), t_vert.y()));
                }            

                vert_pair.first = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), obs.h.second);
                vert_pair.second = Eigen::Vector3d(obs.v[i+1].x(), obs.v[i+1].y(), obs.h.second);

                if (get_line_plane_intersection(vert_pair, normal, g_m.start_end.first, o_vert))
                {
                    // Transform original vertex into 2d, o_vert to t_vert
                    v.push_back(o_vert);

                    Eigen::Vector3d t_vert = g_m.t * o_vert;
                    /** @brief For debug purpose **/
                    // std::cout << t_vert.transpose() << std::endl;
                    
                    vert.push_back(Eigen::Vector2d(t_vert.x(), t_vert.y()));
                }
            }

            if (vert.empty())
                continue;

            if (vert.size() < 3)
                continue; //Not enough points to make a polygon

            obs.c = get_centroid_2d(vert);
            
            // No height data since its a flat plane
            obs.h = std::make_pair(0.0, 0.0);

            // Organize vertices of holes in clockwise format
            if (!sorted)
                graham_scan(vert, obs.c, "cw", obs.v);
            else 
                obs.v = vert;

            polygons.push_back(obs);
        }
    }

    /** 
     * @brief get_expansion_of_obs
     * @param obs Obstacle as input and output 
     * @param inflation Inflation amount
    **/
    void visibility::get_expanded_obs(obstacle &obs, double inflation)
    {
        obstacle tmp = obs;
        for (size_t i = 0; i < tmp.v.size(); i++)
        {
            // Write over the obstacle vertices 
            Eigen::Vector2d norm_vect = (tmp.v[i] - tmp.c).normalized();
            obs.v[i] = tmp.v[i] + norm_vect * inflation;
        }
    }

    /** 
     * @brief get_centroid_2d
     * @param vect Vector of points used to get centroid
     * @param (Return) Centroid
    **/
    Eigen::Vector2d get_centroid_2d(
        std::vector<Eigen::Vector2d> vert)
    {
        int point_size = (int)vert.size();
        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        for (Eigen::Vector2d &p : vert)
            centroid += p;
        
        return centroid/point_size;
    }

    /** 
     * @brief get_rotation
     * @param rpy Euler angles in Eigen::Vector3d
     * @param frame Coordinate frame used
     * @param (Return) 3x3 Rotation matrix
    **/
    Eigen::Matrix3d get_rotation(
        Eigen::Vector3d rpy, std::string frame)
    {
        Eigen::Vector3d orientated_rpy;
        // y is pitch, and RHR indicates positive to be anticlockwise (z is neg)
        // Hence to counter pitch direction we can leave rpy.y() positive
        // z is yaw, and RHR indicates positive to be anticlockwise (y is pos)
        // Hence to counter yaw direction we need to make rpy.z() negative
        if (strcmp(frame.c_str(), "nwu") == 0)
            orientated_rpy = Eigen::Vector3d(rpy.x(), rpy.y(), -rpy.z());
        // x is pitch, and RHR indicates positive to be anticlockwise (z is pos)
        // Hence to counter pitch direction we need to make rpy.x() negative
        // z is yaw, and RHR indicates positive to be anticlockwise (y is pos)
        // Hence to counter yaw direction we need to make rpy.z() negative
        else if (strcmp(frame.c_str(), "enu") == 0)
            orientated_rpy = Eigen::Vector3d(0.0, rpy.x(), -rpy.z());
        // Not implemented for enu yet

        // Get rotation matrix from RPY
        // https://stackoverflow.com/a/21414609
        Eigen::AngleAxisd rollAngle(orientated_rpy.x(), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(orientated_rpy.y(), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(orientated_rpy.z(), Eigen::Vector3d::UnitZ());

        Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;

        return q.matrix();
    }

    /**
     * @brief get_affine_transform
     * @param pos Translational position
     * @param rpy Euler angles
     * @param frame Coordinate frame used
     * @param (Return) Affine3d matrix
    **/
    Eigen::Affine3d get_affine_transform(
        Eigen::Vector3d pos, Eigen::Vector3d rpy, 
        std::string frame)
    {
        Eigen::Affine3d affine;
        Eigen::Matrix3d rot = get_rotation(rpy, frame);
        Eigen::Vector3d rot_pos = rot * -pos;

        affine.translation() = rot_pos;
        affine.linear() = rot;

        return affine;
    }

    /** @brief Main loop **/
    void visibility::calculate_path(bool is_expanded_and_sorted)
    {
        t_p_sc start_time = system_clock::now();

        debug_point_vertices.clear();
        path.clear();

        Eigen::Vector3d direction = map.start_end.second - map.start_end.first;
        direction.normalized();

        
        double div_angle_vector = M_PI / (double)div_angle;
        double plane_angle = 0.0;

        // std::cout << &frame << " " << &map << std::endl;

        std::map<double, VisiLibity::Polyline> shortest_path_vector;
        // Number of solutions given the plane division
        // for (int i = 0; i < div_angle; i++)
        // {
            if (strcmp(frame.c_str(), "nwu") == 0)
            {
                double yaw = atan2(direction.y(), direction.x());
                Eigen::Vector2d h_xy = Eigen::Vector2d(direction.x(), direction.y());
                double length_h_xy = h_xy.norm();
                double pitch = atan2(direction.z(), length_h_xy);
                
                map.rpy = Eigen::Vector3d(plane_angle, pitch, yaw);

                map.t = get_affine_transform(map.start_end.first, map.rpy, frame);
            }

            else if (strcmp(frame.c_str(), "enu") == 0)
            {
                double yaw = atan2(direction.y(), direction.x());
                Eigen::Vector2d h_xy = Eigen::Vector2d(direction.x(), direction.y());
                double length_h_xy = h_xy.norm();
                double pitch = atan2(direction.z(), length_h_xy);
                map.rpy = Eigen::Vector3d(pitch, 0.0, yaw);
                map.t = get_affine_transform(map.start_end.first, map.rpy, frame);
            }

            std::pair<Eigen::Vector3d, Eigen::Vector3d> rot_pair;
            rot_pair.first = map.t * map.start_end.first;
            rot_pair.second = map.t * map.start_end.second;
            std::pair<Eigen::Vector2d, Eigen::Vector2d> rot_pair_2d;
            rot_pair_2d.first = Eigen::Vector2d(rot_pair.first.x(), rot_pair.first.y());
            rot_pair_2d.second = Eigen::Vector2d(rot_pair.second.x(), rot_pair.second.y());

            double setup_time = 
                duration<double>(system_clock::now() - start_time).count();

            /** @brief Check transform **/
            std::cout << "original = " << (map.t.inverse() * rot_pair.first).transpose() << " to " << 
                 (map.t.inverse() * rot_pair.second).transpose() << std::endl;
            std::cout << "transformed = " << rot_pair.first.transpose() << " to " << 
                rot_pair.second.transpose() << std::endl;

            // Get the plane normal
            Eigen::Vector3d normal = 
                get_rotation(map.rpy, frame).inverse() * Eigen::Vector3d(0.0, 0.0, 1.0);

            rot_polygons.clear();
            get_polygons_on_plane(
                map, normal, rot_polygons, debug_point_vertices, 
                is_expanded_and_sorted);

            double fuse_time = 0.0;

            if (!is_expanded_and_sorted)
            {
                // Join the existing obstacles
                check_and_fuse_obstacles();

                for (size_t i = 0; i < rot_polygons.size(); i++)
                {
                    // Expand the obstacles first
                    get_expanded_obs(rot_polygons[i], map.inflation);

                    /** @brief For debug purpose **/
                    // std::cout << i << " polygon_vertices " << (int)rot_polygons[i].v.size() << std::endl;
                }

                // Join the expanded obstacles
                check_and_fuse_obstacles();

                fuse_time = 
                    duration<double>(system_clock::now() - start_time).count() -
                    setup_time;
            }

            /** @brief For debug purpose **/
            for (Eigen::Vector3d &p : debug_point_vertices)
                std::cout << p.transpose() << std::endl;
            std::cout << std::endl;

            //std::vector<VisiLibity::Polygon> vector_polygon;

            // Intersect with the global boundary
            // switch (constrain_type)
            // {
            //     // height constrains
            //     case 1:
            //         break;

            //     // all (height and plane) constrains
            //     case 2:
            //         break;

            //     // Without any constrains
            //     case 0: default:
            //         break;
            // }

            // Create the polygon for boundary
            printf("creating boundary_to_polygon_vertices\n");
            std::pair<Eigen::Vector2d, Eigen::Vector2d> min_max;
            set_2d_min_max_boundary(rot_polygons, rot_pair_2d, min_max);
            std::vector<Eigen::Vector2d> boundary =
                boundary_to_polygon_vertices(min_max, "ccw");
            printf("boundary_to_polygon_vertices\n");

            VisiLibity::Polygon boundary_polygon;
            std::vector<VisiLibity::Point> boundary_vertices;
            for (Eigen::Vector2d &p : boundary)
            {
                VisiLibity::Point vis_vert(p.x(), p.y());
                boundary_vertices.push_back(vis_vert);
            }
            boundary_polygon.set_vertices(boundary_vertices);
            printf("set boundary vertices\n");

            // Add obstacles to environment
            static const auto VISILIBITY_EPSILON = 0.01;

            VisiLibity::Environment my_environment;
            my_environment.set_outer_boundary(boundary_polygon);
            printf("set outer boundary\n");

            if (!rot_polygons.empty())
            {
                // size_t i = 0;
                // Create the polygons for holes
                for (obstacle &poly : rot_polygons)
                {
                    VisiLibity::Polygon polygon;

                    size_t poly_size;
                    // Epsilon will trigger if the points are too close
                    // run proximity check for VISILIBITY_EPSILON
                    check_simple_obstacle_vertices(
                        poly, VISILIBITY_EPSILON, poly_size);
                    
                    // if (!poly.v.empty() && poly_size >= 3)
                    if (poly_size >= 3)
                    {
                        // std::cout << "create " << i << " polygon_vertices " << (int)poly.v.size() << std::endl;
                        /** @brief For debug purpose **/
                        // printf("poly_vert_size %d\n",(int)poly.v.size());

                        for (size_t i = 0; i < poly.v.size(); i++)
                            polygon.push_back(
                                VisiLibity::Point(poly.v[i].x(), poly.v[i].y()));

                        // polygon.eliminate_redundant_vertices(VISILIBITY_EPSILON);
                        polygon.enforce_standard_form();

                        if (polygon.n() < 3)
                            continue;
                        if (polygon.area() >= 0)
                            continue;

                        /** @brief For debug purpose **/
                        printf("polygon_size %d, standard %s, simple %s\n", 
                            polygon.n(), polygon.is_in_standard_form() ? "y" : "n",
                            polygon.is_simple(VISILIBITY_EPSILON) ? "y" : "n");
                        
                        // my_environment.add_hole(polygon);
                        // vector_polygon.push_back(polygon);

                        if (polygon.is_simple(VISILIBITY_EPSILON)) // Sometimes eliminate_redundant_vertices reduces vertices to invalid number
                            my_environment.add_hole(polygon);
                        else
                            continue;
                    }
                    else
                        continue;

                    // check whether start point is inside this polygon 
                    if (visibility_graph::point_in_polygon(poly, rot_pair_2d.first))
                    {
                        double distance = FLT_MAX;
                        Eigen::Vector2d closest_point;
                        for (size_t i = 0; i < poly.v.size(); i++)
                        {
                            Eigen::Vector2d cp;
                            double d;
                            size_t j = (i + 1) % (poly.v.size());
                            get_point_to_line(rot_pair_2d.first, 
                                poly.v[i], poly.v[j], d, cp);
                            if (d < distance)
                            {
                                distance = d;
                                closest_point = cp;
                            }
                        }
                        closest_point += (closest_point - poly.c).normalized() * 0.1;
                        path.emplace_back(
                            map.t.inverse() * Eigen::Vector3d(
                            rot_pair_2d.first.x(), rot_pair_2d.first.y(), 0.0));
                        rot_pair_2d.first.x() = closest_point.x();
                        rot_pair_2d.first.y() = closest_point.y();
                    }
                    // check whether end point is inside this polygon 
                    if (visibility_graph::point_in_polygon(poly, rot_pair_2d.second))
                    {
                        double distance = FLT_MAX;
                        Eigen::Vector2d closest_point;
                        for (size_t i = 0; i < poly.v.size(); i++)
                        {
                            Eigen::Vector2d cp;
                            double d;
                            size_t j = (i + 1) % (poly.v.size());
                            get_point_to_line(rot_pair_2d.second, 
                                poly.v[i], poly.v[j], d, cp);
                            if (d < distance)
                            {
                                distance = d;
                                closest_point = cp;
                            }
                        }
                        closest_point += (closest_point - poly.c).normalized() * 0.1;
                        rot_pair_2d.second.x() = closest_point.x();
                        rot_pair_2d.second.y() = closest_point.y();
                    }
                }
            }
            else
                printf("empty environment\n");

            assert(my_environment.is_valid(VISILIBITY_EPSILON));
            // if (my_environment.is_valid(VISILIBITY_EPSILON))
            // {
            //     printf("my_environment is not valid\n");
            //     return;
            // }

            t_p_sc v_g = system_clock::now();
            VisiLibity::Polyline shortest_path_poly;
            VisiLibity::Point start_vis(rot_pair_2d.first.x(), rot_pair_2d.first.y());
            VisiLibity::Point end_vis(rot_pair_2d.second.x(), rot_pair_2d.second.y());
            shortest_path_poly = my_environment.shortest_path(start_vis, end_vis, VISILIBITY_EPSILON);
            double v_g_time = 
                duration<double>(system_clock::now() - v_g).count();

            shortest_path_vector.insert({
                shortest_path_poly.length(), shortest_path_poly});

            plane_angle += div_angle_vector;

            for (size_t i = 0; i < shortest_path_poly.size(); i++)
            {
                VisiLibity::Point point = shortest_path_poly[(unsigned int)i];
                path.push_back(map.t.inverse() * Eigen::Vector3d(
                    point.x(), point.y(), 0.0));
            }
        // }

        // while (1)
        // {
        //     for (size_t i = 0; i < shortest_path_poly.size(); i++)
        //     {
        //         VisiLibity::Point point = shortest_path_poly[(unsigned int)i];
        //         path.push_back(map.t.inverse() * Eigen::Vector3d(
        //             point.x(), point.y(), 0.0));
        //     }
        // }

        printf("setup %.3lfms, fuse %.3lfms, calc %.3lf\n", 
            setup_time * 1000.0, fuse_time * 1000.0, v_g_time * 1000.0);

        return;
    }

    void visibility::check_and_fuse_obstacles()
    {
        size_t count = 0, check_size = 1;
        auto total_tries = sum_of_range(1, rot_polygons.size()-1);
        int tries = 0;
        while (count != check_size && tries < pow(total_tries,1.25))
        {
            size_t poly_size = rot_polygons.size();
            check_size = sum_of_range(1, poly_size-1);
            count = 0;

            for (size_t i = 0; i < poly_size; i++)
            {
                bool early_break = false;
                for (size_t j = 0; j < poly_size; j++)
                {
                    if (i <= j)
                        continue;
                    
                    std::pair<Eigen::Vector2d, Eigen::Vector2d> p_o;
                    double n_d;
                    obstacle o3;
                    if (rot_polygons[i].v.empty() || rot_polygons[j].v.empty())
                    {
                        count++;
                        continue;
                    }

                    if (find_nearest_distance_2d_polygons_and_fuse(
                        rot_polygons[i], rot_polygons[j], p_o, n_d, o3))
                    {
                        /** @brief For debug purpose **/
                        // std::cout << "fuse" << std::endl;

                        std::vector<obstacle> tmp = rot_polygons;
                        rot_polygons.clear(); 
                        for (size_t k = 0; k < poly_size; k++)
                        {
                            if (k != i && k != j)
                                rot_polygons.push_back(tmp[k]);
                        }
                        rot_polygons.push_back(o3);
                        early_break = true;
                        break;
                    }
                    count++;
                }
                if (early_break)
                    break;
            }

            tries++;
            /** @brief For debug purpose **/
            // std::cout << count << "/" << check_size << "/" << poly_size << std::endl;
        }

        // if (tries >= pow(total_tries,1.25))
        //     std::cout << "[error] early break!" << std::endl;
    }

    void visibility::check_simple_obstacle_vertices(
        obstacle obs, double eps, size_t &valid_vert_size)
    {
        valid_vert_size = obs.v.size(); 
        for (size_t i = 0; i < obs.v.size(); i++)
        {
            for (size_t j = 0; j < obs.v.size(); j++)
            {
                if (j <= i)
                    continue;
                
                if ((obs.v[i] - obs.v[j]).norm() < (eps * 1.1))
                    valid_vert_size -= 1;
            }
        }
    }

    /** @brief Get the rotated polygon (obstacles) **/
    std::vector<obstacle> visibility::get_rotated_poly()
    {
        return rot_polygons;
    }
    
    /** @brief Get the calculated path **/
    std::vector<Eigen::Vector3d> visibility::get_path()
    {
        return path;
    }

    /** @brief Get updated map **/
    global_map visibility::get_map()
    {
        return map;
    }
}