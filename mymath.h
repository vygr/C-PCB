/*
    C-PCB
    Copyright (C) 2015 Chris Hinsley
	chris (dot) hinsley (at) gmail (dot) com

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#ifndef MYMATH_H
#define MYMATH_H

#include <vector>
#include <tuple>

//public math structures

struct point_2d
{
	point_2d() : m_x(0.0), m_y(0.0) {}
	point_2d(double x, double y) : m_x(x), m_y(y) {}
	bool operator==(const point_2d &p) const {
		return std::tie(m_x, m_y) == std::tie(p.m_x, p.m_y); }
	bool operator!=(const point_2d &p) const {
		return std::tie(m_x, m_y) != std::tie(p.m_x, p.m_y); }
	bool operator<(const point_2d &p) const {
		return std::tie(m_x, m_y) < std::tie(p.m_x, p.m_y); }
	double m_x;
	double m_y;
};

typedef std::vector<point_2d> points_2d;

struct point_3d
{
	point_3d() : m_x(0.0), m_y(0.0), m_z(0.0) {}
	point_3d(double x, double y, double z) : m_x(x), m_y(y), m_z(z) {}
	bool operator==(const point_3d &p) const {
		return std::tie(m_x, m_y, m_z) == std::tie(p.m_x, p.m_y, p.m_z); }
	bool operator!=(const point_3d &p) const {
		return std::tie(m_x, m_y, m_z) != std::tie(p.m_x, p.m_y, p.m_z); }
	bool operator<(const point_3d &p) const {
		return std::tie(m_x, m_y, m_z) < std::tie(p.m_x, p.m_y, p.m_z); }
	double m_x;
	double m_y;
	double m_z;
};

typedef std::vector<point_3d> points_3d;

#endif
