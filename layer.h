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

#ifndef LAYER_H
#define LAYER_H

#include "mymath.h"
#include <memory>

extern double length_2d(const point_2d &p);
extern double det_2d(const point_2d &p1, const point_2d &p2);
extern double dot_2d(const point_2d &p1, const point_2d &p2);
extern point_2d sub_2d(const point_2d &p1, const point_2d &p2);
extern point_2d perp_2d(const point_2d &p);
extern point_2d scale_2d(const point_2d &p, double s);
extern bool collide_thick_lines_2d(const point_2d &tl1_p1, const point_2d &tl1_p2,
	 					const point_2d &tl2_p1, const point_2d &tl2_p2, double r);

//layer class
class layer
{
public:
	struct dims
	{
		int m_width;
		int m_height;
	};

	struct line
	{
		bool operator==(const line &l) const
		{
			return std::tie(m_p1, m_p2, m_radius, m_gap) == std::tie(l.m_p1, l.m_p2, l.m_radius, l.m_gap);
		}
		point_2d m_p1;
		point_2d m_p2;
		double m_radius;
		double m_gap;
	};

	struct record
	{
		record(int id, const line &l)
		 	: m_id(id)
			, m_line(l)
		{
			auto pv = perp_2d(sub_2d(l.m_p2, l.m_p1));
			auto len = length_2d(pv);
			m_lv_norm = scale_2d(pv, 1.0 / len);
			m_lv_dist = dot_2d(m_lv_norm, l.m_p1);

			m_pv_norm = perp_2d(m_lv_norm);
			m_pv_dist1 = dot_2d(m_pv_norm, l.m_p1);
			m_pv_dist2 = m_pv_dist1 - len;
		}
		auto hit(const line &l, double d)
		{
			auto dp1 = dot_2d(m_lv_norm, l.m_p1) - m_lv_dist;
			auto dp2 = dot_2d(m_lv_norm, l.m_p2) - m_lv_dist;
			if (dp1 > d && dp2 > d) return false;
			if (dp1 < -d && dp2 < -d) return false;

			dp1 = dot_2d(m_pv_norm, l.m_p1);
			dp2 = dot_2d(m_pv_norm, l.m_p2);
			if (dp1 - m_pv_dist1 > d && dp2 - m_pv_dist1 > d) return false;
			if (dp1 - m_pv_dist2 < -d && dp2 - m_pv_dist2 < -d) return false;

			return collide_thick_lines_2d(l.m_p1, l.m_p2, m_line.m_p1, m_line.m_p2, d);
		}
		int m_id;
		line m_line;
		point_2d m_lv_norm;
		double m_lv_dist;

		point_2d m_pv_norm;
		double m_pv_dist1;
		double m_pv_dist2;
	};

	struct aabb
	{
		int m_minx;
		int m_miny;
		int m_maxx;
		int m_maxy;
	};

	typedef std::vector<std::shared_ptr<record>> bucket;
	typedef std::vector<bucket> buckets;

	layer(const dims &dims, double s);
	~layer();
	aabb get_aabb(const line &l);
	void add_line(const line &l);
	void sub_line(const line &l);
	bool hit_line(const line &l);

private:
	int m_width;
	int m_height;
	double m_scale;
	buckets m_buckets;
	int m_test;
};

//layers class
class layers
{
public:
	struct dims
	{
		int m_width;
		int m_height;
		int m_depth;
	};

	struct line
	{
		point_3d m_p1;
		point_3d m_p2;
		double m_radius;
		double m_gap;
	};

	layers(const dims &dims, double s);
	~layers();
	void add_line(const point_3d &p1, const point_3d &p2, double r, double g);
	void sub_line(const point_3d &p1, const point_3d &p2, double r, double g);
	bool hit_line(const point_3d &p1, const point_3d &p2, double r, double g);
	void add_line(const line &l);
	void sub_line(const line &l);
	bool hit_line(const line &l);

private:
	int m_depth;
	std::vector<std::shared_ptr<layer>> m_layers;
};

#endif
