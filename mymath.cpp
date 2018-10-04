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

#include "mymath.h"
#include <math.h>

///////////////////////
//distance metric stuff
///////////////////////

auto manhattan_distance_2d(const point_2d &p1, const point_2d &p2)
{
	auto dx = p1.m_x - p2.m_x;
	auto dy = p1.m_y - p2.m_y;
	return double(fabs(dx) + fabs(dy));
}

auto manhattan_distance_3d(const point_3d &p1, const point_3d &p2)
{
	auto dx = p1.m_x - p2.m_x;
	auto dy = p1.m_y - p2.m_y;
	auto dz = p1.m_z - p2.m_z;
	return double(fabs(dx) + fabs(dy) + fabs(dz));
}

auto euclidean_distance_2d(const point_2d &p1, const point_2d &p2)
{
	auto dx = p1.m_x - p2.m_x;
	auto dy = p1.m_y - p2.m_y;
	return double(sqrt(dx * dx + dy * dy));
}

auto euclidean_distance_3d(const point_3d &p1, const point_3d &p2)
{
	auto dx = p1.m_x - p2.m_x;
	auto dy = p1.m_y - p2.m_y;
	auto dz = p1.m_z - p2.m_z;
	return double(sqrt(dx * dx + dy * dy + dz * dz));
}

auto squared_euclidean_distance_2d(const point_2d &p1, const point_2d &p2)
{
	auto dx = p1.m_x - p2.m_x;
	auto dy = p1.m_y - p2.m_y;
	return double(dx * dx + dy * dy);
}

auto squared_euclidean_distance_3d(const point_3d &p1, const point_3d &p2)
{
	auto dx = p1.m_x - p2.m_x;
	auto dy = p1.m_y - p2.m_y;
	auto dz = p1.m_z - p2.m_z;
	return double(dx * dx + dy * dy + dz * dz);
}

auto chebyshev_distance_2d(const point_2d &p1, const point_2d &p2)
{
	auto dx = fabs(p1.m_x - p2.m_x);
	auto dy = fabs(p1.m_y - p2.m_y);
	return std::max(dx, dy);
}

auto chebyshev_distance_3d(const point_3d &p1, const point_3d &p2)
{
	auto dx = fabs(p1.m_x - p2.m_x);
	auto dy = fabs(p1.m_y - p2.m_y);
	auto dz = fabs(p1.m_z - p2.m_z);
	auto d = std::max(dx, dy);
	return std::max(d, dz);
}

auto reciprical_distance_2d(const point_2d &p1, const point_2d &p2)
{
	auto d = manhattan_distance_2d(p1, p2);
	if (d == 0.0) return 1.0;
	return 1.0 / d;
}

auto reciprical_distance_3d(const point_3d &p1, const point_3d &p2)
{
	auto d = manhattan_distance_3d(p1, p2);
	if (d == 0.0) return 1.0;
	return 1.0 / d;
}

///////////////////////
//specific vector stuff
///////////////////////

auto add_2d(const point_2d &p1, const point_2d &p2)
{
	return point_2d(p1.m_x + p2.m_x, p1.m_y + p2.m_y);
}

auto sub_2d(const point_2d &p1, const point_2d &p2)
{
	return point_2d(p1.m_x - p2.m_x, p1.m_y - p2.m_y);
}

auto sub_3d(const point_3d &p1, const point_3d &p2)
{
	return point_3d(p1.m_x - p2.m_x, p1.m_y - p2.m_y, p1.m_z - p2.m_z);
}

auto scale_2d(const point_2d &p, double s)
{
	return point_2d(p.m_x * s, p.m_y * s);
}

auto scale_3d(const point_3d &p, double s)
{
	return point_3d(p.m_x * s, p.m_y * s, p.m_z * s);
}

auto perp_2d(const point_2d &p)
{
	return point_2d(-p.m_y, p.m_x);
}

auto dot_2d(const point_2d &p1, const point_2d &p2)
{
	return p1.m_x * p2.m_x + p1.m_y * p2.m_y;
}

auto det_2d(const point_2d &p1, const point_2d &p2)
{
	return p1.m_x * p2.m_y - p1.m_y * p2.m_x;
}

auto dot_3d(const point_3d &p1, const point_3d &p2)
{
	return p1.m_x * p2.m_x + p1.m_y * p2.m_y + p1.m_z * p2.m_z;
}

auto length_2d(const point_2d &p)
{
	return double(sqrt(dot_2d(p, p)));
}

auto length_3d(const point_3d &p)
{
	return double(sqrt(dot_3d(p, p)));
}

auto norm_2d(const point_2d &p)
{
	auto l = length_2d(p);
	if (l == 0.0) return point_2d(0.0, 0.0);
	return scale_2d(p, 1.0 / l);
}

auto norm_3d(const point_3d &p)
{
	auto l = length_3d(p);
	if (l == 0.0) return point_3d(0.0, 0.0, 0.0);
	return scale_3d(p, 1.0 / l);
}

auto distance_2d(const point_2d &p1, const point_2d &p2)
{
	return length_2d(sub_2d(p2, p1));
}

auto distance_squared_2d(const point_2d &p1, const point_2d &p2)
{
	auto p = sub_2d(p2, p1);
	return dot_2d(p, p);
}

auto distance_to_line_2d(const point_2d &p, const point_2d &p1, const point_2d &p2)
{
	auto lv = sub_2d(p2, p1);
	auto pv = sub_2d(p, p1);
	auto c1 = dot_2d(pv, lv);
	if (c1 <= 0.0) return distance_2d(p, p1);
	auto c2 = dot_2d(lv, lv);
	if (c2 <= c1) return distance_2d(p, p2);
	return distance_2d(p, add_2d(p1, scale_2d(lv, c1/c2)));
}

auto distance_squared_to_line_2d(const point_2d &p, const point_2d &p1, const point_2d &p2)
{
	auto lv = sub_2d(p2, p1);
	auto pv = sub_2d(p, p1);
	auto c1 = dot_2d(pv, lv);
	if (c1 <= 0.0) return distance_squared_2d(p, p1);
	auto c2 = dot_2d(lv, lv);
	if (c2 <= c1) return distance_squared_2d(p, p2);
	return distance_squared_2d(p, add_2d(p1, scale_2d(lv, c1/c2)));
}

auto collide_lines_2d(const point_2d &l1_p1, const point_2d &l1_p2, const point_2d &l2_p1, const point_2d &l2_p2)
{
	auto av = sub_2d(l1_p2, l1_p1);
	auto bv = sub_2d(l2_p2, l2_p1);
	auto cv = sub_2d(l2_p2, l1_p1);
	auto axb = det_2d(av, bv);
	auto axc = det_2d(av, cv);
	auto cxb = det_2d(cv, bv);
	if (axb == 0.0) return false;
	if (axb > 0.0)
	{
		if ((axc < 0.0) || (axc > axb)) return false;
		if ((cxb < 0.0) || (cxb > axb)) return false;
	}
	else
	{
		if ((axc > 0.0) || (axc < axb)) return false;
		if ((cxb > 0.0) || (cxb < axb)) return false;
	}
	return true;
}

bool collide_thick_lines_2d(const point_2d &tl1_p1, const point_2d &tl1_p2,
	 						const point_2d &tl2_p1, const point_2d &tl2_p2, double r)
{
	if (collide_lines_2d(tl1_p1, tl1_p2, tl2_p1, tl2_p2)) return true;
	r *= r;
	if (distance_squared_to_line_2d(tl2_p1, tl1_p1, tl1_p2) <= r) return true;
	if (distance_squared_to_line_2d(tl2_p2, tl1_p1, tl1_p2) <= r) return true;
	if (distance_squared_to_line_2d(tl1_p1, tl2_p1, tl2_p2) <= r) return true;
	if (distance_squared_to_line_2d(tl1_p2, tl2_p1, tl2_p2) <= r) return true;
	return false;
}

////////////////////
//generic path stuff
////////////////////

auto circle_as_lines(const point_2d &p, double radius, int resolution)
{
	auto out_points = points_2d{}; out_points.reserve(resolution+1);
	auto rvx = 0.0;
	auto rvy = radius;
	for (auto i = 0; i <= resolution; ++i)
	{
		auto angle = (i * M_PI * 2.0) / resolution;
		auto s = double(sin(angle));
		auto c = double(cos(angle));
		auto rv = point_2d(rvx*c - rvy*s, rvx*s + rvy*c);
		out_points.push_back(sub_2d(p, rv));
	}
	out_points.push_back(out_points[0]);
	return out_points;
}

auto torus_as_tristrip(const point_2d &p, double radius1, double radius2, int resolution)
{
	auto out_points = points_2d{}; out_points.reserve(resolution*2+2);
	auto rvx1 = 0.0;
	auto rvy1 = radius1;
	auto rvx2 = 0.0;
	auto rvy2 = radius2;
	for (auto i = 0; i <= resolution; ++i)
	{
		auto angle = (i * M_PI * 2.0) / resolution;
		auto s = double(sin(angle));
		auto c = double(cos(angle));
		auto rv1 = point_2d(rvx1*c - rvy1*s, rvx1*s + rvy1*c);
		auto rv2 = point_2d(rvx2*c - rvy2*s, rvx2*s + rvy2*c);
		out_points.push_back(sub_2d(p, rv1));
		out_points.push_back(sub_2d(p, rv2));
	}
	out_points.push_back(out_points[0]);
	out_points.push_back(out_points[1]);
	return out_points;
}

auto circle_as_trifan(const point_2d &p, double radius, int resolution)
{
	auto out_points = points_2d{}; out_points.reserve(resolution*2+2);
	auto rvx1 = 0.0;
	auto rvy1 = radius;
	out_points.push_back(p);
	for (auto i = 0; i <= resolution; ++i)
	{
		auto angle = (i * M_PI * 2.0) / resolution;
		auto s = double(sin(angle));
		auto c = double(cos(angle));
		auto rv1 = point_2d(rvx1*c - rvy1*s, rvx1*s + rvy1*c);
		out_points.push_back(sub_2d(p, rv1));
	}
	out_points.push_back(out_points[0]);
	return out_points;
}

auto thicken_path_as_lines(const points_2d &path, double radius, int capstyle, int joinstyle, int resolution)
{
	if (radius == 0.0) radius = 0.00000001;
	auto index = 0;
	auto step = 1;
	auto out_points = points_2d{};
	for (;;)
	{
		auto p1 = path[index];
		index += step;
		auto p2 = path[index];
		index += step;
		auto l2_v = sub_2d(p2, p1);
		auto l2_pv = perp_2d(l2_v);
		auto l2_npv = norm_2d(l2_pv);
		auto rv = scale_2d(l2_npv, radius);
		switch (capstyle)
		{
			case 0:
			{
				//butt cap
				out_points.push_back(sub_2d(p1, rv));
				out_points.push_back(add_2d(p1, rv));
				break;
			}
			case 1:
			{
				//square cap
				auto p0 = add_2d(p1, perp_2d(rv));
				out_points.push_back(sub_2d(p0, rv));
				out_points.push_back(add_2d(p0, rv));
				break;
			}
			case 2:
			{
				//triangle cap
				out_points.push_back(sub_2d(p1, rv));
				out_points.push_back(add_2d(p1, perp_2d(rv)));
				out_points.push_back(add_2d(p1, rv));
				break;
			}
			default:
			{
				//round cap
				auto rvx = rv.m_x;
				auto rvy = rv.m_y;
				for (auto i = 0; i <= resolution; ++i)
				{
					auto angle = (i * -M_PI) / resolution;
					auto s = double(sin(angle));
					auto c = double(cos(angle));
					auto rv = point_2d(rvx*c - rvy*s, rvx*s + rvy*c);
					out_points.push_back(sub_2d(p1, rv));
				}
			}
		}
		while ((index != -1) && (index != static_cast<int>(path.size())))
		{
			auto p1 = p2;
			auto l1_v = l2_v;
			auto l1_npv = l2_npv;
			p2 = path[index];
			index += step;
			l2_v = sub_2d(p2, p1);
			l2_pv = perp_2d(l2_v);
			l2_npv = norm_2d(l2_pv);
			auto nbv = norm_2d(scale_2d(add_2d(l1_npv, l2_npv), 0.5f));
			auto c = dot_2d(nbv, norm_2d(l1_v));
			if (c <= 0.0) goto mitre_join;
			switch (joinstyle)
			{
				case 0:
				{
				mitre_join:
					//mitre join
					auto s = double(sin(acos(c)));
					auto bv = scale_2d(nbv, radius/s);
					out_points.push_back(add_2d(p1, bv));
					break;
				}
				case 1:
				{
					//bevel join
					out_points.push_back(add_2d(p1, scale_2d(l1_npv, radius)));
					out_points.push_back(add_2d(p1, scale_2d(l2_npv, radius)));
					break;
				}
				default:
				{
					//round join
					auto rv = scale_2d(l1_npv, radius);
					auto rvx = rv.m_x;
					auto rvy = rv.m_y;
					auto theta = -double(acos(dot_2d(l1_npv, l2_npv)));
					auto segs = int((theta/-M_PI)*resolution) + 1;
					for (auto i = 0; i <= segs; ++i)
					{
						auto angle = (i * theta) / segs;
						auto s = double(sin(angle));
						auto c = double(cos(angle));
						auto rv = point_2d(rvx*c - rvy*s, rvx*s + rvy*c);
						out_points.push_back(add_2d(p1, rv));
					}
				}
			}
		}
		if (step < 0) break;
		step = -step;
		index += step;
	}
	out_points.push_back(out_points[0]);
	return out_points;
}

auto thicken_path_as_tristrip(const points_2d &path, double radius, int capstyle, int joinstyle, int resolution)
{
	if (radius == 0.0) radius = 0.00000001f;
	auto index = 0;
	auto step = 1;
	auto out_points = points_2d{};
	for (;;)
	{
		auto p1 = path[index];
		index += step;
		auto p2 = path[index];
		index += step;
		auto l2_v = sub_2d(p2, p1);
		auto l2_pv = perp_2d(l2_v);
		auto l2_npv = norm_2d(l2_pv);
		auto rv = scale_2d(l2_npv, radius);
		switch (capstyle)
		{
			case 0:
			{
				//butt cap
				out_points.push_back(p1);
				out_points.push_back(sub_2d(p1, rv));
				out_points.push_back(p1);
				out_points.push_back(add_2d(p1, rv));
				break;
			}
			case 1:
			{
				//square cap
				auto p0 = add_2d(p1, perp_2d(rv));
				out_points.push_back(p0);
				out_points.push_back(sub_2d(p0, rv));
				out_points.push_back(p0);
				out_points.push_back(add_2d(p0, rv));
				break;
			}
			case 2:
			{
				//triangle cap
				out_points.push_back(p1);
				out_points.push_back(sub_2d(p1, rv));
				out_points.push_back(p1);
				out_points.push_back(add_2d(p1, perp_2d(rv)));
				out_points.push_back(p1);
				out_points.push_back(add_2d(p1, rv));
				break;
			}
			default:
			{
				//round cap
				auto rvx = rv.m_x;
				auto rvy = rv.m_y;
				for (auto i = 0; i <= resolution; ++i)
				{
					auto angle = (i * -M_PI) / resolution;
					auto s = double(sin(angle));
					auto c = double(cos(angle));
					auto rv = point_2d(rvx*c - rvy*s, rvx*s + rvy*c);
					out_points.push_back(p1);
					out_points.push_back(sub_2d(p1, rv));
				}
			}
		}
		while ((index != -1) && (index != static_cast<int>(path.size())))
		{
			auto p1 = p2;
			auto l1_v = l2_v;
			auto l1_npv = l2_npv;
			p2 = path[index];
			index += step;
			l2_v = sub_2d(p2, p1);
			l2_pv = perp_2d(l2_v);
			l2_npv = norm_2d(l2_pv);
			auto nbv = norm_2d(scale_2d(add_2d(l1_npv, l2_npv), 0.5f));
			auto c = dot_2d(nbv, norm_2d(l1_v));
			if (c <= 0.0) goto mitre_join;
			switch (joinstyle)
			{
				case 0:
				{
				mitre_join:
					//mitre join
					auto s = double(sin(acos(c)));
					auto bv = scale_2d(nbv, radius/s);
					out_points.push_back(p1);
					out_points.push_back(add_2d(p1, bv));
					break;
				}
				case 1:
				{
					//bevel join
					out_points.push_back(p1);
					out_points.push_back(add_2d(p1, scale_2d(l1_npv, radius)));
					out_points.push_back(p1);
					out_points.push_back(add_2d(p1, scale_2d(l2_npv, radius)));
					break;
				}
				default:
				{
					//round join
					auto rv = scale_2d(l1_npv, radius);
					auto rvx = rv.m_x;
					auto rvy = rv.m_y;
					auto theta = -double(acos(dot_2d(l1_npv, l2_npv)));
					auto segs = int((theta/-M_PI)*resolution) + 1;
					for (auto i = 0; i <= segs; ++i)
					{
						auto angle = (i * theta) / segs;
						auto s = double(sin(angle));
						auto c = double(cos(angle));
						auto rv = point_2d(rvx*c - rvy*s, rvx*s + rvy*c);
						out_points.push_back(p1);
						out_points.push_back(add_2d(p1, rv));
					}
				}
			}
		}
		if (step < 0) break;
		step = -step;
		index += step;
	}
	out_points.push_back(out_points[0]);
	out_points.push_back(out_points[1]);
	return out_points;
}

auto recursive_bezier(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4,
	 					points_2d &points, double distance_tolerance)
{
	//calculate all the mid-points of the line segments
	auto x12 = (x1 + x2) * 0.5f;
	auto y12 = (y1 + y2) * 0.5f;
	auto x23 = (x2 + x3) * 0.5f;
	auto y23 = (y2 + y3) * 0.5f;
	auto x34 = (x3 + x4) * 0.5f;
	auto y34 = (y3 + y4) * 0.5f;
	auto x123 = (x12 + x23) * 0.5f;
	auto y123 = (y12 + y23) * 0.5f;
	auto x234 = (x23 + x34) * 0.5f;
	auto y234 = (y23 + y34) * 0.5f;
	auto x1234 = (x123 + x234) * 0.5f;
	auto y1234 = (y123 + y234) * 0.5f;

	//try to approximate the full cubic curve by a single straight line
	auto dx = x4 - x1;
	auto dy = y4 - y1;

	auto d2 = double(fabs((x2-x4)*dy - (y2-y4)*dx));
	auto d3 = double(fabs((x3-x4)*dy - (y3-y4)*dx));

	if ((d2+d3)*(d2+d3) < (distance_tolerance*(dx*dx+dy*dy)))
	{
		points.push_back(point_2d(x1234, y1234));
		return;
	}

	//continue subdivision
	recursive_bezier(x1, y1, x12, y12, x123, y123, x1234, y1234, points, distance_tolerance);
	recursive_bezier(x1234, y1234, x234, y234, x34, y34, x4, y4, points, distance_tolerance);
}

//create bezier path
auto bezier_path_as_lines(const point_2d &p1, const point_2d &p2,
	 						const point_2d &p3, const point_2d &p4, double distance_tolerance)
{
	auto points = points_2d{};
	points.push_back(point_2d(p1.m_x, p1.m_y));
	recursive_bezier(p1.m_x, p1.m_y, p2.m_x, p2.m_y, p3.m_x, p3.m_y, p4.m_x, p4.m_y, points, distance_tolerance);
	points.push_back(point_2d(p4.m_x, p4.m_y));
	return points;
}
