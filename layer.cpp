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

#include "layer.h"
#include <algorithm>

layer::layer(const dims &dims, double s)
	: m_width(dims.m_width)
	, m_height(dims.m_height)
	, m_scale(s)
	, m_test(0)
{
	m_buckets.resize(m_width * m_height);
}

layer::~layer()
{}

layer::aabb layer::get_aabb(const line &l)
{
	auto x1 = l.m_p1.m_x;
	auto y1 = l.m_p1.m_y;
	auto x2 = l.m_p2.m_x;
	auto y2 = l.m_p2.m_y;
	if (x1 > x2) std::swap(x1, x2);
	if (y1 > y2) std::swap(y1, y2);
	auto r = l.m_radius + l.m_gap;
	auto minx = int((x1 - r) * m_scale);
	auto miny = int((y1 - r) * m_scale);
	auto maxx = int((x2 + r) * m_scale);
	auto maxy = int((y2 + r) * m_scale);
	minx = std::max(0, minx);
	miny = std::max(0, miny);
	maxx = std::max(0, maxx);
	maxy = std::max(0, maxy);
	minx = std::min(m_width - 1, minx);
	maxx = std::min(m_width - 1, maxx);
	miny = std::min(m_height - 1, miny);
	maxy = std::min(m_height - 1, maxy);
	return aabb{minx, miny, maxx, maxy};
}

void layer::add_line(const line &l)
{
	auto bb = get_aabb(l);
	auto r = std::make_shared<record>(0, l);
	for (auto y = bb.m_miny; y <= bb.m_maxy; ++y)
	{
		for (auto x = bb.m_minx; x <= bb.m_maxx; ++x)
		{
			m_buckets[y*m_width + x].push_back(r);
		}
	}
}

void layer::sub_line(const line &l)
{
	auto bb = get_aabb(l);
	for (auto y = bb.m_miny; y <= bb.m_maxy; ++y)
	{
		for (auto x = bb.m_minx; x <= bb.m_maxx; ++x)
		{
			auto &&b = m_buckets[y*m_width + x];
			auto itr = std::find_if(begin(b), end(b), [&] (auto &e)
			{
				return e->m_line == l;
			});
			if (itr == end(b)) continue;
			std::iter_swap(itr, end(b) - 1);
			b.pop_back();
		}
	}
}

bool layer::hit_line(const line &l)
{
	m_test += 1;
	auto bb = get_aabb(l);
	for (auto y = bb.m_miny; y <= bb.m_maxy; ++y)
	{
		for (auto x = bb.m_minx; x <= bb.m_maxx; ++x)
		{
			for (auto &record : m_buckets[y*m_width+x])
			{
				if (record->m_id == m_test) continue;
				record->m_id = m_test;
				auto d = l.m_radius + record->m_line.m_radius + std::max(l.m_gap, record->m_line.m_gap);
				if (record->hit(l, d)) return true;
			}
		}
	}
	return false;
}

layers::layers(const dims &dims, double s)
	: m_depth(dims.m_depth)
{
	m_layers.reserve(m_depth);
	for (auto z = 0; z < m_depth; ++z)
		m_layers.push_back(std::make_shared<layer>(layer::dims{dims.m_width, dims.m_height}, s));
}

layers::~layers()
{}

void layers::add_line(const point_3d &p1, const point_3d &p2, double r, double g)
{
	auto z1 = int(p1.m_z);
	auto z2 = int(p2.m_z);
	if (z1 > z2) std::swap(z1, z2);
	auto l = layer::line{point_2d(p1.m_x, p1.m_y), point_2d(p2.m_x, p2.m_y), r, g};
	for (auto z = z1; z <= z2; ++z) m_layers[z]->add_line(l);
}

void layers::sub_line(const point_3d &p1, const point_3d &p2, double r, double g)
{
	auto z1 = int(p1.m_z);
	auto z2 = int(p2.m_z);
	if (z1 > z2) std::swap(z1, z2);
	auto l = layer::line{point_2d(p1.m_x, p1.m_y), point_2d(p2.m_x, p2.m_y), r, g};
	for (auto z = z1; z <= z2; ++z) m_layers[z]->sub_line(l);
}

bool layers::hit_line(const point_3d &p1, const point_3d &p2, double r, double g)
{
	auto z1 = int(p1.m_z);
	auto z2 = int(p2.m_z);
	if (z1 > z2) std::swap(z1, z2);
	auto l = layer::line{point_2d(p1.m_x, p1.m_y), point_2d(p2.m_x, p2.m_y), r, g};
	for (auto z = z1; z <= z2; ++z) if (m_layers[z]->hit_line(l)) return true;
	return false;
}

void layers::add_line(const line &l)
{
	auto z1 = int(l.m_p1.m_z);
	auto z2 = int(l.m_p2.m_z);
	if (z1 > z2) std::swap(z1, z2);
	auto l1 = layer::line{point_2d(l.m_p1.m_x, l.m_p1.m_y),
						point_2d(l.m_p2.m_x, l.m_p2.m_y),
						l.m_radius, l.m_gap};
	for (auto z = z1; z <= z2; ++z) m_layers[z]->add_line(l1);
}

void layers::sub_line(const line &l)
{
	auto z1 = int(l.m_p1.m_z);
	auto z2 = int(l.m_p2.m_z);
	if (z1 > z2) std::swap(z1, z2);
	auto l1 = layer::line{point_2d(l.m_p1.m_x, l.m_p1.m_y),
						point_2d(l.m_p2.m_x, l.m_p2.m_y),
						l.m_radius, l.m_gap};
	for (auto z = z1; z <= z2; ++z) m_layers[z]->sub_line(l1);
}

bool layers::hit_line(const line &l)
{
	auto z1 = int(l.m_p1.m_z);
	auto z2 = int(l.m_p2.m_z);
	if (z1 > z2) std::swap(z1, z2);
	auto l1 = layer::line{point_2d(l.m_p1.m_x, l.m_p1.m_y),
						point_2d(l.m_p2.m_x, l.m_p2.m_y),
						l.m_radius, l.m_gap};
	for (auto z = z1; z <= z2; ++z) if (m_layers[z]->hit_line(l1)) return true;
	return false;
}
