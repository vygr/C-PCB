#include "layer.h"
#include <algorithm>
#include <iostream>

extern bool collide_thick_lines_2d(const point_2d &tl1_p1, const point_2d &tl1_p2,
	 						const point_2d &tl2_p1, const point_2d &tl2_p2, float r);

layer::layer(const dims &dims, float s)
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
			auto b = &m_buckets[y*m_width + x];
			auto found = std::find_if(b->begin(), b->end(), [&] (auto &e)
			{
				return e->m_line == l;
			});
			if (found == b->end()) b->push_back(r);
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
			auto b = &m_buckets[y*m_width + x];
			b->erase(std::remove_if(b->begin(), b->end(), [&] (auto &e)
			{
				return e->m_line == l;
			}), b->end());
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
				auto r = l.m_radius + record->m_line.m_radius;
				if (l.m_gap >= record->m_line.m_gap)
				{
					r += l.m_gap;
				}
				else
				{
					r += record->m_line.m_gap;
				}
				if (collide_thick_lines_2d(l.m_p1, l.m_p2, record->m_line.m_p1, record->m_line.m_p2, r)) return true;
			}
		}
	}
	return false;
}

layers::layers(const dims &dims, float s)
	: m_depth(dims.m_depth)
{
	m_layers.reserve(m_depth);
	for (auto z = 0; z < m_depth; ++z)
	{
		m_layers.push_back(std::make_shared<layer>(layer::dims{dims.m_width, dims.m_height}, s));
	}
}

layers::~layers()
{}

void layers::add_line(const point_3d &p1, const point_3d &p2, float r, float g)
{
	auto z1 = int(p1.m_z);
	auto z2 = int(p2.m_z);
	if (z1 > z2) std::swap(z1, z2);
	auto l = layer::line{point_2d(p1.m_x, p1.m_y), point_2d(p2.m_x, p2.m_y), r, g};
	for (auto z = z1; z <= z2; ++z) m_layers[z]->add_line(l);
}

void layers::sub_line(const point_3d &p1, const point_3d &p2, float r, float g)
{
	auto z1 = int(p1.m_z);
	auto z2 = int(p2.m_z);
	if (z1 > z2) std::swap(z1, z2);
	auto l = layer::line{point_2d(p1.m_x, p1.m_y), point_2d(p2.m_x, p2.m_y), r, g};
	for (auto z = z1; z <= z2; ++z) m_layers[z]->sub_line(l);
}

bool layers::hit_line(const point_3d &p1, const point_3d &p2, float r, float g)
{
	auto z1 = int(p1.m_z);
	auto z2 = int(p2.m_z);
	if (z1 > z2) std::swap(z1, z2);
	auto l = layer::line{point_2d(p1.m_x, p1.m_y), point_2d(p2.m_x, p2.m_y), r, g};
	for (auto z = z1; z <= z2; ++z) if (m_layers[z]->hit_line(l)) return true;
	return false;
}
