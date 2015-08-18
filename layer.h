#ifndef LAYER_H
#define LAYER_H

#include "mymath.h"
#include <memory>

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
			return std::tie(l.m_p1, l.m_p2, l.m_radius, l.m_gap) == std::tie(m_p1, m_p2, m_radius, m_gap);
		}
		point_2d m_p1;
		point_2d m_p2;
		float m_radius;
		float m_gap;
	};

	struct record
	{
		record(int id, const line &l) : m_id(id), m_line(l) {}
		int m_id;
		line m_line;
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

	layer(const dims &dims, float s);
	~layer();
	aabb get_aabb(const line &l);
	void add_line(const line &l);
	void sub_line(const line &l);
	bool hit_line(const line &l);

private:
	int m_width;
	int m_height;
	float m_scale;
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

	layers(const dims &dims, float s);
	~layers();
	void add_line(const point_3d &p1, const point_3d &p2, float r, float g);
	void sub_line(const point_3d &p1, const point_3d &p2, float r, float g);
	bool hit_line(const point_3d &p1, const point_3d &p2, float r, float g);

private:
	int m_depth;
	std::vector<std::shared_ptr<layer>> m_layers;
};

#endif
