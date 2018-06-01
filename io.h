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

#ifndef IO_H
#define IO_H

#include "router.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include <sstream>

//read input till given byte appears
auto read_until(std::istream &in, char c)
{
	char input;
	while (in.get(input))
	{
		if (input == c) return false;
	}
	return true;
}

//read, (width height depth)
auto read_dimentions(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto dims = pcb::dims{};
	in >> dims.m_width;
	in >> dims.m_height;
	in >> dims.m_depth;
	if (read_until(in, ')')) exit(1);
	return dims;
}

//read, (x y)
auto read_point_2d(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto p = point_2d{};
	in >> p.m_x;
	in >> p.m_y;
	if (read_until(in, ')')) exit(1);
	return p;
}

//read, (x y z)
auto read_point_3d(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto p = point_3d{};
	in >> p.m_x;
	in >> p.m_y;
	in >> p.m_z;
	if (read_until(in, ')')) exit(1);
	return p;
}

//read, ((x  y) ...)
auto read_shape(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto cords = points_2d{};
	for (;;)
	{
		if (in.peek() == ')') break;
		cords.push_back(read_point_2d(in));
	}
	if (read_until(in, ')')) exit(1);
	return cords;
}

//read, ((x y z) ...)
auto read_path(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto cords = path{};
	for (;;)
	{
		if (in.peek() == ')') break;
		cords.push_back(read_point_3d(in));
	}
	if (read_until(in, ')')) exit(1);
	return cords;
}

//read, (((x y z) ...) ...)
auto read_paths(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto t = paths{};
	for (;;)
	{
		if (in.peek() == ')') break;
		t.push_back(read_path(in));
	}
	if (read_until(in, ')')) exit(1);
	return t;
}

//read, (radius gap (x y z) ((x y) ...))
auto read_pad(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto t = pad{};
	in >> t.m_radius;
	in >> t.m_gap;
	t.m_term = read_point_3d(in);
	t.m_shape = read_shape(in);
	if (read_until(in, ')')) exit(1);
	return t;
}

//read all pads for one track
auto read_pads(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto t = pads{};
	for (;;)
	{
		if (in.peek() == ')') break;
		t.push_back(read_pad(in));
	}
	if (read_until(in, ')')) exit(1);
	return t;
}

//read one track
auto read_track(std::istream &in)
{
	if (read_until(in, '(')) return std::pair<track, bool>(track{}, true);
	if (in.peek() == ')') return std::pair<track, bool>(track{}, true);
	auto t = track{};
	in >> t.m_radius;
	in >> t.m_via;
	in >> t.m_gap;
	t.m_pads = read_pads(in);
	t.m_paths = read_paths(in);
	if (read_until(in, ')')) exit(1);
	return std::pair<track, bool>(t, false);
}

#endif
