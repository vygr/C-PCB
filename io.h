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
	auto t = pcb::dims{};
	in >> t.m_width;
	in >> t.m_height;
	in >> t.m_depth;
	if (read_until(in, ')')) exit(1);
	return t;
}

//read, (x y)
auto read_point_2d(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto t = point_2d{};
	in >> t.m_x;
	in >> t.m_y;
	if (read_until(in, ')')) exit(1);
	return t;
}

//read, (x y z)
auto read_point_3d(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto t = point_3d{};
	in >> t.m_x;
	in >> t.m_y;
	in >> t.m_z;
	if (read_until(in, ')')) exit(1);
	return t;
}

//read, ((x  y) ...)
auto read_shape(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto t = points_2d{};
	for (;(in.peek() != ')');) t.push_back(read_point_2d(in));
	if (read_until(in, ')')) exit(1);
	return t;
}

//read, ((x y z) ...)
auto read_path(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto t = path{};
	for (;(in.peek() != ')');) t.push_back(read_point_3d(in));
	if (read_until(in, ')')) exit(1);
	return t;
}

//read, (((x y z) ...) ...)
auto read_paths(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto t = paths{};
	for (;(in.peek() != ')');) t.push_back(read_path(in));
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
	t.m_pos = read_point_3d(in);
	t.m_shape = read_shape(in);
	if (read_until(in, ')')) exit(1);
	return t;
}

//read all pads for one track
auto read_pads(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto t = pads{};
	for (;(in.peek() != ')');) t.push_back(read_pad(in));
	if (read_until(in, ')')) exit(1);
	return t;
}

//read one track
auto read_track(std::istream &in)
{
	if (read_until(in, '(')) return std::pair<track, bool>(track{}, true);
	if (in.peek() == ')') return std::pair<track, bool>(track{}, true);
	auto t = track{};
	in >> t.m_id;
	in >> t.m_track_radius;
	in >> t.m_via_radius;
	in >> t.m_gap;
	t.m_pads = read_pads(in);
	t.m_paths = read_paths(in);
	if (read_until(in, ')')) exit(1);
	return std::pair<track, bool>(t, false);
}

#endif
