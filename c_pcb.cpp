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

#include "io.h"

extern double length_2d(const point_2d &p);

//generate range of routing vectors
auto gen_vectors(int vec_range, int x_range, int y_range)
{
	auto yield = nodes{};
	for (auto y = y_range; y >= -y_range; --y)
	{
		for (auto x = x_range; x >= -x_range; --x)
		{
			auto p = point_2d{double(x), double(y)};
			if (length_2d(p) > 0.1 && length_2d(p) <= double(vec_range))
			{
				yield.push_back(node{x, y, 0});
			}
		}
	}
	return yield;
}

void ss_reset(std::stringstream &ss, std::string s)
{
	ss.str(s);
	ss.clear();
}

int main(int argc, char *argv[])
{
	//process comand args
	auto use_file = false;
	std::ifstream arg_infile;
	auto arg_t = double(600.0);
	auto arg_v = 0;
	auto arg_s = 1;
	auto arg_z = 0;
	auto arg_r = 1;
	auto arg_q = 1;
	auto arg_fr = 2;
	auto arg_xr = 1;
	auto arg_yr = 1;

	std::stringstream ss;
	for (auto i = 1; i < argc; ++i)
	{
		if (argv[i][0] == '-')
		{
			//option
			std::string opt = argv[i];
			while (!opt.empty() && opt[0] == '-') opt.erase(0, 1);
			if (++i >= argc) goto help;
			ss_reset(ss, argv[i]);
			if (opt == "t") ss >> arg_t;
			else if (opt == "v") ss >> arg_v;
			else if (opt == "z") ss >> arg_z;
			else if (opt == "s") ss >> arg_s;
			else if (opt == "r") ss >> arg_r;
			else if (opt == "q") ss >> arg_q;
			else if (opt == "fr") ss >> arg_fr;
			else if (opt == "xr") ss >> arg_xr;
			else if (opt == "yr") ss >> arg_yr;
			else
			{
			help:
				std::cout << "c_pcb [switches] [filename]\neg. c_pcb -t 600 -s 1 netlist.pcb\n";
				std::cout << "reads from stdin if no filename.\n";
				std::cout << "-t:  timeout in seconds, default 600\n";
				std::cout << "-v:  verbosity level 0..1, default 0\n";
				std::cout << "-z:  vias cost, 0..100, default 0\n";
				std::cout << "-s:  number of samples, default 1\n";
				std::cout << "-r:  grid resolution 1..4, default 1\n";
				std::cout << "-q:  area quantization, default 1\n";
				std::cout << "-fr: flood range 1..5, default 2\n";
				std::cout << "-xr: even layer x range 0..5, default 1\n";
				std::cout << "-yr: odd layer y range 0..5, default 1\n";
				exit(0);
			}
		}
		else
		{
			//filename
			arg_infile.open(argv[i], std::ifstream::in);
			use_file = true;
		}
	}

	//reading from stdin or file
	std::istream &in = use_file ? arg_infile : std::cin;

	//create flooding and backtracking vectors
	auto flood_range = arg_fr;
	auto flood_range_x_even_layer = arg_xr;
	auto flood_range_y_odd_layer = arg_yr;
	auto path_range = flood_range + 0;
	auto path_range_x_even_layer = flood_range_x_even_layer + 0;
	auto path_range_y_odd_layer = flood_range_y_odd_layer + 0;

	auto routing_flood_vectorss = nodess{
			gen_vectors(flood_range, flood_range_x_even_layer, flood_range),
			gen_vectors(flood_range, flood_range, flood_range_y_odd_layer)};

	auto routing_path_vectorss = nodess{
			gen_vectors(path_range, path_range_x_even_layer, path_range),
			gen_vectors(path_range, path_range, path_range_y_odd_layer)};

	//create pcb object and populate with tracks from input
	auto current_pcb = pcb(read_dimentions(in), routing_flood_vectorss, routing_path_vectorss,
					arg_r, arg_v, arg_q, arg_z);
	for (;;)
	{
		auto result = read_track(in);
		auto track = result.first;
		auto eof = result.second;
		if (eof) break;
		current_pcb.add_track(track);
	}

	//run number of samples of solution and pick best one
	current_pcb.print_pcb();
	auto best_cost = 1000000000;
	auto best_pcb = current_pcb;
	for (auto i = 0; i < arg_s; ++i)
	{
		if (!current_pcb.route(arg_t))
		{
			current_pcb.increase_quantization();
			continue;
		}
		auto cost = current_pcb.cost();
		if (cost <= best_cost)
		{
			best_cost = cost;
			best_pcb = current_pcb;
			best_pcb.print_stats();
		}
	}
	if (best_cost != 1000000000)
	{
		best_pcb.print_netlist();
		best_pcb.print_stats();
	}
	else
	{
		std::cout << "()" << std::endl;
	}
	return 0;
}
