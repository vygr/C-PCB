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

#include "router.h"
#include <chrono>
#include <random>
#include <algorithm>
#include <iostream>

const double spacial_hash_res = 0.75;

extern point_3d norm_3d(const point_3d &p);
extern point_3d sub_3d(const point_3d &p1, const point_3d &p2);

std::default_random_engine rand_gen(0);

//pcb methods

pcb::pcb(const dims &dims, const nodess &rfvs, const nodess &rpvs,
		dfunc_t dfunc, int res, int verb, int quant, int viascost)
	: m_width(dims.m_width)
	, m_height(dims.m_height)
	, m_depth(dims.m_depth)
	, m_routing_flood_vectors(rfvs)
	, m_routing_path_vectors(rpvs)
	, m_dfunc(dfunc)
	, m_resolution(res)
	, m_verbosity(verb)
	, m_quantization(quant * res)
	, m_viascost(viascost * res)
	, m_layers(layers(layers::dims{static_cast<int>(dims.m_width * spacial_hash_res),
		 							static_cast<int>(dims.m_height * spacial_hash_res),
									dims.m_depth}, spacial_hash_res/res))
{
	m_width *= res;
	m_height *= res;
	m_stride = m_width * m_height;
	m_nodes.resize(m_stride*m_depth);
}

pcb::~pcb()
{}

//add net
void pcb::add_track(track &t)
{
	m_netlist.push_back(net(t.m_terms, t.m_radius, t.m_via, t.m_gap, this));
}

//attempt to route board within time
bool pcb::route(float timeout)
{
	remove_netlist();
	unmark_distances();
	reset_areas();
	shuffle_netlist();
	std::sort(begin(m_netlist), end(m_netlist), [&] (auto &n1, auto &n2)
	{
		if (n1.m_area == n2.m_area) return n1.m_radius > n2.m_radius;
		return n1.m_area < n2.m_area;
	});
	auto hoisted_nets = std::set<net*>{};
	auto index = 0;
	auto start_time = std::chrono::high_resolution_clock::now();
	while (index < (int)m_netlist.size())
	{
		if (m_netlist[index].route()) index++;
		else
		{
			if (index == 0)
			{
				reset_areas();
				shuffle_netlist();
				std::sort(begin(m_netlist), end(m_netlist), [&] (auto &n1, auto &n2)
				{
					if (n1.m_area == n2.m_area) return n1.m_radius > n2.m_radius;
					return n1.m_area < n2.m_area;
				});
				hoisted_nets.clear();
			}
			else
			{
				auto pos = hoist_net(index);
				if ((pos == index) || (hoisted_nets.find(&m_netlist[pos]) != end(hoisted_nets)))
				{
					if (pos != 0)
					{
						m_netlist[pos].m_area = m_netlist[pos-1].m_area;
						pos = hoist_net(pos);
					}
					hoisted_nets.erase(&m_netlist[pos]);
				}
				else hoisted_nets.insert(&m_netlist[pos]);
				while (index > pos)
				{
					m_netlist[index].remove();
					m_netlist[index].shuffle_topology();
					index--;
				}
			}
		}
		auto end_time = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float> elapsed = end_time - start_time;
		if (elapsed.count() >= timeout) return false;
		if (m_verbosity >= 1) print_netlist();
	}
	return true;
}

//cost of board in complexity terms
int pcb::cost()
{
	auto sum = 0;
	for (auto &net : m_netlist) for (auto &path : net.m_paths) sum += path.size();
	return sum;
}

//increase area quantization
void pcb::increase_quantization()
{
	m_quantization++;
}

//output dimensions of board for viewer app
void pcb::print_pcb()
{
	auto scale = 1.0 / m_resolution;
	std::cout << '[' << m_width*scale << ',' << m_height*scale << ',' << m_depth << ']' << std::endl;
}

//output netlist and paths of board for viewer app
void pcb::print_netlist()
{
	for (auto &net : m_netlist) net.print_net();
	std::cout << "[]" << std::endl;
}

//output stats to screen
void pcb::print_stats()
{
	auto vias_set = std::set<node>{};
	auto num_pads = 0;
	auto num_nets = m_netlist.size();
	for (auto &net : m_netlist)
	{
		for (auto &path : net.m_paths)
		{
			auto p1 = path[0];
			for (auto itr = begin(path) + 1; itr != end(path); ++itr)
			{
				auto p0 = p1;
				p1 = *itr;
				if (p0.m_z != p1.m_z) vias_set.insert(node{p0.m_x, p0.m_y, 0});
			}
		}
	}
	for (auto &net : m_netlist)
	{
		num_pads += net.m_terminals.size();
		for (auto &term : net.m_terminals)
		{
			auto x = int(term.m_term.m_x+0.5);
			auto y = int(term.m_term.m_y+0.5);
			vias_set.erase(node{x, y, 0});
		}
	}
	std::cerr << "Number of Pads: " << num_pads << '\n';
	std::cerr << "Number of Nets: " << num_nets << '\n';
	std::cerr << "Number of Vias: " << vias_set.size() << std::endl;
}

//convert grid node to space node
point_3d pcb::grid_to_space_point(const node &n)
{
	auto itr = m_deform.find(n);
	if (itr != end(m_deform)) return itr->second;
	return point_3d{float(n.m_x), float(n.m_y), float(n.m_z)};
}

//set grid node to value
inline void pcb::set_node(const node &n, unsigned int value)
{
	m_nodes[(m_stride*n.m_z)+(n.m_y*m_width)+n.m_x] = value;
}

//get grid node value
inline auto pcb::get_node(const node &n)
{
	return m_nodes[(m_stride*n.m_z)+(n.m_y*m_width)+n.m_x];
}

//generate all grid points surrounding node, that are not value 0
sort_nodes &pcb::all_marked(const nodess &vec, const node &n)
{
	static auto yield = sort_nodes{}; yield.clear();
	auto x = n.m_x;
	auto y = n.m_y;
	auto z = n.m_z;
	for (auto &v : vec[z%2])
	{
		auto nx = x + v.m_x;
		auto ny = y + v.m_y;
		auto nz = z + v.m_z;
		if ((0 <= nx) && (nx < m_width)
		 	&& (0 <= ny) && (ny < m_height)
			&& (0 <= nz) && (nz < m_depth))
		{
			auto n = node{nx, ny, nz};
			auto mark = get_node(n);
			if (mark != 0) yield.push_back(sort_node{float(mark), n});
		}
	}
	return yield;
}

//generate all grid points surrounding node, that are value 0
nodes &pcb::all_not_marked(const nodess &vec, const node &n)
{
	static auto yield = nodes{}; yield.clear();
	auto x = n.m_x;
	auto y = n.m_y;
	auto z = n.m_z;
	for (auto &v : vec[z%2])
	{
		auto nx = x + v.m_x;
		auto ny = y + v.m_y;
		auto nz = z + v.m_z;
		if ((0 <= nx) && (nx < m_width)
		 	&& (0 <= ny) && (ny < m_height)
			&& (0 <= nz) && (nz < m_depth))
		{
			auto n = node{nx, ny, nz};
			if (get_node(n) == 0) yield.push_back(n);
		}
	}
	return yield;
}

//generate all grid points surrounding node sorted
nodes &pcb::all_nearer_sorted(const nodess &vec, const node &n, dfunc_t dfunc)
{
	static auto yield = nodes{}; yield.clear();
	auto gp = grid_to_space_point(n);
	auto distance = float(get_node(n));
	auto &marked_nodes = all_marked(vec, n);
	auto marked_nodes_end = std::remove_if(begin(marked_nodes), end(marked_nodes), [=, &gp] (auto &mn)
	{
		if ((distance - mn.m_mark) <= 0) return true;
		mn.m_mark = dfunc(grid_to_space_point(mn.m_node), gp);
		return false;
	});
	std::sort(begin(marked_nodes), marked_nodes_end, [&] (auto &s1, auto &s2)
	{
		return s1.m_mark < s2.m_mark;
	});
	std::for_each(begin(marked_nodes), marked_nodes_end, [&] (auto &sn)
	{
		yield.push_back(sn.m_node);
	});
	return yield;
}

//generate all grid points surrounding node that are not shorting with an existing track
nodes &pcb::all_not_shorting(const nodes &gather, const node &n, float radius, float gap)
{
	static auto yield = nodes{}; yield.clear();
	auto np = grid_to_space_point(n);
	for (auto &new_node : gather)
	{
		auto nnp = grid_to_space_point(new_node);
		if (!m_layers.hit_line(np, nnp, radius, gap)) yield.push_back(new_node);
	}
	return yield;
}

//flood fill distances from starts till ends covered
void pcb::mark_distances(const nodess &vec, float radius, float via, float gap,
						const node_set &starts, const nodes &ends)
{
	static auto via_vectors = nodess{
		nodes{node{0, 0, -1}, node{0, 0, 1}},
		nodes{node{0, 0, -1}, node{0, 0, 1}}};
	auto distance = 1U;
	auto frontier = starts;
	auto vias_nodes = std::map<int, node_set>{};
	while (!frontier.empty() || !vias_nodes.empty())
	{
		for (auto &node : frontier) set_node(node, distance);
		if (std::all_of(begin(ends), end(ends), [&] (auto &&n)
		{
			return get_node(n) != 0;
		})) break;
		auto new_nodes = node_set{};
		for (auto &node : frontier)
		{
			for (auto &new_node : all_not_shorting(
				all_not_marked(vec, node), node, radius, gap))
			{
				new_nodes.insert(new_node);
			}
		}
		auto new_vias_nodes = node_set{};
		for (auto &node : frontier)
		{
			for (auto &new_node : all_not_shorting(
				all_not_marked(via_vectors, node), node, via, gap))
			{
				new_vias_nodes.insert(new_node);
			}
		}
		if (!new_vias_nodes.empty()) vias_nodes[distance+m_viascost] = std::move(new_vias_nodes);
		auto delay_nodes = vias_nodes.find(distance);
		if (delay_nodes != end(vias_nodes))
		{
			for (auto &node : delay_nodes->second) if (get_node(node) == 0) new_nodes.insert(node);
			vias_nodes.erase(delay_nodes);
		}
		frontier = std::move(new_nodes);
		distance++;
	}
}

//set all grid values back to 0
void pcb::unmark_distances()
{
	std::fill(begin(m_nodes), end(m_nodes), 0);
}

//aabb of terminals
auto aabb_terminals(const terminals &terms, int quantization)
{
	auto minx = (int(terms[0].m_term.m_x) / quantization) * quantization;
	auto miny = (int(terms[0].m_term.m_y) / quantization) * quantization;
	auto maxx = ((int(terms[0].m_term.m_x) + (quantization - 1)) / quantization) * quantization;
	auto maxy = ((int(terms[0].m_term.m_y) + (quantization - 1)) / quantization) * quantization;
	for (auto i = 1; i < (int)terms.size(); ++i)
	{
		auto tminx = (int(terms[i].m_term.m_x) / quantization) * quantization;
		auto tminy = (int(terms[i].m_term.m_y) / quantization) * quantization;
		auto tmaxx = ((int(terms[i].m_term.m_x) + (quantization - 1)) / quantization) * quantization;
		auto tmaxy = ((int(terms[i].m_term.m_y) + (quantization - 1)) / quantization) * quantization;
		minx = std::min(tminx, minx);
		miny = std::min(tminy, miny);
		maxx = std::max(tmaxx, maxx);
		maxy = std::max(tmaxy, maxy);
	}
	auto rec = layer::aabb{minx, miny, maxx, maxy};
	return std::pair<int, layer::aabb>((maxx - minx) * (maxy - miny), rec);
}

//reset areas
void pcb::reset_areas()
{
	for (auto &net : m_netlist)
	{
		auto result = aabb_terminals(net.m_terminals, m_quantization);
		net.m_area = result.first;
		net.m_bbox = result.second;
	}
}

//shuffle order of netlist
void pcb::shuffle_netlist()
{
	std::shuffle(begin(m_netlist), end(m_netlist), rand_gen);
	for (auto &net : m_netlist) net.shuffle_topology();
}

//move net to top of area group
int pcb::hoist_net(int n)
{
	auto i = 0;
	if (n != 0)
	{
		for (i = n; i >= 0; --i) if (m_netlist[i].m_area < m_netlist[n].m_area) break;
		i++;
		if (n != i)
		{
			auto net = std::move(m_netlist[n]);
			m_netlist.erase(begin(m_netlist) + n);
			m_netlist.insert(begin(m_netlist) + i, net);
		}
	}
	return i;
}

//remove netlist from board
void pcb::remove_netlist()
{
	for (auto &net : m_netlist) net.remove();
}

//net methods

net::net(const terminals &terms, float radius, float via, float gap, pcb *pcb)
	: m_pcb(pcb)
	, m_radius(radius * pcb->m_resolution)
	, m_via(via * pcb->m_resolution)
	, m_gap(gap * pcb->m_resolution)
	, m_terminals(terms)
{
	process_terminals();
	auto result = aabb_terminals(m_terminals, pcb->m_quantization);
	m_area = result.first;
	m_bbox = result.second;
	remove();
	for (auto &term : m_terminals)
	{
		auto p = node{int(term.m_term.m_x + 0.5), int(term.m_term.m_y + 0.5), (int)term.m_term.m_z};
		auto sp = point_3d{term.m_term.m_x, term.m_term.m_y, term.m_term.m_z};
		pcb->m_deform[p] = sp;
	}
}

//terminal collision lines
void net::process_terminals()
{
	//scale terminals for resolution of grid
	for (auto &term : m_terminals)
	{
		term.m_radius *= m_pcb->m_resolution;
		term.m_gap *= m_pcb->m_resolution;
		term.m_term.m_x *= m_pcb->m_resolution;
		term.m_term.m_y *= m_pcb->m_resolution;
		for (auto &p : term.m_shape)
		{
			p.m_x *= m_pcb->m_resolution;
			p.m_y *= m_pcb->m_resolution;
		}
	}

	//build terminal collision lines and endpoint nodes
	std::sort(begin(m_terminals), end(m_terminals));
	for (auto i = begin(m_terminals); i != end(m_terminals);)
	{
		auto r = i->m_radius;
		auto g = i->m_gap;
		auto x = i->m_term.m_x;
		auto y = i->m_term.m_y;
		auto z1 = i->m_term.m_z;
		auto &&shape = i->m_shape;
		auto z2 = z1;
		while ((++i != end(m_terminals)) && std::tie(x, y, r, g, shape)
				== std::tie(i->m_term.m_x, i->m_term.m_y, i->m_radius, i->m_gap, i->m_shape))
		{
			z2 = i->m_term.m_z;
		}
		if (shape.empty())
			m_terminal_collision_lines.emplace_back(layers::line{point_3d{x, y, z1}, point_3d{x, y, z2}, r, g});
		else
		{
			for (auto z = z1; z <= z2; ++z)
			{
				auto p1 = point_3d{x + shape[0].m_x, y + shape[0].m_y, z};
				for (auto i = 1; i < (int)shape.size(); ++i)
				{
					auto p0 = p1;
					p1 = point_3d{x + shape[i].m_x, y + shape[i].m_y, z};
					m_terminal_collision_lines.emplace_back(layers::line{p0, p1, r, g});
				}
			}
		}

		m_terminal_end_nodes.emplace_back(nodes{});
		auto &&ends = m_terminal_end_nodes.back();
		for (auto z = z1; z <= z2; ++z) ends.emplace_back(node{int(x+0.5), int(y+0.5), int(z)});
	}
}

//randomize order of terminals
void net::shuffle_topology()
{
	std::shuffle(begin(m_terminal_end_nodes), end(m_terminal_end_nodes), rand_gen);
}

//add terminal entries to spacial cache
void net::add_terminal_collision_lines()
{
	for (auto &&l : m_terminal_collision_lines) m_pcb->m_layers.add_line(l);
}

//remove terminal entries from spacial cache
void net::sub_terminal_collision_lines()
{
	for (auto &&l : m_terminal_collision_lines) m_pcb->m_layers.sub_line(l);
}

//paths collision lines
std::vector<layers::line> net::paths_collision_lines() const
{
	auto max_lines = std::accumulate(cbegin(m_paths), cend(m_paths), 0u,
		[&] (auto acc, auto &&p) { return acc + p.size(); });
	auto paths_lines = std::vector<layers::line>{};
	paths_lines.reserve(max_lines);
	for (auto const &path : m_paths)
	{
		auto p1 = m_pcb->grid_to_space_point(path[0]);
		for (auto i = 1; i < static_cast<int>(path.size()); ++i)
		{
			auto p0 = p1;
			p1 = m_pcb->grid_to_space_point(path[i]);
			if (path[i-1].m_z != path[i].m_z) paths_lines.emplace_back(layers::line{p0, p1, m_via, m_gap});
			else paths_lines.emplace_back(layers::line{p0, p1, m_radius, m_gap});
		}
	}
	return paths_lines;
}

//add paths entries to spacial cache
void net::add_paths_collision_lines()
{
	m_paths_collision_lines = paths_collision_lines();
	for (auto &&l : m_paths_collision_lines) m_pcb->m_layers.add_line(l);
}

//remove paths entries from spacial cache
void net::sub_paths_collision_lines()
{
	for (auto &&l : m_paths_collision_lines) m_pcb->m_layers.sub_line(l);
}

//remove net entries from spacial grid
void net::remove()
{
	sub_paths_collision_lines();
	sub_terminal_collision_lines();
	m_paths.clear();
	add_terminal_collision_lines();
}

//remove redundant points from paths
nodess net::optimise_paths(const nodess &paths)
{
	auto opt_paths = nodess{};
	for (auto &path : paths)
	{
		auto opt_path = nodes{};
		auto d = point_3d{0.0, 0.0, 0.0};
		auto p1 = m_pcb->grid_to_space_point(path[0]);
		for (auto i = 1; i < (int)path.size(); ++i)
		{
			auto p0 = p1;
			p1 = m_pcb->grid_to_space_point(path[i]);
			auto d1 = norm_3d(sub_3d(p1, p0));
			if (d1 != d)
			{
				opt_path.push_back(path[i-1]);
				d = d1;
			}
		}
		opt_path.push_back(path[path.size()-1]);
		opt_paths.push_back(opt_path);
	}
	return opt_paths;
}

//backtrack path from ends to starts
std::pair<nodes, bool> net::backtrack_path(const node_set &visited, const node &end_node,
	 									float radius, float via, float gap)
{
	static auto via_vectors = nodess{
		nodes{node{0, 0, -1}, node{0, 0, 1}},
		nodes{node{0, 0, -1}, node{0, 0, 1}}};
	static auto nearer_nodes = nodes{};
	auto path = nodes{};
	auto path_node = end_node;
	for (;;)
	{
		path.push_back(path_node);
		nearer_nodes.clear();
		for (auto &node : m_pcb->all_not_shorting(
			m_pcb->all_nearer_sorted(m_pcb->m_routing_path_vectors, path_node, m_pcb->m_dfunc),
			path_node, radius, gap))
		{
			nearer_nodes.push_back(node);
		}
		for (auto &node : m_pcb->all_not_shorting(
			m_pcb->all_nearer_sorted(via_vectors, path_node, m_pcb->m_dfunc),
			path_node, via, gap))
		{
			nearer_nodes.push_back(node);
		}
		if (nearer_nodes.empty()) return std::pair<nodes, bool>(nodes{}, false);
		auto search = std::find_if(begin(nearer_nodes), end(nearer_nodes), [&] (auto &node)
		{
			return visited.find(node) != end(visited);
		});
		if (search != end(nearer_nodes))
		{
			//found existing track
			path.push_back(*search);
			return std::pair<nodes, bool>(std::move(path), true);
		}
		path_node = nearer_nodes[0];
	}
}

//attempt to route this net on the current boards state
bool net::route()
{
	//check for unused terminals track
	if (m_radius == 0.0) return true;
	m_paths = nodess{};
	sub_terminal_collision_lines();
	auto visited = node_set{};
	for (auto index = 1; index < static_cast<int>(m_terminal_end_nodes.size()); ++index)
	{
		auto &&ends = m_terminal_end_nodes[index];
		if (std::any_of(begin(ends), end(ends), [&] (auto &&node)
		{
			return visited.find(node) != end(visited);
		})) continue;
		for (auto &&start : m_terminal_end_nodes[index - 1]) visited.insert(start);
		m_pcb->mark_distances(m_pcb->m_routing_flood_vectors, m_radius, m_via, m_gap, visited, ends);
		std::sort(begin(ends), end(ends), [&] (auto &&n1, auto && n2)
		{
			 return m_pcb->get_node(n1) < m_pcb->get_node(n2);
		});
		auto result = backtrack_path(visited, ends[0], m_radius, m_via, m_gap);
		m_pcb->unmark_distances();
		if (!result.second)
		{
			remove();
			return false;
		}
		for (auto &node : result.first) visited.insert(node);
		m_paths.push_back(std::move(result.first));
	}
	m_paths = optimise_paths(m_paths);
	add_paths_collision_lines();
	add_terminal_collision_lines();
	return true;
}

//output net, terminals and paths, for viewer app
void net::print_net()
{
	auto scale = 1.0 / m_pcb->m_resolution;
	std::cout << "[" << m_radius*scale << "," << m_via*scale << "," << m_gap*scale << ",[";
	for (auto i = 0; i < static_cast<int>(m_terminals.size()); ++i)
	{
		auto t = m_terminals[i];
		std::cout << "(" << t.m_radius*scale << "," << t.m_gap*scale << ",("
		 	<< t.m_term.m_x*scale << "," << t.m_term.m_y*scale << "," << t.m_term.m_z << "),[";
		for (auto j = 0; j < static_cast<int>(t.m_shape.size()); ++j)
		{
			auto c = t.m_shape[j];
			std::cout << "(" << c.m_x*scale << "," << c.m_y*scale << ")";
			if (j != (static_cast<int>(t.m_shape.size()) - 1)) std::cout << ",";
		}
		std::cout << "])";
		if (i != (static_cast<int>(m_terminals.size())) - 1) std::cout << ",";
	}
	std::cout << "],[";
	for (auto i = 0; i < static_cast<int>(m_paths.size()); ++i)
	{
		auto path = m_paths[i];
		std::cout << "[";
		for (auto j = 0; j < static_cast<int>(path.size()); ++j)
		{
			auto p = path[j];
			auto sp = m_pcb->grid_to_space_point(p);
			std::cout << "(" << sp.m_x*scale << "," << sp.m_y*scale << "," << sp.m_z << ")";
			if (j != (static_cast<int>(path.size()) - 1)) std::cout << ",";
		}
		std::cout << "]";
		if (i != (static_cast<int>(m_paths.size())) - 1) std::cout << ",";
	}
	std::cout << "]]\n";
	return;
}
