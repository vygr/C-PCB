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
	while (index < m_netlist.size())
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
	auto num_vias = 0;
	auto num_terminals = 0;
	auto num_nets = m_netlist.size();
	for (auto &net : m_netlist)
	{
		num_terminals += net.m_terminals.size();
		for (auto &path : net.m_paths)
		{
			auto p1 = path[0];
			for (auto itr = begin(path) + 1; itr != end(path); ++itr)
			{
				auto p0 = p1;
				p1 = *itr;
				if (p0.m_z != p1.m_z) num_vias++;
			}
		}
	}
	std::cerr << "Number of terminals: " << num_terminals << '\n';
	std::cerr << "Number of Nets: " << num_nets << '\n';
	std::cerr << "Number of Vias: " << num_vias << std::endl;
}

//convert grid node to space node
point_3d pcb::grid_to_space_point(const node &n)
{
	auto itr = m_deform.find(n);
	if (itr != end(m_deform)) return itr->second;
	return point_3d{float(n.m_x), float(n.m_y), float(n.m_z)};
}

//set grid node to value
inline void pcb::set_node(const node &n, int value)
{
	m_nodes[(m_stride*n.m_z)+(n.m_y*m_width)+n.m_x] = value;
}

//get grid node value
inline int pcb::get_node(const node &n)
{
	return m_nodes[(m_stride*n.m_z)+(n.m_y*m_width)+n.m_x];
}

//generate all grid points surrounding node, that are not value 0
void pcb::all_marked(sort_nodes &yield, const nodess &vec, const node &n)
{
	auto x = n.m_x;
	auto y = n.m_y;
	auto z = n.m_z;
	yield.clear();
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
}

//generate all grid points surrounding node, that are value 0
void pcb::all_not_marked(nodes &yield, const nodess &vec, const node &n)
{
	auto x = n.m_x;
	auto y = n.m_y;
	auto z = n.m_z;
	yield.clear();
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
}

//generate all grid points surrounding node sorted
void pcb::all_nearer_sorted(nodes &yield, sort_nodes &marked_nodes, const nodess &vec, const node &n, dfunc_t dfunc)
{
	auto gp = grid_to_space_point(n);
	auto distance = float(get_node(n));
	all_marked(marked_nodes, vec, n);
	auto sns_end = std::remove_if(begin(marked_nodes), end(marked_nodes), [=, &gp] (auto &mn)
	{
		if ((distance - mn.m_mark) <= 0) return true;
		mn.m_mark = dfunc(grid_to_space_point(mn.m_node), gp);
		return false;
	});
	std::sort(begin(marked_nodes), sns_end, [&] (auto &s1, auto &s2)
	{
		return s1.m_mark < s2.m_mark;
	});
	yield.clear();
	std::for_each(begin(marked_nodes), sns_end, [&] (auto &sn)
	{
		yield.push_back(sn.m_node);
	});
}

//generate all grid points surrounding node that are not shorting with an existing track
void pcb::all_not_shorting(nodes &yield, const nodes &gather, const node &n, float radius, float gap)
{
	yield.clear();
	auto np = grid_to_space_point(n);
	for (auto &new_node : gather)
	{
		auto nnp = grid_to_space_point(new_node);
		if (!m_layers.hit_line(np, nnp, radius, gap)) yield.push_back(new_node);
	}
}

//flood fill distances from starts till ends covered
void pcb::mark_distances(const nodess &vec, float radius, float via, float gap,
						const node_set &starts, const nodes &ends)
{
	static auto via_vectors = nodess{
		nodes{node{0, 0, -1}, node{0, 0, 1}},
		nodes{node{0, 0, -1}, node{0, 0, 1}}};
	auto distance = 1;
	auto frontier = starts;
	auto vias_nodes = std::map<int, node_set>{};
	auto not_marked_nodes = nodes{};
	auto not_shorting_nodes = nodes{};
	while ((frontier.size() > 0) || (vias_nodes.size() > 0))
	{
		for (auto &node : frontier) set_node(node, distance);
		auto flag = true;
		for (auto &node : ends)
		{
			if (get_node(node) == 0)
			{
				flag = false;
				break;
			}
		}
		if (flag) break;
		auto new_nodes = node_set{};
		for (auto &node : frontier)
		{
			all_not_marked(not_marked_nodes, vec, node);
			all_not_shorting(not_shorting_nodes, not_marked_nodes, node, radius, gap);
			for (auto &new_node : not_shorting_nodes) new_nodes.insert(new_node);
		}
		auto new_vias_nodes = node_set{};
		for (auto &node : frontier)
		{
			all_not_marked(not_marked_nodes, via_vectors, node);
			all_not_shorting(not_shorting_nodes, not_marked_nodes, node, via, gap);
			for (auto &new_node : not_shorting_nodes) new_vias_nodes.insert(new_node);
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
	for (auto i = 1; i < terms.size(); ++i)
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

//scale terminals for resolution of grid
auto scale_terminals(terminals &terms, int res)
{
	for (auto &term : terms)
	{
		term.m_radius *= res;
		term.m_gap *= res;
		term.m_term.m_x *= res;
		term.m_term.m_y *= res;
		term.m_term.m_z *= res;
		for (auto &p : term.m_shape)
		{
			p.m_x *= res;
			p.m_y *= res;
		}
	}
}

//net methods

net::net(const terminals &terms, float radius, float via, float gap, pcb *pcb)
	: m_pcb(pcb)
	, m_radius(radius * pcb->m_resolution)
	, m_via(via * pcb->m_resolution)
	, m_gap(gap * pcb->m_resolution)
	, m_terminals(terms)
{
	scale_terminals(m_terminals, pcb->m_resolution);
	auto result = aabb_terminals(m_terminals, pcb->m_quantization);
	m_area = result.first;
	m_bbox = result.second;
	remove();
	for (auto &term : m_terminals)
	{
		for (auto z = 0; z < pcb->m_depth; ++z)
		{
			auto p = node{int(term.m_term.m_x + 0.5), int(term.m_term.m_y + 0.5), z};
			auto sp = point_3d{term.m_term.m_x, term.m_term.m_y, float(z)};
			pcb->m_deform[p] = sp;
		}
	}
}

//randomize order of terminals
void net::shuffle_topology()
{
    std::shuffle(begin(m_terminals), end(m_terminals), rand_gen);
}

//add terminal entries to spacial cache
void net::add_terminal_collision_lines()
{
	for (auto &node : m_terminals)
	{
		auto r = node.m_radius;
		auto g = node.m_gap;
		auto x = node.m_term.m_x;
		auto y = node.m_term.m_y;
		auto shape = node.m_shape;
		if (shape.empty())
			m_pcb->m_layers.add_line(point_3d{x, y, 0}, point_3d{x, y, float(m_pcb->m_depth - 1)}, r, g);
		else
		{
			for (auto z = 0; z < m_pcb->m_depth; ++z)
			{
				auto p1 = point_3d{x + shape[0].m_x, y + shape[0].m_y, float(z)};
				for (auto i = 1; i < shape.size(); ++i)
				{
					auto p0 = p1;
					p1 = point_3d{x + shape[i].m_x, y + shape[i].m_y, float(z)};
					m_pcb->m_layers.add_line(p0, p1, r, g);
				}
			}
		}
	}
}

//remove terminal entries from spacial cache
void net::sub_terminal_collision_lines()
{
	for (auto &node : m_terminals)
	{
		auto r = node.m_radius;
		auto g = node.m_gap;
		auto x = node.m_term.m_x;
		auto y = node.m_term.m_y;
		auto shape = node.m_shape;
		if (shape.empty())
			m_pcb->m_layers.sub_line(point_3d{x, y, 0}, point_3d{x, y, float(m_pcb->m_depth - 1)}, r, g);
		else
		{
			for (auto z = 0; z < m_pcb->m_depth; ++z)
			{
				auto p1 = point_3d{x + shape[0].m_x, y + shape[0].m_y, float(z)};
				for (auto i = 1; i < shape.size(); ++i)
				{
					auto p0 = p1;
					p1 = point_3d{x + shape[i].m_x, y + shape[i].m_y, float(z)};
					m_pcb->m_layers.sub_line(p0, p1, r, g);
				}
			}
		}
	}
}

//add paths entries to spacial cache
void net::add_paths_collision_lines()
{
	for (auto &path : m_paths)
	{
		auto p1 = m_pcb->grid_to_space_point(path[0]);
		for (auto i = 1; i < path.size(); ++i)
		{
			auto p0 = p1;
			p1 = m_pcb->grid_to_space_point(path[i]);
			if (path[i-1].m_z != path[i].m_z) m_pcb->m_layers.add_line(p0, p1, m_via, m_gap);
			else m_pcb->m_layers.add_line(p0, p1, m_radius, m_gap);
		}
	}
}

//remove paths entries from spacial cache
void net::sub_paths_collision_lines()
{
	for (auto &path : m_paths)
	{
		auto p1 = m_pcb->grid_to_space_point(path[0]);
		for (auto i = 1; i < path.size(); ++i)
		{
			auto p0 = p1;
			p1 = m_pcb->grid_to_space_point(path[i]);
			if (path[i-1].m_z != path[i].m_z) m_pcb->m_layers.sub_line(p0, p1, m_via, m_gap);
			else m_pcb->m_layers.sub_line(p0, p1, m_radius, m_gap);
		}
	}
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
		for (auto i = 1; i < path.size(); ++i)
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
	auto not_shorting_nodes = nodes{};
	auto nearer_nodes = nodes{};
	auto sorted_nodes = nodes{};
	auto marked_nodes = sort_nodes{};
	auto path = nodes{};
	auto path_node = end_node;
	for (;;)
	{
		path.push_back(path_node);
		nearer_nodes.clear();
		m_pcb->all_nearer_sorted(sorted_nodes, marked_nodes, m_pcb->m_routing_path_vectors, path_node, m_pcb->m_dfunc);
		m_pcb->all_not_shorting(not_shorting_nodes, sorted_nodes, path_node, radius, gap);
		for (auto &node : not_shorting_nodes) nearer_nodes.push_back(node);
		m_pcb->all_nearer_sorted(sorted_nodes, marked_nodes, via_vectors, path_node, m_pcb->m_dfunc);
		m_pcb->all_not_shorting(not_shorting_nodes, sorted_nodes, path_node, via, gap);
		for (auto &node : not_shorting_nodes) nearer_nodes.push_back(node);
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
	for (auto index = 1; index < m_terminals.size(); ++index)
	{
		auto ends = nodes{}; ends.reserve(m_pcb->m_depth);
		for (auto z = 0; z < m_pcb->m_depth; ++z)
		{
			auto x = int(m_terminals[index].m_term.m_x+0.5);
			auto y = int(m_terminals[index].m_term.m_y+0.5);
			ends.push_back(node{x, y, z});
		}
		auto search = std::find_if(begin(ends), end(ends), [&] (auto &node)
		{
			return visited.find(node) != end(visited);
		});
		if (search != end(ends)) continue;
		for (auto z = 0; z < m_pcb->m_depth; ++z)
		{
			auto x = int(m_terminals[index-1].m_term.m_x+0.5);
			auto y = int(m_terminals[index-1].m_term.m_y+0.5);
			visited.insert(node{x, y, z});
		}
		m_pcb->mark_distances(m_pcb->m_routing_flood_vectors, m_radius, m_via, m_gap, visited, ends);
		auto sorted_ends = sort_nodes{}; sorted_ends.reserve(ends.size());
		for (auto &node : ends) sorted_ends.push_back(sort_node{float(m_pcb->get_node(node)), node});
		std::sort(begin(sorted_ends), end(sorted_ends), [&] (auto &s1, auto &s2)
		{
			return s1.m_mark < s2.m_mark;
		});
		auto result = backtrack_path(visited, sorted_ends[0].m_node, m_radius, m_via, m_gap);
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
	for (auto i = 0; i < m_terminals.size(); ++i)
	{
		auto t = m_terminals[i];
		std::cout << "(" << t.m_radius*scale << "," << t.m_gap*scale << ",("
		 	<< t.m_term.m_x*scale << "," << t.m_term.m_y*scale << "," << t.m_term.m_z << "),[";
		for (auto j = 0; j < t.m_shape.size(); ++j)
		{
			auto c = t.m_shape[j];
			std::cout << "(" << c.m_x*scale << "," << c.m_y*scale << ")";
			if (j != (t.m_shape.size() - 1)) std::cout << ",";
		}
		std::cout << "])";
		if (i != (m_terminals.size()) - 1) std::cout << ",";
	}
	std::cout << "],[";
	for (auto i = 0; i < m_paths.size(); ++i)
	{
		auto path = m_paths[i];
		std::cout << "[";
		for (auto j = 0; j < path.size(); ++j)
		{
			auto p = path[j];
			auto sp = m_pcb->grid_to_space_point(p);
			std::cout << "(" << sp.m_x*scale << "," << sp.m_y*scale << "," << sp.m_z << ")";
			if (j != (path.size() - 1)) std::cout << ",";
		}
		std::cout << "]";
		if (i != (m_paths.size()) - 1) std::cout << ",";
	}
	std::cout << "]]";
	return;
}
