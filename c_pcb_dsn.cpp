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
#include "math.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

struct tree
{
	std::string m_value;
	std::vector <tree> m_branches;
};

struct pin
{
	std::string m_name;
	std::string m_form;
	float m_x;
	float m_y;
	float m_angle;
};

struct component
{
	std::string m_name;
	std::map<std::string, pin> m_pin_map;
};

struct instance
{
	std::string m_name;
	std::string m_comp;
	std::string m_side;
	float m_x;
	float m_y;
	float m_angle;
};

struct rule
{
	float m_radius;
	float m_gap;
};

struct circuit
{
	std::string m_via;
	rule m_rule;
};

struct padstack
{
	points_2d m_shape;
	rule m_rule;
};

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems)
{
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim))
	{
		elems.push_back(item);
	}
	return elems;
}

std::vector<std::string> split(const std::string &s, char delim)
{
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}

auto shape_to_cords(const points_2d &shape, float a1, float a2)
{
	auto cords = points_2d{};
	auto rads = fmod(a1+a2, 2*M_PI);
	auto s = sin(rads);
	auto c = cos(rads);
	for (auto &p : shape)
	{
		auto px = float(c*p.m_x - s*p.m_y);
		auto py = float(s*p.m_x + c*p.m_y);
		cords.push_back(point_2d{px, py});
	}
	return cords;
}

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

//read whitespace
auto read_whitespace(std::istream &in)
{
	for (;;)
	{
		auto b = in.peek();
		if (b != '\t' && b != '\n' && b != '\r' && b != ' ') break;
		char c;
		in.get(c);
	}
}

auto read_node_name(std::istream &in)
{
	std::string s;
	for (;;)
	{
		auto b = in.peek();
		if (b == '\t' || b == '\n' || b == '\r' || b == ' ' || b == ')') break;
		char c;
		in.get(c);
		s.push_back(c);
	}
	return s;
}

auto read_string(std::istream &in)
{
	std::string s;
	for (;;)
	{
		auto b = in.peek();
		if (b == '\t' || b == '\n' || b == '\r' || b == ' ' || b == ')') break;
		char c;
		in.get(c);
		s.push_back(c);
	}
	return tree{s, {}};
}

auto read_quoted_string(std::istream &in)
{
	std::string s;
	for (;;)
	{
		auto b = in.peek();
		if (b == '"') break;
		char c;
		in.get(c);
		s.push_back(c);
	}
	return tree{s, {}};
}

tree read_tree(std::istream &in)
{
	read_until(in, '(');
	read_whitespace(in);
	auto t = tree{read_node_name(in), {}};
	for (;;)
	{
		read_whitespace(in);
		auto b = in.peek();
		char c;
		if (b == EOF) break;
		if (b == ')')
		{
			in.get(c);
			break;
		}
		if (b == '(')
		{
			t.m_branches.push_back(read_tree(in));
			continue;
		}
		if (b == '"')
		{
			in.get(c);
			t.m_branches.push_back(read_quoted_string(in));
			in.get(c);
			continue;
		}
		t.m_branches.push_back(read_string(in));
	}
	return t;
}

const tree *search_tree(const tree &t, const char *s)
{
	if (t.m_value == s) return &t;
	for (auto &ct : t.m_branches)
	{
		auto st = search_tree(ct, s);
		if (st != nullptr) return st;
	}
	return nullptr;
}

void print_tree(const tree &t, int indent)
{
	if (!t.m_value.empty())
	{
		for (auto i = 0; i < indent; ++i) std::cout << "  ";
		std::cout << t.m_value << '\n';
	}
	for (auto &ct : t.m_branches) print_tree(ct, indent+1);
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
	auto arg_b = 1;

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
			if (opt == "b") ss >> arg_b;
			else
			{
			help:
				std::cout << "c_pcb_dsn [switches] [filename]\neg. c_pcb_dsn -b 6 test1.dsn\n";
				std::cout << "reads from stdin if no filename.\n";
				std::cout << "-b: border gap, default 1\n";
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

	//create tree from input
	auto tree = read_tree(in);

	auto structure_root = search_tree(tree, "structure");
	auto layer_to_index_map = std::map<std::string, int>{};
	const auto units = 1000.0f;
	auto num_layers = 0;
	auto minx = float(1000000.0);
	auto miny = float(1000000.0);
	auto maxx = float(-1000000.0);
	auto maxy = float(-1000000.0);
	auto default_rule = rule{0.25, 0.25};
	auto default_via = std::string{"Via[0-1]_600:400_um"};
	for (auto &&structure_node : structure_root->m_branches)
	{
		if (structure_node.m_value == "layer")
		{
			num_layers++;
			auto layer_index = 0;
			auto layer_name = structure_node.m_branches[0].m_value;
			for (auto &&layer_node : structure_node.m_branches)
			{
				if (layer_node.m_value == "property")
				{
					for (auto &&property_node : layer_node.m_branches)
					{
						if (property_node.m_value == "index")
						{
							ss_reset(ss, property_node.m_branches[0].m_value);
							ss >> layer_index;
						}
					}
				}
			}
			layer_to_index_map[layer_name] = layer_index;
		}
		else if (structure_node.m_value == "via")
		{
			for (auto &&via_node : structure_node.m_branches)
			{
				default_via = via_node.m_value;
			}
		}
		else if (structure_node.m_value == "rule")
		{
			for (auto &&rule_node : structure_node.m_branches)
			{
				if (rule_node.m_value == "width")
				{
					ss_reset(ss, rule_node.m_branches[0].m_value);
					ss >> default_rule.m_radius;
					default_rule.m_radius /= (2 * units);
				}
				else if ((rule_node.m_value == "clear"
						|| rule_node.m_value == "clearance")
						&& rule_node.m_branches.size() == 1)
				{
					ss_reset(ss, rule_node.m_branches[0].m_value);
					ss >> default_rule.m_gap;
					default_rule.m_gap /= (2 * units);
				}
			}
		}
		else if (structure_node.m_value == "boundary")
		{
			for (auto &&boundary_node : structure_node.m_branches)
			{
				if (boundary_node.m_value == "path")
				{
					for (auto cords = begin(boundary_node.m_branches) + 2;
					 		cords != end(boundary_node.m_branches); cords += 2)
					{
						float px, py;
						ss_reset(ss, cords->m_value);
						ss >> px;
						ss_reset(ss, (cords+1)->m_value);
						ss >> py;
						px /= units;
						py /= -units;
						minx = std::min(px, minx);
						maxx = std::max(px, maxx);
						miny = std::min(py, miny);
						maxy = std::max(py, maxy);
					}
				}
				else if (boundary_node.m_value == "rect")
				{
					float x1, y1, x2, y2;
					ss_reset(ss, boundary_node.m_branches[1].m_value);
					ss >> x1;
					ss_reset(ss, boundary_node.m_branches[2].m_value);
					ss >> y1;
					ss_reset(ss, boundary_node.m_branches[3].m_value);
					ss >> x2;
					ss_reset(ss, boundary_node.m_branches[4].m_value);
					ss >> y2;
					x1 /= units;
					y1 /= -units;
					x2 /= units;
					y2 /= -units;
					minx = std::min(x1, minx);
					maxx = std::max(x1, maxx);
					miny = std::min(y1, miny);
					maxy = std::max(y1, maxy);
					minx = std::min(x2, minx);
					maxx = std::max(x2, maxx);
					miny = std::min(y2, miny);
					maxy = std::max(y2, maxy);
				}
			}
		}
	}

	auto library_root = search_tree(tree, "library");
	auto name_to_component_map = std::map<std::string, component>{};
	auto name_to_padstack_map = std::map<std::string, std::map<int, padstack>>{};
	for (auto &&library_node : library_root->m_branches)
	{
		if (library_node.m_value == "image")
		{
			auto component_name = library_node.m_branches[0].m_value;
			auto the_comp = component{component_name, std::map<std::string, pin>{}};
			for (auto image_node = begin(library_node.m_branches) + 1;
					image_node != end(library_node.m_branches); ++image_node)
			{
				if (image_node->m_value == "pin")
				{
					auto the_pin = pin{};
					the_pin.m_form = image_node->m_branches[0].m_value;
					if (image_node->m_branches[1].m_value == "rotate")
					{
						ss_reset(ss, image_node->m_branches[1].m_branches[0].m_value);
						ss >> the_pin.m_angle;
						the_pin.m_name = image_node->m_branches[2].m_value;
						ss_reset(ss, image_node->m_branches[3].m_value);
						ss >> the_pin.m_x;
						ss_reset(ss, image_node->m_branches[4].m_value);
						ss >> the_pin.m_y;
						the_pin.m_angle *= (M_PI / 180.0);
					}
					else
					{
						the_pin.m_angle = 0.0;
						the_pin.m_name = image_node->m_branches[1].m_value;
						ss_reset(ss, image_node->m_branches[2].m_value);
						ss >> the_pin.m_x;
						ss_reset(ss, image_node->m_branches[3].m_value);
						ss >> the_pin.m_y;
					}
					the_pin.m_x /= units;
					the_pin.m_y /= -units;
					the_comp.m_pin_map[the_pin.m_name] = the_pin;
				}
			}
			name_to_component_map[component_name] = the_comp;
		}
		else if (library_node.m_value == "padstack")
		{
			for (auto padstack_node = begin(library_node.m_branches) + 1;
					padstack_node != end(library_node.m_branches); ++padstack_node)
			{
				if (padstack_node->m_value == "shape")
				{
					auto points = points_2d{};
					auto the_rule = default_rule;
					if (padstack_node->m_branches[0].m_value == "circle")
					{
						ss_reset(ss, padstack_node->m_branches[0].m_branches[1].m_value);
						ss >> the_rule.m_radius;
						the_rule.m_radius /= (2 * units);
					}
					else if (padstack_node->m_branches[0].m_value == "path")
					{
						ss_reset(ss, padstack_node->m_branches[0].m_branches[1].m_value);
						ss >> the_rule.m_radius;
						the_rule.m_radius /= (2 * units);
						float x1, y1, x2, y2;
						ss_reset(ss, padstack_node->m_branches[0].m_branches[2].m_value);
						ss >> x1;
						ss_reset(ss, padstack_node->m_branches[0].m_branches[3].m_value);
						ss >> y1;
						ss_reset(ss, padstack_node->m_branches[0].m_branches[4].m_value);
						ss >> x2;
						ss_reset(ss, padstack_node->m_branches[0].m_branches[5].m_value);
						ss >> y2;
						if (x1 != 0.0
							|| x2 != 0.0
							|| y1 != 0.0
							|| y2 != 0.0)
						{
							x1 /= units;
							y1 /= -units;
							x2 /= units;
							y2 /= -units;
							points.push_back(point_2d{x1, y1});
							points.push_back(point_2d{x2, y2});
						}
					}
					else if (padstack_node->m_branches[0].m_value == "rect")
					{
						the_rule.m_radius = 0.0;
						float x1, y1, x2, y2;
						ss_reset(ss, padstack_node->m_branches[0].m_branches[1].m_value);
						ss >> x1;
						ss_reset(ss, padstack_node->m_branches[0].m_branches[2].m_value);
						ss >> y1;
						ss_reset(ss, padstack_node->m_branches[0].m_branches[3].m_value);
						ss >> x2;
						ss_reset(ss, padstack_node->m_branches[0].m_branches[4].m_value);
						ss >> y2;
						x1 /= units;
						y1 /= -units;
						x2 /= units;
						y2 /= -units;
						points.push_back(point_2d{x1, y1});
						points.push_back(point_2d{x2, y1});
						points.push_back(point_2d{x2, y2});
						points.push_back(point_2d{x1, y2});
						points.push_back(point_2d{x1, y1});
					}
					else if (padstack_node->m_branches[0].m_value == "polygon")
					{
						the_rule.m_radius = 0.0;
						for (auto poly_node = begin(padstack_node->m_branches[0].m_branches) + 2;
							 poly_node != end(padstack_node->m_branches[0].m_branches); poly_node += 2)
						{
							float x1, y1;
							ss_reset(ss, poly_node[0].m_value);
							ss >> x1;
							ss_reset(ss, poly_node[1].m_value);
							ss >> y1;
							x1 /= units;
							y1 /= -units;
							points.push_back(point_2d{x1, y1});
						}
						points.push_back(points.front());
					}
					auto layer_index = layer_to_index_map[padstack_node->m_branches[0].m_branches[0].m_value];
					name_to_padstack_map[library_node.m_branches[0].m_value][layer_index] = padstack{points, the_rule};
				}
			}
		}
	}

	auto placement_root = search_tree(tree, "placement");
	auto name_to_instance_map = std::map<std::string, instance>{};
	for (auto &&placement_node : placement_root->m_branches)
	{
		if (placement_node.m_value == "component")
		{
			auto component_name = placement_node.m_branches[0].m_value;
			for (auto component_node = begin(placement_node.m_branches) + 1;
					component_node != end(placement_node.m_branches); ++component_node)
			{
				if (component_node->m_value == "place")
				{
					auto the_instance = instance{};
					auto instance_name = component_node->m_branches[0].m_value;
					the_instance.m_name = instance_name;
					the_instance.m_comp = component_name;
					ss_reset(ss, component_node->m_branches[1].m_value);
					ss >> the_instance.m_x;
					ss_reset(ss, component_node->m_branches[2].m_value);
					ss >> the_instance.m_y;
					the_instance.m_side = component_node->m_branches[3].m_value;
					ss_reset(ss, component_node->m_branches[4].m_value);
					ss >> the_instance.m_angle;
					the_instance.m_angle *= -(M_PI / 180.0);
					the_instance.m_x /= units;
					the_instance.m_y /= -units;
					name_to_instance_map[instance_name] = the_instance;
				}
			}
		}
	}

	auto all_pads = pads{};
	for (auto &&inst : name_to_instance_map)
	{
		auto instance = inst.second;
		auto component = name_to_component_map[instance.m_comp];
		for (auto &&p : component.m_pin_map)
		{
			auto pin = p.second;
			auto m_x = pin.m_x;
			auto m_y = pin.m_y;
			if (instance.m_side != "front") m_x = -m_x;
			auto s = sin(instance.m_angle);
			auto c = cos(instance.m_angle);
			auto px = float((c*m_x - s*m_y) + instance.m_x);
			auto py = float((s*m_x + c*m_y) + instance.m_y);
			for (auto &&p : name_to_padstack_map[pin.m_form])
			{
				auto tp = point_3d{px, py, (float)p.first};
				auto cords = shape_to_cords(p.second.m_shape, pin.m_angle, instance.m_angle);
				all_pads.push_back(pad{p.second.m_rule.m_radius, p.second.m_rule.m_gap, tp, cords});
			}
			minx = std::min(px, minx);
			maxx = std::max(px, maxx);
			miny = std::min(py, miny);
			maxy = std::max(py, maxy);
		}
	}

	auto network_root = search_tree(tree, "network");
	auto name_to_circuit_map = std::map<std::string, circuit>{};
	for (auto &&network_node : network_root->m_branches)
	{
		if (network_node.m_value == "class")
		{
			auto the_circuit = circuit{default_via, default_rule};
			for (auto &&class_node : network_node.m_branches)
			{
				if (class_node.m_value == "rule")
				{
					for (auto &&dims : class_node.m_branches)
					{
						if (dims.m_value == "width")
						{
							ss_reset(ss, dims.m_branches[0].m_value);
							ss >> the_circuit.m_rule.m_radius;
							the_circuit.m_rule.m_radius /= (2 * units);
						}
						else if ((dims.m_value == "clearance"
								|| dims.m_value == "clear")
								&& dims.m_branches.size() == 1)
						{
							ss_reset(ss, dims.m_branches[0].m_value);
							ss >> the_circuit.m_rule.m_gap;
							the_circuit.m_rule.m_gap /= (2 * units);
						}
					}
				}
				else if (class_node.m_value == "circuit")
				{
					for (auto &&circuit_node : class_node.m_branches)
					{
						if (circuit_node.m_value == "use_via")
						{
							the_circuit.m_via = circuit_node.m_branches[0].m_value;
						}
					}
				}
			}
			for (auto &&netname : network_node.m_branches)
			{
				if (netname.m_branches.empty()) name_to_circuit_map[netname.m_value] = the_circuit;
			}
		}
	}

	auto wiring_root = search_tree(tree, "wiring");
	auto net_to_wires_map = std::map<std::string, paths>{};
	for (auto &&wiring_node : wiring_root->m_branches)
	{
		if (wiring_node.m_value == "wire")
		{
			auto z = 0.0f;
			auto radius = 0.0f;
			auto wire = path{};
			for (auto &&wire_node : wiring_node.m_branches)
			{
				if (wire_node.m_value == "path"
					|| wire_node.m_value == "polyline_path")
				{
					z = layer_to_index_map[wire_node.m_branches[0].m_value];
					ss_reset(ss, wire_node.m_branches[1].m_value);
					ss >> radius;
					radius /= (2 * units);

					for (auto poly_node = begin(wire_node.m_branches) + 2;
							poly_node != end(wire_node.m_branches); poly_node += 2)
					{
						float x, y;
						ss_reset(ss, poly_node[0].m_value);
						ss >> x;
						ss_reset(ss, poly_node[1].m_value);
						ss >> y;
						x /= units;
						y /= -units;
						wire.push_back(point_3d(x, y, z));
					}
				}
				else if (wire_node.m_value == "net")
				{
					net_to_wires_map[wire_node.m_branches[0].m_value].emplace_back(std::move(wire));
				}
			}
		}
		else if (wiring_node.m_value == "via")
		{
			float x, y;
			ss_reset(ss, wiring_node.m_branches[1].m_value);
			ss >> x;
			ss_reset(ss, wiring_node.m_branches[2].m_value);
			ss >> y;
			x /= units;
			y /= -units;

			for (auto wire_node = begin(wiring_node.m_branches) + 3;
					wire_node != end(wiring_node.m_branches); ++wire_node)
			{
				if (wire_node->m_value == "net")
				{
					net_to_wires_map[wire_node->m_branches[0].m_value].emplace_back(
						path{point_3d{x, y, 0}, point_3d{x, y, float(num_layers - 1)}});
				}
			}
		}
	}

	auto the_tracks = tracks{};
	for (auto &&network_node : network_root->m_branches)
	{
		if (network_node.m_value == "net")
		{
			for (auto &net_node : network_node.m_branches)
			{
				if (net_node.m_value == "pins")
				{
					auto the_pads = pads{};
					for (auto &&p : net_node.m_branches)
					{
						auto pin_info = split(p.m_value, '-');
						auto instance_name = pin_info[0];
						auto pin_name = pin_info[1];
						auto instance = name_to_instance_map[instance_name];
						auto component = name_to_component_map[instance.m_comp];
						auto pin = component.m_pin_map[pin_name];
						auto m_x = pin.m_x;
						auto m_y = pin.m_y;
						if (instance.m_side != "front") m_x = -m_x;
						auto s = sin(instance.m_angle);
						auto c = cos(instance.m_angle);
						auto px = float((c*m_x - s*m_y) + instance.m_x);
						auto py = float((s*m_x + c*m_y) + instance.m_y);
						for (auto &&p : name_to_padstack_map[pin.m_form])
						{
							auto tp = point_3d{px, py, (float)p.first};
							auto cords = shape_to_cords(p.second.m_shape, pin.m_angle, instance.m_angle);
							auto term = pad{p.second.m_rule.m_radius, p.second.m_rule.m_gap, tp, cords};
							the_pads.push_back(term);
							all_pads.erase(std::find(begin(all_pads), end(all_pads), term));
						}
					}
					auto &&net_name = network_node.m_branches[0].m_value;
					auto &&net = name_to_circuit_map[net_name];
					auto track_rule = net.m_rule;
					auto via_rule = name_to_padstack_map[net.m_via][0].m_rule;
					the_tracks.push_back(track{track_rule.m_radius, via_rule.m_radius, track_rule.m_gap,
												the_pads, net_to_wires_map[net_name]});
				}
			}
		}
	}
	the_tracks.push_back(track{0.0, 0.0, 0.0, all_pads, {}});

	//output pcb format
	auto border = double(arg_b);
	std::cout << "(" << int(maxx-minx+(border*2)+0.5)
	 			<< " " << int(maxy-miny+(border*2)+0.5)
				<< " " << num_layers << ")\n";
	for (auto &&track : the_tracks)
	{
		std::cout << "(" << track.m_radius << " " << track.m_via << " " << track.m_gap << " (";
		for (auto i = 0; i < static_cast<int>(track.m_pads.size()); ++i)
		{
			auto &&term = track.m_pads[i];
			std::cout << "(" << term.m_radius << " " << term.m_gap
				<< " (" << term.m_term.m_x-float(minx-border)
				<< " " << term.m_term.m_y-float(miny-border)
				<< " " << term.m_term.m_z << ") (";
			for (auto j = 0; j < static_cast<int>(term.m_shape.size()); ++j)
			{
				auto cord = term.m_shape[j];
				std::cout << "(" << cord.m_x << " " << cord.m_y << ")";
			}
			std::cout << "))";
		}
		std::cout << ") (";
		for (auto i = 0; i < static_cast<int>(track.m_paths.size()); ++i)
		{
			std::cout << "(";
			auto &&p = track.m_paths[i];
			for (auto j = 0; j < static_cast<int>(p.size()); ++j)
			{
				std::cout << "(" << p[j].m_x-float(minx-border)
					<< " " << p[j].m_y-float(miny-border)
					<< " " << p[j].m_z << ")";
			}
			std::cout << ")";
		}
		std::cout << "))\n";
	}
}
