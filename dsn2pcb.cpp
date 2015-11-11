#include "router.h"
#include "math.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

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
	points_2d m_shape;
};

struct circuit
{
	std::string m_via;
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
	auto arg_b = 0;

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
				std::cout << "dsn2pcb [switches] [filename]\neg. dsn2pcb -b 6 test1.dsn\n";
				std::cout << "reads from stdin if no filename.\n";
				std::cout << "-b: border gap, default 0\n";
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
	auto num_layers = 0;
	auto minx = float(1000000.0);
	auto miny = float(1000000.0);
	auto maxx = float(-1000000.0);
	auto maxy = float(-1000000.0);
	for (auto &structure_node : structure_root->m_branches)
	{
		if (structure_node.m_value == "layer") num_layers++;
		if (structure_node.m_value == "boundary")
		{
			for (auto boundary_node : structure_node.m_branches)
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
						px /= 1000.0;
						py /= -1000.0;
						minx = std::min(px, minx);
						maxx = std::max(px, maxx);
						miny = std::min(py, miny);
						maxy = std::max(py, maxy);
					}
				}
			}
		}
	}

	auto library_root = search_tree(tree, "library");
	auto component_map = std::map<std::string, component>{};
	auto rule_map = std::map<std::string, rule>{};
	for (auto &library_node : library_root->m_branches)
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
					the_pin.m_x /= 1000.0;
					the_pin.m_y /= -1000.0;
					the_comp.m_pin_map[the_pin.m_name] = the_pin;
				}
			}
			component_map[component_name] = the_comp;
		}
		if (library_node.m_value == "padstack")
		{
			for (auto padstack_node = begin(library_node.m_branches) + 1;
					padstack_node != end(library_node.m_branches); ++padstack_node)
			{
				if (padstack_node->m_value == "shape")
				{
					auto points = points_2d{};
					auto the_rule = rule{0.5, 0.125, {}};
					if (padstack_node->m_branches[0].m_value == "circle")
					{
						ss_reset(ss, padstack_node->m_branches[0].m_branches[1].m_value);
						ss >> the_rule.m_radius;
						the_rule.m_radius /= 2000.0;
					}
					if (padstack_node->m_branches[0].m_value == "path")
					{
						ss_reset(ss, padstack_node->m_branches[0].m_branches[1].m_value);
						ss >> the_rule.m_radius;
						the_rule.m_radius /= 2000.0;
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
							x1 /= 1000.0;
							y1 /= -1000.0;
							x2 /= 1000.0;
							y2 /= -1000.0;
							points.push_back(point_2d{x1, y1});
							points.push_back(point_2d{x2, y2});
						}
					}
					if (library_node.m_branches[1].m_branches[0].m_value == "rect")
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
						x1 /= 1000.0;
						y1 /= -1000.0;
						x2 /= 1000.0;
						y2 /= -1000.0;
						points.push_back(point_2d{x1, y1});
						points.push_back(point_2d{x2, y1});
						points.push_back(point_2d{x2, y2});
						points.push_back(point_2d{x1, y2});
						points.push_back(point_2d{x1, y1});
					}
					the_rule.m_shape = points;
					rule_map[library_node.m_branches[0].m_value] = the_rule;
				}
			}
		}
	}

	auto placement_root = search_tree(tree, "placement");
	auto instance_map = std::map<std::string, instance>{};
	for (auto &placement_node : placement_root->m_branches)
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
					the_instance.m_x /= 1000.0;
					the_instance.m_y /= -1000.0;
					instance_map[instance_name] = the_instance;
				}
			}
		}
	}

	auto all_terminals = terminals{};
	for (auto &inst : instance_map)
	{
		auto instance = inst.second;
		auto component = component_map[instance.m_comp];
		for (auto &p : component.m_pin_map)
		{
			auto pin = p.second;
			auto m_x = pin.m_x;
			auto m_y = pin.m_y;
			if (instance.m_side != "front") m_x = -m_x;
			auto s = sin(instance.m_angle);
			auto c = cos(instance.m_angle);
			auto px = float((c*m_x - s*m_y) + instance.m_x);
			auto py = float((s*m_x + c*m_y) + instance.m_y);
			auto pin_rule = rule_map[pin.m_form];
			auto tp = point_3d{px, py, 0.0};
			auto cords = shape_to_cords(pin_rule.m_shape, pin.m_angle, instance.m_angle);
			all_terminals.push_back(terminal{pin_rule.m_radius, pin_rule.m_gap, tp, cords});
			minx = std::min(px, minx);
			maxx = std::max(px, maxx);
			miny = std::min(py, miny);
			maxy = std::max(py, maxy);
		}
	}

	auto network_root = search_tree(tree, "network");
	auto circuit_map = std::map<std::string, circuit>{};
	for (auto &network_node : network_root->m_branches)
	{
		if (network_node.m_value == "class")
		{
			auto net_rule = rule{0.125, 0.125, {}};
			auto the_circuit = circuit{};
			for (auto &class_node : network_node.m_branches)
			{
				if (class_node.m_value == "rule")
				{
					for (auto &dims : class_node.m_branches)
					{
						if (dims.m_value == "width")
						{
							ss_reset(ss, dims.m_branches[0].m_value);
							ss >> net_rule.m_radius;
							net_rule.m_radius /= 2000.0;
						}
						if (dims.m_value == "clearance")
						{
							ss_reset(ss, dims.m_branches[0].m_value);
							ss >> net_rule.m_gap;
							net_rule.m_gap /= 2000.0;
						}
					}
				}
				if (class_node.m_value == "circuit")
				{
					for (auto &circuit_node : class_node.m_branches)
					{
						if (circuit_node.m_value == "use_via")
						{
							the_circuit.m_via = circuit_node.m_branches[0].m_value;
						}
					}
				}
			}
			the_circuit.m_rule = net_rule;
			for (auto &netname : network_node.m_branches)
			{
				if (netname.m_branches.empty()) circuit_map[netname.m_value] = the_circuit;
			}
		}
	}

	auto the_tracks = tracks{};
	for (auto &network_node : network_root->m_branches)
	{
		if (network_node.m_value == "net")
		{
			auto the_terminals = terminals{};
			for (auto &p : network_node.m_branches[1].m_branches)
			{
				auto pin_info = split(p.m_value, '-');
				auto instance_name = pin_info[0];
				auto pin_name = pin_info[1];
				auto instance = instance_map[instance_name];
				auto component = component_map[instance.m_comp];
				auto pin = component.m_pin_map[pin_name];
				auto m_x = pin.m_x;
				auto m_y = pin.m_y;
				if (instance.m_side != "front") m_x = -m_x;
				auto s = sin(instance.m_angle);
				auto c = cos(instance.m_angle);
				auto px = float((c*m_x - s*m_y) + instance.m_x);
				auto py = float((s*m_x + c*m_y) + instance.m_y);
				auto pin_rule = rule_map[pin.m_form];
				auto tp = point_3d{px, py, 0.0};
				auto cords = shape_to_cords(pin_rule.m_shape, pin.m_angle, instance.m_angle);
				auto term = terminal{pin_rule.m_radius, pin_rule.m_gap, tp, cords};
				the_terminals.push_back(term);
				all_terminals.erase(std::find(begin(all_terminals), end(all_terminals), term));
			}
			auto circuit = circuit_map[network_node.m_branches[0].m_value];
			auto net_rule = circuit.m_rule;
			auto via_rule = rule_map[circuit.m_via];
			the_tracks.push_back(track{net_rule.m_radius, via_rule.m_radius, net_rule.m_gap, the_terminals});
		}
	}
	the_tracks.push_back(track{0.0, 0.0, 0.0, all_terminals});

	//output pcb format
	auto border = double(arg_b);
	std::cout << "[" << int(maxx-minx+(border*2)+0.5)
	 			<< "," << int(maxy-miny+(border*2)+0.5)
				<< "," << num_layers << "]\n";
	for (auto &track : the_tracks)
	{
		std::cout << "[" << track.m_radius << "," << track.m_via << "," << track.m_gap << ",[";
		for (auto i = 0; i < static_cast<int>(track.m_terms.size()); ++i)
		{
			auto term = track.m_terms[i];
			std::cout << "(" << term.m_radius << "," << term.m_gap
			 			<< ",(" << term.m_term.m_x-float(minx-border)
						<< "," << term.m_term.m_y-float(miny-border)
						<< "," << term.m_term.m_z << "),[";
			for (auto j = 0; j < static_cast<int>(term.m_shape.size()); ++j)
			{
				auto cord = term.m_shape[j];
				std::cout << "(" << cord.m_x << "," << cord.m_y << ")";
				if (j != (static_cast<int>(term.m_shape.size()) - 1)) std::cout << ",";
			}
			std::cout << "])";
			if (i != (static_cast<int>(track.m_terms.size()) - 1)) std::cout << ",";
		}
		std::cout << "]]\n";
	}
}
