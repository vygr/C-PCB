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

#define GLFW_INCLUDE_GLCOREARB
#ifdef __linux__
	#include "GL/glew.h"
#endif
#include "GLFW/glfw3.h"
#include "router.h"
#include <limits>
#include <iostream>
#include <fstream>
#include <sstream>

extern point_2d add_2d(const point_2d &p1, const point_2d &p2);
extern points_2d torus_as_tristrip(const point_2d &p, float radius1, float radius2, int resolution);
extern points_2d circle_as_trifan(const point_2d &p, float radius, int resolution);
extern points_2d thicken_path_as_tristrip(const points_2d &path, float radius, int capstyle, int joinstyle, int resolution);

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

//read, [width, height, depth]
auto read_dimentions(std::istream &in)
{
	if (read_until(in, '[')) exit(1);
	auto dims = pcb::dims{};
	in >> dims.m_width;
	in.ignore(std::numeric_limits<std::streamsize>::max(), ',');
	in >> dims.m_height;
	in.ignore(std::numeric_limits<std::streamsize>::max(), ',');
	in >> dims.m_depth;
	if (read_until(in, ']')) exit(1);
	return dims;
}

//read, (x, y)
auto read_point_2d(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto p = point_2d{};
	in >> p.m_x;
	in.ignore(std::numeric_limits<std::streamsize>::max(), ',');
	in >> p.m_y;
	if (read_until(in, ')')) exit(1);
	return p;
}

//read, (x, y, z)
auto read_point_3d(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto p = point_3d{};
	in >> p.m_x;
	in.ignore(std::numeric_limits<std::streamsize>::max(), ',');
	in >> p.m_y;
	in.ignore(std::numeric_limits<std::streamsize>::max(), ',');
	in >> p.m_z;
	if (read_until(in, ')')) exit(1);
	return p;
}

//read, [(x, y), ...]
auto read_shape(std::istream &in)
{
	if (read_until(in, '[')) exit(1);
	auto cords = points_2d{};
	for (;;)
	{
		if (in.peek() == ']') break;
		cords.push_back(read_point_2d(in));
	}
	if (read_until(in, ']')) exit(1);
	return cords;
}

//read, [(x, y, z), ...]
auto read_path(std::istream &in)
{
	if (read_until(in, '[')) exit(1);
	auto cords = path{};
	for (;;)
	{
		if (in.peek() == ']') break;
		cords.push_back(read_point_3d(in));
	}
	if (read_until(in, ']')) exit(1);
	return cords;
}

//read, [[(x, y, z), ...], ...]
auto read_paths(std::istream &in)
{
	if (read_until(in, '[')) exit(1);
	auto t = paths{};
	for (;;)
	{
		if (in.peek() == ']') break;
		t.push_back(read_path(in));
	}
	if (read_until(in, ']')) exit(1);
	return t;
}

//read, (radius, gap, (x, y, z), [(x, y), ...])
auto read_terminal(std::istream &in)
{
	if (read_until(in, '(')) exit(1);
	auto t = terminal{};
	in >> t.m_radius;
	in.ignore(std::numeric_limits<std::streamsize>::max(), ',');
	in >> t.m_gap;
	in.ignore(std::numeric_limits<std::streamsize>::max(), '(');
	in >> t.m_term.m_x;
	in.ignore(std::numeric_limits<std::streamsize>::max(), ',');
	in >> t.m_term.m_y;
	in.ignore(std::numeric_limits<std::streamsize>::max(), ',');
	in >> t.m_term.m_z;
	in.ignore(std::numeric_limits<std::streamsize>::max(), ',');
	t.m_shape = read_shape(in);
	if (read_until(in, ')')) exit(1);
	return t;
}

//read all terminals for one track
auto read_terminals(std::istream &in)
{
	if (read_until(in, '[')) exit(1);
	auto t = terminals{};
	for (;;)
	{
		if (in.peek() == ']') break;
		t.push_back(read_terminal(in));
	}
	if (read_until(in, ']')) exit(1);
	return t;
}

//read one track
auto read_track(std::istream &in)
{
	if (read_until(in, '[')) return std::pair<track, bool>(track{}, true);
	if (in.peek() == ']') return std::pair<track, bool>(track{}, true);
	auto t = track{};
	in >> t.m_radius;
	in.ignore(std::numeric_limits<std::streamsize>::max(), ',');
	in >> t.m_via;
	in.ignore(std::numeric_limits<std::streamsize>::max(), ',');
	in >> t.m_gap;
	in.ignore(std::numeric_limits<std::streamsize>::max(), ',');
	t.m_terms = read_terminals(in);
	t.m_paths = read_paths(in);
	if (read_until(in, ']')) exit(1);
	return std::pair<track, bool>(t, false);
}

//compile gl shader
unsigned int compile_shader(unsigned int type, std::string source)
{
	auto shader = glCreateShader(type);
	const char *c_str = source.c_str();
	glShaderSource(shader, 1, &c_str, NULL);
	glCompileShader(shader);
	GLint status;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
	if (!status)
	{
		GLint length;
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
		std::string log(length, ' ');
		glGetShaderInfoLog(shader, length, &length, &log[0]);
		std::cerr << log;
		return 0;
	}
	return shader;
}

auto new_program(std::string vertex_source, std::string fragment_source)
{
	auto vertex_shader = compile_shader(GL_VERTEX_SHADER, vertex_source);
	auto fragment_shader = compile_shader(GL_FRAGMENT_SHADER, fragment_source);
	auto prog = glCreateProgram();
	glAttachShader(prog, vertex_shader);
	glAttachShader(prog, fragment_shader);
	glLinkProgram(prog);
	glDeleteShader(vertex_shader);
	glDeleteShader(fragment_shader);
	return prog;
}

//load shader progs
auto make_program(std::string vert_file_name, std::string frag_file_name)
{
	std::ifstream vs(vert_file_name);
	std::stringstream vert_source;
	vert_source << vs.rdbuf();

	std::ifstream fs(frag_file_name);
	std::stringstream frag_source;
	frag_source << fs.rdbuf();

	return new_program(vert_source.str(), frag_source.str());
}

//draw a line strip polygon
auto draw_polygon_strip(const point_2d &offset, const points_2d &data)
{
	auto vertex_buffer_data = points_2d{};
	vertex_buffer_data.reserve(data.size());
	for (auto &p : data)
	{
		vertex_buffer_data.push_back(add_2d(p, offset));
	}
	glBufferData(GL_ARRAY_BUFFER, vertex_buffer_data.size()*8, &vertex_buffer_data[0], GL_STATIC_DRAW);
	glDrawArrays(GL_LINE_STRIP, 0, int(vertex_buffer_data.size()));
}

//draw a triangle strip polygon
auto draw_filled_polygon_strip(const point_2d &offset, const points_2d &data)
{
	auto vertex_buffer_data = points_2d{};
	vertex_buffer_data.reserve(data.size());
	for (auto &p : data)
	{
		vertex_buffer_data.push_back(add_2d(p, offset));
	}
	glBufferData(GL_ARRAY_BUFFER, vertex_buffer_data.size()*8, &vertex_buffer_data[0], GL_STATIC_DRAW);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, int(vertex_buffer_data.size()));
}

//draw a triangle strip polygon
auto draw_filled_polygon_fan(const point_2d &offset, const points_2d &data)
{
	auto vertex_buffer_data = points_2d{};
	vertex_buffer_data.reserve(data.size());
	for (auto &p : data)
	{
		vertex_buffer_data.push_back(add_2d(p, offset));
	}
	glBufferData(GL_ARRAY_BUFFER, vertex_buffer_data.size()*8, &vertex_buffer_data[0], GL_STATIC_DRAW);
	glDrawArrays(GL_TRIANGLE_FAN, 0, int(vertex_buffer_data.size()));
}

//create circle polygon
auto create_filled_circle(float radius)
{
	static auto circle_map = std::map<float, points_2d>{};
	auto circle_itr = circle_map.find(radius);
	if (circle_itr != end(circle_map)) return &circle_itr->second;
	circle_map[radius] = circle_as_trifan(point_2d{0.0, 0.0}, radius, 32);
	return &circle_map[radius];
}

//draw layers
auto draw_layers(const tracks &ts, int pcb_height, int pcb_depth, float arg_m, float arg_s, float s)
{
	for (auto depth = pcb_depth - 1; depth > -1; --depth)
	{
		//draw track
		auto yoffset = float((pcb_height + (arg_m * 2)) * arg_s * depth);
		for (auto &t : ts)
		{
			//draw paths
			for (auto &path : t.m_paths)
			{
				auto start = 0;
				auto end = 0;
				for (end = 0; end < static_cast<int>(path.size()); ++end)
				{
					if (path[start].m_z != path[end].m_z)
					{
						if (path[start].m_z == float(depth))
						{
							if ((end - start) > 1)
							{
								auto points = points_2d{};
								points.reserve(end - start);
								for (auto i = begin(path) + start; i != begin(path) + end; ++i)
								{
									points.push_back(point_2d{i->m_x, i->m_y});
								}
								draw_filled_polygon_strip(point_2d{0.0, yoffset},
									thicken_path_as_tristrip(points, t.m_radius + (t.m_gap * s), 3, 2, 16));
							}
						}
						start = end;
					}
				}
				if (path[start].m_z == float(depth))
				{
					if ((end - start) > 1)
					{
						auto points = points_2d{};
						points.reserve(end - start);
						for (auto i = begin(path) + start; i != begin(path) + end; ++i)
						{
							points.push_back(point_2d{i->m_x, i->m_y});
						}
						draw_filled_polygon_strip(point_2d{0.0, yoffset},
							thicken_path_as_tristrip(points, t.m_radius + (t.m_gap * s), 3, 2, 16));
					}
				}
			}
		}
		//draw terminals and vias
		for (auto &t : ts)
		{
			for (auto &path : t.m_paths)
			{
				for (auto i = 0; i < (static_cast<int>(path.size()) - 1); ++i)
				{
					if (path[i].m_z != path[i+1].m_z)
					{
						draw_filled_polygon_fan(point_2d{path[i].m_x, path[i].m_y + yoffset},
							*create_filled_circle(t.m_via + (t.m_gap * s)));
					}
				}
			}
			for (auto &term : t.m_terms)
			{
				if (term.m_term.m_z != float(depth)) continue;
				if (term.m_shape.empty())
				{
					draw_filled_polygon_fan(point_2d{term.m_term.m_x, term.m_term.m_y + yoffset},
						*create_filled_circle(term.m_radius + (term.m_gap * s)));
				}
				else
				{
					auto points = points_2d{};
					points.reserve(term.m_shape.size());
					for (auto &cord : term.m_shape)
					{
						points.push_back(point_2d{cord.m_x, cord.m_y});
					}
					if ((s != 0) || (term.m_radius != 0))
					{
						draw_filled_polygon_strip(point_2d{0.0, yoffset},
							thicken_path_as_tristrip(points, term.m_radius + (term.m_gap * s), 3, 2, 16));
					}
					else
					{
						draw_filled_polygon_fan(point_2d{0.0, yoffset}, points);
					}
				}
			}
		}
	}
}

void ss_reset(std::stringstream &ss, std::string s)
{
	ss.str(s);
	ss.clear();
}

static void error_callback(int, const char* description)
{
	std::cerr << description;
}

static void key_callback(GLFWwindow* window, int key, int, int action, int)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);
}

int main(int argc, char *argv[])
{
	//process comand args
	auto use_file = false;
	std::ifstream arg_infile;
	auto arg_s = 9;
	auto arg_o = 0;
	auto arg_m = 2;

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
			if (opt == "s") ss >> arg_s;
			else if (opt == "o") ss >> arg_o;
			else if (opt == "m") ss >> arg_m;
			else
			{
			help:
				std::cout << "c_pcb_view [switches] [filename]\neg. c_pcb_view -s 9 -o 1 anim\n";
				std::cout << "reads from stdin if no filename.\n";
				std::cout << "-s:  scale factor, default 9\n";
				std::cout << "-o:  overlay modes 0..1, default 0\n";
				std::cout << "-m:  margin 0..10, default 2\n";
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

	//read dimensions of the pcb
	auto dimensions = read_dimentions(in);
	auto pcb_width = dimensions.m_width;
	auto pcb_height = dimensions.m_height;
	auto pcb_depth = dimensions.m_depth;
	auto width = (pcb_width + (arg_m * 2)) * arg_s;
	auto height = (pcb_height + (arg_m * 2)) * arg_s;
	if (arg_o == 1) height *= pcb_depth;

	//create window
	glfwSetErrorCallback(error_callback);
	if (!glfwInit()) exit(EXIT_FAILURE);

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // needed for macs
	auto window = glfwCreateWindow(width, height, "PCB Viewer", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwMakeContextCurrent(window);
	glfwSetKeyCallback(window, key_callback);
	glfwSetInputMode(window, GLFW_STICKY_KEYS, 1);

	//set gl settings
	glGetError();
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glLineWidth(1.0);

	//create vertex array
	unsigned int vertex_array;
	glGenVertexArrays(1, &vertex_array);
	glBindVertexArray(vertex_array);

	//load shaders and get address of shader variables
	auto prog = make_program("VertexShader.vert", "FragmentShader.frag");
	auto vert_color_id = glGetUniformLocation(prog, "vert_color");
	auto vert_scale_id = glGetUniformLocation(prog, "vert_scale");
	auto vert_offset_id = glGetUniformLocation(prog, "vert_offset");

	//use the loaded shader program
	glUseProgram(prog);

	//set aspect and offset for 2D drawing
	glUniform2f(vert_scale_id, 2.0/float(width), -2.0/float(height));
	glUniform2f(vert_offset_id, -1.0, 1.0);

	//setup vertex buffer ready for use
	unsigned int vertex_buffer;
	glGenBuffers(1, &vertex_buffer);
	auto vertex_attrib = glGetAttribLocation(prog, "vert_vertex");
	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
	glEnableVertexAttribArray(vertex_attrib);
	glVertexAttribPointer(vertex_attrib, 2, GL_FLOAT, false, 0, 0);

	for (;;)
	{
		glfwPollEvents();
		if ((glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) || glfwWindowShouldClose(window)) break;

		//load track and exit if no track loaded
		auto ts = tracks{};
		for (;;)
		{
			auto result = read_track(in);
			if (result.second == true) break;
			ts.push_back(result.first);
		}
		if (ts.empty()) break;

		//scale tracks acording to window size
		auto scale = float(arg_s);
		auto border = float(arg_m * arg_s);
		for (auto &t : ts)
		{
			t.m_radius *= scale;
			t.m_via *= scale;
			t.m_gap *= scale;
			for (auto &term : t.m_terms)
			{
				term.m_radius *= scale;
				term.m_gap *= scale;
				term.m_term.m_x *= scale;
				term.m_term.m_y *= scale;
				term.m_term.m_x += border;
				term.m_term.m_y += border;
				for (auto &cord : term.m_shape)
				{
					cord.m_x *= scale;
					cord.m_y *= scale;
					cord.m_x += term.m_term.m_x;
					cord.m_y += term.m_term.m_y;
				}
			}
			for (auto &path : t.m_paths)
			{
				for (auto &node : path)
				{
					node.m_x *= scale;
					node.m_y *= scale;
					node.m_x += border;
					node.m_y += border;
				}
			}
		}

		//clear background
		glClear(GL_COLOR_BUFFER_BIT);

		if (arg_o == 0)
		{
			//draw paths for each layer
			static auto colors = std::vector<float>{
				1.0, 0.0, 0.0,
				0.0, 1.0, 0.0,
				0.0, 0.0, 1.0,
				1.0, 1.0, 0.0,
				0.0, 1.0, 1.0,
				1.0, 0.0, 1.0,
			};
			for (auto depth = pcb_depth - 1; depth > -1; --depth)
			{
				auto color = (depth % (colors.size() / 3)) * 3;
				glUniform4f(vert_color_id, colors[color], colors[color+1], colors[color+2], 0.5);
				for (auto &t : ts)
				{
					for (auto &path : t.m_paths)
					{
						auto start = 0;
						auto end = 0;
						for (end = 0; end < static_cast<int>(path.size()); ++end)
						{
							if (path[start].m_z != path[end].m_z)
							{
								if (path[start].m_z == float(depth))
								{
									if ((end - start) > 1)
									{
										auto points = points_2d{};
										points.reserve(end - start);
										for (auto i = begin(path) + start; i != begin(path) + end; ++i)
										{
											points.push_back(point_2d{i->m_x, i->m_y});
										}
										draw_filled_polygon_strip(point_2d{0.0, 0.0},
											thicken_path_as_tristrip(points, t.m_radius, 3, 2, 16));
									}
								}
								start = end;
							}
						}
						if (path[start].m_z == float(depth))
						{
							if ((end - start) > 1)
							{
								auto points = points_2d{};
								points.reserve(end - start);
								for (auto i = begin(path) + start; i != begin(path) + end; ++i)
								{
									points.push_back(point_2d{i->m_x, i->m_y});
								}
								draw_filled_polygon_strip(point_2d{0.0, 0.0},
									thicken_path_as_tristrip(points, t.m_radius, 3, 2, 16));
							}
						}
					}
				}
			}
			//draw terminals and vias
			glUniform4f(vert_color_id, 1.0, 1.0, 1.0, 1.0);
			for (auto &t : ts)
			{
				for (auto &path : t.m_paths)
				{
					for (auto i = 0; i < (static_cast<int>(path.size()) - 1); ++i)
					{
						if (path[i].m_z != path[i+1].m_z)
						{
							draw_filled_polygon_fan(point_2d{path[i].m_x, path[i].m_y},
								*create_filled_circle(t.m_via));
						}
					}
				}
				for (auto &term : t.m_terms)
				{
					if (term.m_shape.empty())
					{
						draw_filled_polygon_fan(point_2d{term.m_term.m_x, term.m_term.m_y},
							*create_filled_circle(term.m_radius));
					}
					else
					{
						auto points = points_2d{};
						points.reserve(term.m_shape.size());
						for (auto &cord : term.m_shape)
						{
							points.push_back(point_2d{cord.m_x, cord.m_y});
						}
						if (term.m_radius != 0)
						{
							draw_filled_polygon_strip(point_2d{0.0, 0.0},
								thicken_path_as_tristrip(points, term.m_radius, 3, 2, 16));
						}
						else
						{
							draw_filled_polygon_fan(point_2d{0.0, 0.0}, points);
						}
					}
				}
			}
		}
		else
		{
			//draw each layer in white
			glUniform4f(vert_color_id, 1.0, 1.0, 1.0, 1.0);
			draw_layers(ts, pcb_height, pcb_depth, arg_m, arg_s, 1);
			//draw each layer in black
			glUniform4f(vert_color_id, 0.0, 0.0, 0.0, 1.0);
			draw_layers(ts, pcb_height, pcb_depth, arg_m, arg_s, 0);
		}

		//show window just drawn
		glfwSwapBuffers(window);
	}

	//wait till exit or close button pressed, 'hold on last frame'
	for (;;)
	{
		glfwPollEvents();
		if ((glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) || glfwWindowShouldClose(window)) break;
	}

	//clean up
	glDeleteBuffers(1, &vertex_buffer);
	glDeleteVertexArrays(1, &vertex_array);
	glDeleteProgram(prog);
	glfwTerminate();
	return 0;
}
