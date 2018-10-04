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
#include "io.h"

extern point_2d add_2d(const point_2d &p1, const point_2d &p2);
extern points_2d torus_as_tristrip(const point_2d &p, double radius1, double radius2, int resolution);
extern points_2d circle_as_trifan(const point_2d &p, double radius, int resolution);
extern points_2d thicken_path_as_tristrip(const points_2d &path, double radius, int capstyle, int joinstyle, int resolution);

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
	glBufferData(GL_ARRAY_BUFFER, vertex_buffer_data.size()*sizeof(point_2d), &vertex_buffer_data[0], GL_STATIC_DRAW);
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
	glBufferData(GL_ARRAY_BUFFER, vertex_buffer_data.size()*sizeof(point_2d), &vertex_buffer_data[0], GL_STATIC_DRAW);
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
	glBufferData(GL_ARRAY_BUFFER, vertex_buffer_data.size()*sizeof(point_2d), &vertex_buffer_data[0], GL_STATIC_DRAW);
	glDrawArrays(GL_TRIANGLE_FAN, 0, int(vertex_buffer_data.size()));
}

//create circle polygon
auto create_filled_circle(double radius)
{
	static auto circle_map = std::map<double, points_2d>{};
	auto circle_itr = circle_map.find(radius);
	if (circle_itr != end(circle_map)) return &circle_itr->second;
	circle_map[radius] = circle_as_trifan(point_2d{0.0, 0.0}, radius, 32);
	return &circle_map[radius];
}

//draw layers
auto draw_layers(const tracks &ts, int pcb_height, int pcb_depth, double arg_m, double arg_s, double s)
{
	for (auto depth = pcb_depth - 1; depth > -1; --depth)
	{
		//draw track
		auto yoffset = double((pcb_height + (arg_m * 2)) * arg_s * depth);
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
						if (path[start].m_z == double(depth))
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
									thicken_path_as_tristrip(points, t.m_track_radius + (t.m_gap * s), 3, 2, 16));
							}
						}
						start = end;
					}
				}
				if (path[start].m_z == double(depth))
				{
					if ((end - start) > 1)
					{
						auto points = points_2d{};
						points.reserve(end - start);
						for (auto i = begin(path) + start; i != begin(path) + end; ++i)
						{
							points.push_back(point_2d{i->m_x, i->m_y});
						}
						if (t.m_track_radius)
						{
							draw_filled_polygon_strip(point_2d{0.0, yoffset},
								thicken_path_as_tristrip(points, t.m_track_radius + (t.m_gap * s), 3, 2, 16));
						}
					}
				}
			}
		}
		//draw pads and vias
		for (auto &t : ts)
		{
			for (auto &path : t.m_paths)
			{
				for (auto i = 0; i < (static_cast<int>(path.size()) - 1); ++i)
				{
					if (path[i].m_z != path[i+1].m_z)
					{
						draw_filled_polygon_fan(point_2d{path[i].m_x, path[i].m_y + yoffset},
							*create_filled_circle(t.m_via_radius + (t.m_gap * s)));
					}
				}
			}
			for (auto &term : t.m_pads)
			{
				if (term.m_pos.m_z != double(depth)) continue;
				if (term.m_shape.empty())
				{
					draw_filled_polygon_fan(point_2d{term.m_pos.m_x, term.m_pos.m_y + yoffset},
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
	auto pcb_depth = int(dimensions.m_depth);
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
	glUniform2f(vert_scale_id, 2.0/double(width), -2.0/double(height));
	glUniform2f(vert_offset_id, -1.0, 1.0);

	//setup vertex buffer ready for use
	unsigned int vertex_buffer;
	glGenBuffers(1, &vertex_buffer);
	auto vertex_attrib = glGetAttribLocation(prog, "vert_vertex");
	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
	glEnableVertexAttribArray(vertex_attrib);
	glVertexAttribPointer(vertex_attrib, 2, GL_DOUBLE, false, 0, 0);

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
		auto scale = double(arg_s);
		auto border = double(arg_m * arg_s);
		for (auto &t : ts)
		{
			t.m_track_radius *= scale;
			t.m_via_radius *= scale;
			t.m_gap *= scale;
			for (auto &term : t.m_pads)
			{
				term.m_radius *= scale;
				term.m_gap *= scale;
				term.m_pos.m_x *= scale;
				term.m_pos.m_y *= scale;
				term.m_pos.m_x += border;
				term.m_pos.m_y += border;
				for (auto &cord : term.m_shape)
				{
					cord.m_x *= scale;
					cord.m_y *= scale;
					cord.m_x += term.m_pos.m_x;
					cord.m_y += term.m_pos.m_y;
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
			static auto colors = std::vector<double>{
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
								if (path[start].m_z == double(depth))
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
											thicken_path_as_tristrip(points, t.m_track_radius, 3, 2, 16));
									}
								}
								start = end;
							}
						}
						if (path[start].m_z == double(depth))
						{
							if ((end - start) > 1)
							{
								auto points = points_2d{};
								points.reserve(end - start);
								for (auto i = begin(path) + start; i != begin(path) + end; ++i)
								{
									points.push_back(point_2d{i->m_x, i->m_y});
								}
								if (t.m_track_radius)
								{
									draw_filled_polygon_strip(point_2d{0.0, 0.0},
										thicken_path_as_tristrip(points, t.m_track_radius, 3, 2, 16));
								}
								else
								{
									draw_filled_polygon_strip(point_2d{0.0, 0.0},
										thicken_path_as_tristrip(points, 0.05 * scale, 3, 2, 16));
								}
							}
						}
					}
				}
			}
			//draw pads and vias
			glUniform4f(vert_color_id, 1.0, 1.0, 1.0, 0.5);
			for (auto &t : ts)
			{
				for (auto &path : t.m_paths)
				{
					for (auto i = 0; i < (static_cast<int>(path.size()) - 1); ++i)
					{
						if (path[i].m_z != path[i+1].m_z)
						{
							draw_filled_polygon_fan(point_2d{path[i].m_x, path[i].m_y},
								*create_filled_circle(t.m_via_radius));
							glUniform4f(vert_color_id, 0.0, 0.0, 0.0, 0.5);
							draw_filled_polygon_fan(point_2d{path[i].m_x, path[i].m_y},
								*create_filled_circle(t.m_via_radius * 0.5));
							glUniform4f(vert_color_id, 1.0, 1.0, 1.0, 0.5);
						}
					}
				}
				for (auto &term : t.m_pads)
				{
					if (term.m_shape.empty())
					{
						draw_filled_polygon_fan(point_2d{term.m_pos.m_x, term.m_pos.m_y},
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
					glUniform4f(vert_color_id, 0.0, 0.0, 0.0, 0.5);
					draw_filled_polygon_fan(point_2d{term.m_pos.m_x, term.m_pos.m_y},
						*create_filled_circle(0.75));
					glUniform4f(vert_color_id, 1.0, 1.0, 1.0, 0.5);
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
