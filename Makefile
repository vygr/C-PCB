all:		pcb dsn2pcb view

pcb:		pcb.cpp mymath.cpp mymath.h router.cpp router.h layer.cpp layer.h
			c++ -O2 --std=c++14 pcb.cpp router.cpp layer.cpp mymath.cpp -opcb

dsn2pcb:	dsn2pcb.cpp
			c++ -O2 --std=c++14 dsn2pcb.cpp -odsn2pcb

view:		view.cpp mymath.cpp mymath.h
			c++ -O2 --std=c++14 `pkg-config --cflags glfw3` view.cpp mymath.cpp -oview `pkg-config --static --libs glfw3`

clean:
			rm pcb dsn2pcb view
