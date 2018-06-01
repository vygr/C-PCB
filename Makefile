all:		c_pcb c_pcb_dsn c_pcb_view

c_pcb:		c_pcb.cpp mymath.cpp mymath.h router.cpp router.h layer.cpp layer.h io.h
			c++ -O2 --std=c++14 c_pcb.cpp router.cpp layer.cpp mymath.cpp -oc_pcb

c_pcb_dsn:	c_pcb_dsn.cpp
			c++ -O2 --std=c++14 c_pcb_dsn.cpp -oc_pcb_dsn

c_pcb_view:	c_pcb_view.cpp mymath.cpp mymath.h io.h
			c++ -O2 --std=c++14 `pkg-config --cflags glfw3` c_pcb_view.cpp mymath.cpp -oc_pcb_view `pkg-config --static --libs glfw3`

clean:
			rm c_pcb c_pcb_dsn c_pcb_view
