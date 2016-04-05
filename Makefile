all:		pcb dsn2pcb

pcb:		pcb.cpp router.cpp layer.cpp mymath.cpp
			clang++ -O2 --std=c++14 pcb.cpp router.cpp layer.cpp mymath.cpp -opcb

dsn2pcb:	dsn2pcb.cpp
			clang++ -O2 --std=c++14 dsn2pcb.cpp -odsn2pcb

clean:
			rm pcb dsn2pcb
