# C-PCB

![](./test3.png)

## C++14 PCB router and solver.

Requires the glfw3 libs to be installed if you wish to build the viewer
application.

Build everything with:

```
make -j
```

Build just the parts you need with:

```
make -j [c_pcb c_pcb_dsn c_pcb_view]
```

Example command lines would be:

```
./c_pcb --v 1 netlist.pcb | ./c_pcb_view --o 1 --s 7
./c_pcb_dsn test1.dsn | ./c_pcb --v 1 --z 8 --q 10 --r 2 | ./c_pcb_view
```

You can drop the output to a file and view it as an animation with:

```
./c_pcb --v 1 netlist.pcb > anim
./c_pcb_view anim
```

`-h` or `--help` for help on either app.

Format of .pcb input file or stdin is:

```
DIMS : [width, height, depth]
POINT2D : (x, y)
POINT3D : (x, y, z)
SHAPE : [POINT2D, ...]
PATH : [POINT3D, ...]
PATHS : [PATH, ...]
PAD : (pad_radius, pad_gap, POINT3D, SHAPE)
PADS : [PAD, ...]
TRACK : [track_radius, via_radius, track_gap, PADS, PATHS]
PCB : DIMS TRACK ...
```

You can stop a netlist early by just putting and empty track:

```
[]
```

For example:

```
DIMS
TRACK
TRACK
[]
TRACK
TRACK
```

## More screenshots
![](./test5.png)
![](./test1.png)
