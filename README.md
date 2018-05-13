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
[width, height, depth]
[track_radius, via_radius, track_gap, [(terminal_radius, terminal_gap, (x, y, z), [(x, y), ...]), ...]...]
```

You can stop a netlist early by just putting:

```
[]
```

For example:

```
[width, height, depth]
[track_radius, via_radius, track_gap, [(terminal_radius, terminal_gap, (x, y, z), [(x, y), ...]), ...]...]
[track_radius, via_radius, track_gap, [(terminal_radius, terminal_gap, (x, y, z), [(x, y), ...]), ...]...]
[]
[track_radius, via_radius, track_gap, [(terminal_radius, terminal_gap, (x, y, z), [(x, y), ...]), ...]...]
[track_radius, via_radius, track_gap, [(terminal_radius, terminal_gap, (x, y, z), [(x, y), ...]), ...]...]
```

Format of the viewer input is similar but has the track paths appended and the gaps removed:

```
[width, height, depth]
[track_radius, via_radius, track_gap, [(terminal_radius, terminal_gap, (x, y, z), [(x, y), ...]), ...]...], [(x, y, z), ...]]
```

## More screenshots
![](./test5.png)
![](./test1.png)
