## How to Run
Build the package by running `catkin_make` in the root folder, then do `source devel/setup.bash` (or `source devel/setup.zsh`if you are using zshell) so that your terminal is aware of the newly compiled binaries.
Then finally, run the this command,
```bash
roslaunch server_node map.launch
```
## Configuration
Within the file `config/cfg.yaml` lies three parameters.
  - `center_clearence_radius` is the length of the rectangular clearence created at the center of the map in terms of cells, each 0.1 meters across.
  - `obstacle_length` is the length of the rectangular obstacles situated around the map.
  - `obstacle_count` is the number of obstacles scattered across the map.
You may modify these as you wish to attain different setups to test your algorithms with.

## Considerations
The map is spawned in the `map` reference frame, with an identity transform mapping it to the `world` transform, you can easily view it through Rviz without any problems, it is published at a frequency of 2.
