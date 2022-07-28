## lqr_rftrk

Ongoing:\
Record data from simulation session in text file to parse and subsequently
plot using `pyplot`

Usage:\
At the root of this directory, build the packages:\
`colcon build --symlink-install`\
In another terminal, at the root of this directory, source:\
`. install/setup.bash`\
Then, on that same terminal:\
`ros2 launch lqr_rftrk lqr_rftrk.launch.py`\

Running the simulation should have files `states.txt` and `refs.txt` written.\
Run `python3 gazebo_plot.py` to see the simulation results graphically after
the simulation closes.\
Note: Running the simulation again results in those files being overwritten