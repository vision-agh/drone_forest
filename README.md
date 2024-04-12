# Drone forest

Toy environment for UAV flying through the random forest.

## How to start learning with TPGs

To start TPG learning process you need to build and run `run_gegelati` CMake target.

During learning, the program will iteratively show an evaluation of current UAV performance by displaying the rendered environment window.
You can control the execution of the program by pressing the following keys on active display window (internally it uses the OpenCV's waitKey() method to read inputs):

- `q` - quit program,
- `h` - hide evaluation (in fact: DO NOT perform evaluation as it is only for user), much faster learning process,
- `s` - show evaluation

Program also outputs some data to files that are stored in the directory printed by the program in the command line at the beginning.
