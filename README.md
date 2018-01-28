# PID Controller

*Term 2, Project 9 of Udacity Self-Driving Car Nanodegree, by vuiseng9, Jan 2018*

The goal of this project is to implement a PID controller in C++ to manuever the vehicle around the lake race track in the Udacity Term 2 Simulator. Hyperparameter tuning of Proportional, Integral and Derivative gain is required to ensure a smooth manuever.

> [Demo video of the Final Tuned PID controller](ww)

## Program Design
The PID controller is designed to only actuate steering angle using the cross crack error (CTE) and the speed of vehicle is contrained with a constant throttle of 0.3. The program incorporates the twiddle algoritm outlined in the lesson to tune Proportional, Integral and Derivative term of the contoller. The cost function of the algorithm is a total sum of squared of (1) CTE error (2) steering angle - to minimize overall steering change (3) speed gap to 30 mph - to ensure the vehicle moves.

For easy user tuning, a CLI interface has been designed through [Taywee/args](https://github.com/Taywee/args) to allow setting of the initial conditions of hyperparameters.

- ``` ./pid ``` The program starts in **autonomous mode** with preset PID gain.

- ``` ./pid --kp <kp> --ki <ki> --kd <kd>``` The program starts in **autonomous mode** with user-specified PID gain.

- ``` ./pid -t ``` The program starts in **twiddle mode** with preset PID gain where max range of delta of PID gain is initiated at 1.

- ``` ./pid -t --dp <kp> --di <di> --dd <dd>``` The program starts in **twiddle mode** with preset PID gain and with user-specified max range of delta for each gain term.

- ``` ./pid -t --kp <kp> --ki <ki> --kd <kd> --dp <kp> --di <di> --dd <dd>``` The program starts in **twiddle mode** with user-specified PID gain and with user-specified max range of delta for each gain term.

- ```-n_step <number of step>``` This option is only for **twiddle mode** where each twiddle iteration is ended after n_step.

CLI help menu is as following.

```
$ ./pid -h
  ./pid {OPTIONS}

    an PID controller app that drives Udacity SDC Simulator Lake Track

  OPTIONS:

      -h, --help                        Display help menu
      kp, ki, kd need to coexist
        --kp=[float]                      set/initialize proportional gain
        --ki=[float]                      set/initialize integral gain
        --kd=[float]                      set/initialize derivative gain
      -t, --twiddle                     enable twiddle mode to tune gain
      --n_step=[int]                    set number of step per twiddle iteration
      dp, di, dd need to coexist
        --dp=[float]                      kp max tunable range
        --di=[float]                      ki max tunable range
        --dd=[float]                      kd max tunable range
      --cp=[characters...]              The character flag

    Running ./pid without any argument invokes best pre-tuned gain.
```
## Discussions
In summary, the effect of Proportional is the speed of reducing the CTE error. The larger the P value, the faster the controller to close the gap of error. However if it is set too large, the vehicle will start wobbling and especially in a curvy road, the CTE is dynamic, the wobbling becomes larger and vehicle would be gone out of control.

The Integral term is to zero the CTE error by multiplying a term to sum of error over time. Since it is a multipler to the total error over time, the I term is generally a lot smaller than P and D value. Experiment have been carried out that the vehicle is able to complete the track without an I term.

The D term is useful during the curve because it's a term acting proportinally to the change of CTE. During the curve of the during, the error CTE is changed to a larger degree. The D acts as a look-ahead agent and putting a corresponding actuation to that. This makes the vehicle can respond quick enough during the curve of the road.

The process of getting the optimal gain value began with setting 200 steps per iteration in twiddle mode and using initial kp,ki,kd of 0 and dp,di,dd of 1. The idea of limiting smaller steps per twiddle is to quickly proxy the operating region of kp,ki,kd because if the vehicle could drive the first 200 steps that covers some profile of the track, chances of it to complete whole track is higher and optimal gain term would not be far away. Also, the rate of ascent and descent of dp,di,dd is fixed at only 10% (1.1 or 0.9) and we would spend tonnes of cycles iterating on unhelpful gain term. (Maybe rate of ascent/descent can be implemented through something like deep learning momento learning rate? hmm...) 

With that, we settled at somewhere about kp=0.05, ki=0.002, kd=0.7. The final round of twiddle tuning was performed with 800 steps per iteration which covered a single lap, setting initial kp=0.05, ki=0.002, kd=0.7 and range of change to dd=0.05 di=0.002 ,dd=0.5. After a few hours, we obtained the final tuned value of **kp=0.0547, ki=0.0014, kd=0.7**.

The final thought is that the current PID controller is simplistic and probably works on tracks of similar profile, it is not only because of the subjective tuning but also the cost function does not consider disturbances of the vehicle where we have assumed ideal (means we neglect) in this exercise.


### **From this point onwards, the content is a duplicate of original readme from udacity repo.**
-----
# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

