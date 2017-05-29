# emc_simulator
Let's define a variable to make these commands easier. Assuming you are inside your main_repo: (~/emc/main_repo for me) 
```
export MAIN_REPO=$PWD
```
Create folders in your root GitLab directory
```
mkdir emc
cd emc
```
Clone visualizer
```
git clone git@github.com:Karel-van-de-Plassche/emc_simulator.git
```
Build visualizer
```
cd $MAIN_REPO/build
make
```
Copy visualizer to weird EMC directory..
```
cp $MAIN_REPO/bin/pico_simulator2 $HOME/.emc/system/src/emc_simulator/
```
And go! 
```
rosrun emc_simulator pico_simulator2
```
