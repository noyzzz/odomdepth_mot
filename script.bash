#get an argument from the command line for is_sim is 1 or 0
is_sim=$1
source ../../../devel/setup.bash
source ~/cvbridge_build_ws/install/setup.bash --extend
#if is_sim is 1 then run rosrun my_tracker helper_f.py --is_sim else run rosrun my_tracker helper_f.py
if [ $is_sim -eq 1 ]
then
    rosrun my_tracker helper_f.py --is_sim
else
    rosrun my_tracker helper_f.py
fi