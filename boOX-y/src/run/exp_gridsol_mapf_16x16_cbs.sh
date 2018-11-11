ROBOT_LIST=`cat robots_16x16`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
    ./mapf_solver_boOX --algorithm=cbs++ --timeout=512 '--input-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.mpf' '--output-file=mapf-cbs_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.out' > 'mapf-cbs_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.txt'
done
