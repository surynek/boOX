ROBOT_LIST=`cat robots_16x16`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
    ./perm_solver_boOX --algorithm=cbs++ --timeout=512 '--input-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.mpf' '--output-file=perm-cbs_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.out' > 'perm-cbs_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.txt'
done
