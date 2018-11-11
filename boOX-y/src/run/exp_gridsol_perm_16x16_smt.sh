ROBOT_LIST=`cat robots_16x16`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
   ./perm_solver_boOX --algorithm=smtcbs+ --timeout=512 '--input-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.mpf' '--output-file=perm-smt_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.out' > 'perm-smt_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.txt'
done
