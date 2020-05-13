ROBOT_LIST=`cat robots_08x08`
SIZE=8
TIMEOUT=`cat timeout`

for ROBOTS in $ROBOT_LIST;
do
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
  ../main/mapf_solver_boOX --algorithm=cbs '--timeout='$TIMEOUT '--input-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.mpf' '--output-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.out' > 'mapf-cbs_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.txt'
done
