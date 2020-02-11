ROBOT_LIST=`cat robots_16x16`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
  echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'    
  ../main/mapf_solver_boOX '--input-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.mpf' '--output-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.out' > 'grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.txt'
done
