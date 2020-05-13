ROBOT_LIST=`cat robots_32x32`
SEED_LIST=`cat seeds_10`
SIZE=32
TIMEOUT=`cat timeout`

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do            
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
   ../main/mapf_solver_boOX --algorithm=smtcbs++ '--timeout='$TIMEOUT '--input-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.mpf' '--output-file=mapf-smt++_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.out' > 'mapf-smt++_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.txt'
  done
done
