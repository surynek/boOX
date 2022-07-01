ROBOT_LIST=`cat robots_08x08`
SEED_LIST=`cat seeds_10`
SIZE=8

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do        
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
  ../main/mapf_solver_boOX --algorithm=smtcbs --timeout=64 '--input-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.mpf' '--output-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.out' > 'mapf-smt_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.txt'
  done
done
