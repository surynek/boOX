ROBOT_LIST=`cat robots_16`
SEED_LIST=`cat seeds_10`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do            
    echo 'Solving clique instance '$SIZE' with '$ROBOTS' agents ...'
    ./perm_solver_boOX --algorithm=cbs++ --timeout=512 '--input-file=clique_'$SIZE'_a'$ROBOTS'_'$SEED'.mpf' '--output-file=perm-cbs_clique_'$SIZE'_a'$ROBOTS'_'$SEED'.out' > 'perm-cbs_clique_'$SIZE'_a'$ROBOTS'_'$SEED'.txt'
  done
done
