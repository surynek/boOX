ROBOT_LIST=`cat robots_maps`
SEED_LIST=`cat seeds_10`
SIZE=16
TIMEOUT=`cat timeout`

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do            
    echo 'Solving clique instance '$SIZE' with '$ROBOTS' agents ...'
    ../main/perm_solver_boOX --algorithm=cbs++ '--timeout='$TIMEOUT '--input-file=ost003d_a'$ROBOTS'_'$SEED'.mpf' '--output-file=perm-cbs_ost003d_a'$ROBOTS'_'$SEED'.out' > 'perm-cbs_ost003d_a'$ROBOTS'_'$SEED'.txt'
  done
done
