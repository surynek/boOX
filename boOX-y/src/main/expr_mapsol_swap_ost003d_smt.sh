ROBOT_LIST=`cat robots_maps`
SEED_LIST=`cat seeds_10`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do            
    echo 'Solving ost003d instance with '$ROBOTS' agents ...'
    ./swap_solver_boOX --algorithm=smtcbs --timeout=64 '--input-file=ost003d_a'$ROBOTS'_'$SEED'.mpf' '--output-file=ost003d_a'$ROBOTS'_'$SEED'.out' > 'swap-smt_ost003d_a'$ROBOTS'_'$SEED'.txt'
  done
done
