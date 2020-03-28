ROBOT_LIST=`cat robots_16x16_c`
SEED_LIST=`cat seeds_10`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do        
    echo $ROBOTS,$SEED
    grep "machine TIME" 'rota-smt++_grid_'$SIZE'x'$SIZE'_c2_a'$ROBOTS'_'$SEED'.txt'
    grep "clauses" 'rota-smt++_grid_'$SIZE'x'$SIZE'_c2_a'$ROBOTS'_'$SEED'.txt'
  done
done

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do        
    echo $ROBOTS,$SEED
    grep "machine TIME" 'rota-smt++_grid_'$SIZE'x'$SIZE'_c3_a'$ROBOTS'_'$SEED'.txt'
    grep "clauses" 'rota-smt++_grid_'$SIZE'x'$SIZE'_c3_a'$ROBOTS'_'$SEED'.txt'
  done
done

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do        
    echo $ROBOTS,$SEED
    grep "machine TIME" 'rota-smt++_grid_'$SIZE'x'$SIZE'_c4_a'$ROBOTS'_'$SEED'.txt'
    grep "clauses" 'rota-smt++_grid_'$SIZE'x'$SIZE'_c4_a'$ROBOTS'_'$SEED'.txt'
  done
done

