ROBOT_LIST=`cat robots_64x64`
SEED_LIST=`cat seeds_10`
SIZE=64

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do        
    echo $ROBOTS,$SEED
    grep "machine TIME" 'swap-smt++_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.txt'
    grep "clauses" 'swap-smt++_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.txt'
  done
done
