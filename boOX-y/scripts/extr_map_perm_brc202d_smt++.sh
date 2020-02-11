ROBOT_LIST=`cat robots_maps`
SEED_LIST=`cat seeds_10`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do            
    echo $ROBOTS,$SEED
     grep "machine TIME" 'perm-smt++_brc202d_a'$ROBOTS'_'$SEED'.txt'
     grep "clauses" 'perm-smt++_brc202d_a'$ROBOTS'_'$SEED'.txt'
  done
done
