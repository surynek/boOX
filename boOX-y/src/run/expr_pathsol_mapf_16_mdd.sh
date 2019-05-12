ROBOT_LIST=`cat robots_16`
SEED_LIST=`cat seeds_10`
SIZE=16
TIMEOUT=`cat timeout`

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do            
    echo 'Solving path instance '$SIZE' with '$ROBOTS' agents ...'
    ../main/insolver_reLOC '--total-timeout='$TIMEOUT '--minisat-timeout='$TIMEOUT --encoding=mdd '--input-file=path_'$SIZE'_a'$ROBOTS'_'$SEED'.cpf' '--output-file=mapf-mdd_path_'$SIZE'_a'$ROBOTS'_'$SEED'.out' > 'mapf-mdd_path_'$SIZE'_a'$ROBOTS'_'$SEED'.txt'
  done
done
