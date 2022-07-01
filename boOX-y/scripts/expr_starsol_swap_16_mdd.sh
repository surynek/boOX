ROBOT_LIST=`cat robots_16`
SEED_LIST=`cat seeds_10`
SIZE=16
TIMEOUT=`cat timeout`

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do            
    echo 'Solving star instance '$SIZE' with '$ROBOTS' agents ...'
    ../main/insolver_reLOC '--total-timeout='$TIMEOUT '--minisat-timeout='$TIMEOUT --encoding=temdd '--input-file=star_'$SIZE'_a'$ROBOTS'_'$SEED'.cpf' '--output-file=swap-mdd_star_'$SIZE'_a'$ROBOTS'_'$SEED'.out' > 'swap-mdd_star_'$SIZE'_a'$ROBOTS'_'$SEED'.txt'
  done
done
