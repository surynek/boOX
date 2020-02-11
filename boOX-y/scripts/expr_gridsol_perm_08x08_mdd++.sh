ROBOT_LIST=`cat robots_08x08`
SEED_LIST=`cat seeds_10`
SIZE=8
TIMEOUT=`cat timeout`

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do        
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
    ../main/insolver_reLOC '--total-timeout='$TIMEOUT '--minisat-timeout='$TIMEOUT --encoding=rmdd '--input-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.cpf' '--output-file=perm-mdd++_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.out' > 'perm-mdd++_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.txt'
  done
done
