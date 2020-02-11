ROBOT_LIST=`cat robots_16x16_c`
SEED_LIST=`cat seeds_10`
SIZE=16
TIMEOUT=`cat timeout`

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do        
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
    ../main/insolver_reLOC '--total-timeout='$TIMEOUT '--minisat-timeout='$TIMEOUT --encoding=pcmdd '--input-file=grid_'$SIZE'x'$SIZE'_c2_a'$ROBOTS'_'$SEED'.cpf' '--output-file=rota-mdd++_grid_'$SIZE'x'$SIZE'_c2_a'$ROBOTS'_'$SEED'.out' > 'rota-mdd++_grid_'$SIZE'x'$SIZE'_c2_a'$ROBOTS'_'$SEED'.txt'
  done
done

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do        
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
    ../main/insolver_reLOC '--total-timeout='$TIMEOUT '--minisat-timeout='$TIMEOUT --encoding=pcmdd '--input-file=grid_'$SIZE'x'$SIZE'_c3_a'$ROBOTS'_'$SEED'.cpf' '--output-file=rota-mdd++_grid_'$SIZE'x'$SIZE'_c3_a'$ROBOTS'_'$SEED'.out' > 'rota-mdd++_grid_'$SIZE'x'$SIZE'_c3_a'$ROBOTS'_'$SEED'.txt'
  done
done

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do        
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
    ../main/insolver_reLOC '--total-timeout='$TIMEOUT '--minisat-timeout='$TIMEOUT --encoding=pcmdd '--input-file=grid_'$SIZE'x'$SIZE'_c4_a'$ROBOTS'_'$SEED'.cpf' '--output-file=rota-mdd++_grid_'$SIZE'x'$SIZE'_c4_a'$ROBOTS'_'$SEED'.out' > 'rota-mdd++_grid_'$SIZE'x'$SIZE'_c4_a'$ROBOTS'_'$SEED'.txt'
  done
done
