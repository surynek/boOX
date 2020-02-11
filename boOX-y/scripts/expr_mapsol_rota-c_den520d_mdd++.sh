ROBOT_LIST=`cat robots_maps_c`
SEED_LIST=`cat seeds_10`
SIZE=16
TIMEOUT=`cat timeout`

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do            
    echo 'Solving den520d instance with '$ROBOTS' agents ...'
    ../main/insolver_reLOC '--total-timeout='$TIMEOUT '--minisat-timeout='$TIMEOUT --encoding=pcmdd '--input-file=den520d_c2_a'$ROBOTS'_'$SEED'.cpf' '--output-file=rota-mdd++_den520d_c2_a'$ROBOTS'_'$SEED'.out' > 'rota-mdd++_den520d_c2_a'$ROBOTS'_'$SEED'.txt'
  done
done

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do            
    echo 'Solving den520d instance with '$ROBOTS' agents ...'
    ../main/insolver_reLOC '--total-timeout='$TIMEOUT '--minisat-timeout='$TIMEOUT --encoding=pcmdd '--input-file=den520d_c3_a'$ROBOTS'_'$SEED'.cpf' '--output-file=rota-mdd++_den520d_c3_a'$ROBOTS'_'$SEED'.out' > 'rota-mdd++_den520d_c3_a'$ROBOTS'_'$SEED'.txt'
  done
done

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do            
    echo 'Solving den520d instance with '$ROBOTS' agents ...'
    ../main/insolver_reLOC '--total-timeout='$TIMEOUT '--minisat-timeout='$TIMEOUT --encoding=pcmdd '--input-file=den520d_c4_a'$ROBOTS'_'$SEED'.cpf' '--output-file=rota-mdd++_den520d_c4_a'$ROBOTS'_'$SEED'.out' > 'rota-mdd++_den520d_c4_a'$ROBOTS'_'$SEED'.txt'
  done
done
