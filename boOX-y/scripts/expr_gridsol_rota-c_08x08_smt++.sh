ROBOT_LIST=`cat robots_08x08_c`
SEED_LIST=`cat seeds_10`
SIZE=8
TIMEOUT=`cat timeout`

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do            
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
    ../main/rota_solver_boOX --capacitated --algorithm=smtcbs++ '--timeout='$TIMEOUT '--input-file=grid_'$SIZE'x'$SIZE'_c2_a'$ROBOTS'_'$SEED'.mpf' '--output-file=rota-smt++_grid_'$SIZE'x'$SIZE'_c2_a'$ROBOTS'_'$SEED'.out' > 'rota-smt++_grid_'$SIZE'x'$SIZE'_c2_a'$ROBOTS'_'$SEED'.txt'
  done
done

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do            
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
    ../main/rota_solver_boOX --capacitated --algorithm=smtcbs++ '--timeout='$TIMEOUT '--input-file=grid_'$SIZE'x'$SIZE'_c3_a'$ROBOTS'_'$SEED'.mpf' '--output-file=rota-smt++_grid_'$SIZE'x'$SIZE'_c3_a'$ROBOTS'_'$SEED'.out' > 'rota-smt++_grid_'$SIZE'x'$SIZE'_c3_a'$ROBOTS'_'$SEED'.txt'
  done
done

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do            
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
    ../main/rota_solver_boOX --capacitated --algorithm=smtcbs++ '--timeout='$TIMEOUT '--input-file=grid_'$SIZE'x'$SIZE'_c4_a'$ROBOTS'_'$SEED'.mpf' '--output-file=rota-smt++_grid_'$SIZE'x'$SIZE'_c4_a'$ROBOTS'_'$SEED'.out' > 'rota-smt++_grid_'$SIZE'x'$SIZE'_c4_a'$ROBOTS'_'$SEED'.txt'
  done
done


