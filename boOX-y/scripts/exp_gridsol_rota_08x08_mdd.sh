ROBOT_LIST=`cat robots_08x08`
SIZE=8

for ROBOTS in $ROBOT_LIST;
do
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
  ../main/insolver_reLOC --total-timeout=64 --minisat-timeout=64 --encoding=pmdd '--input-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.cpf' '--output-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.out' > 'rota-mdd_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.txt'
done
