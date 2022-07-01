ROBOT_LIST=`cat robots_16x16`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
  ../main/insolver_reLOC --total-timeout=64 --minisat-timeout=64 --encoding=temdd '--input-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.cpf' '--output-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.out' > 'swap-mdd_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.txt'
done
