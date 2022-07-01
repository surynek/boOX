ROBOT_LIST=`cat robots_16x16`
SIZE=16
TIMEOUT=`cat timeout`

for ROBOTS in $ROBOT_LIST;
do
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
  ../main/insolver_reLOC '--total-timeout='$TIMEOUT '--minisat-timeout=64 --encoding=mdd '--input-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.cpf' '--output-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.out' > 'mapf-mdd_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.txt'
done
