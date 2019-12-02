ROBOT_LIST=`cat robots_16x16`
SIZE=16
TIMEOUT=`cat timeout`

for ROBOTS in $ROBOT_LIST;
do
    echo 'Solving grid instance '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
   ../main/rota_solver_boOX --algorithm=smtcbs+ '--timeout='$TIMEOUT '--input-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.mpf' '--output-file=rota-smt_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.out' > 'rota-smt_grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.txt'
done
