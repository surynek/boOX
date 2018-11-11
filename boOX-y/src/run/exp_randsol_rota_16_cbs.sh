ROBOT_LIST=`cat robots_16`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
    echo 'Solving random instance '$SIZE' with '$ROBOTS' agents ...'
    ../main/rota_solver_boOX --algorithm=cbs++ --timeout=128 '--input-file=rand_'$SIZE'_a'$ROBOTS'.mpf' '--output-file=rota-cbs_rand_'$SIZE'_a'$ROBOTS'.out' > 'rota-cbs_rand_'$SIZE'_a'$ROBOTS'.txt'
done
