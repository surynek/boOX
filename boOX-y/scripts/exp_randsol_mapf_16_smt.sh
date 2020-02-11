ROBOT_LIST=`cat robots_16`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
    echo 'Solving random instance '$SIZE' with '$ROBOTS' agents ...'
   ../main/mapf_solver_boOX --algorithm=smtcbs+ --timeout=128 '--input-file=random_'$SIZE'_a'$ROBOTS'.mpf' '--output-file=mapf-smt_random_'$SIZE'_a'$ROBOTS'.out' > 'mapf-smt_random_'$SIZE'_a'$ROBOTS'.txt'
done
