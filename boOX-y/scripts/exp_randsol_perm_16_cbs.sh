ROBOT_LIST=`cat robots_16`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
    echo 'Solving random instance '$SIZE' with '$ROBOTS' agents ...'
    ../main/perm_solver_boOX --timeout=128 --algorithm=cbs++ '--input-file=random_'$SIZE'_a'$ROBOTS'.mpf' '--output-file=perm-cbs_random_'$SIZE'_a'$ROBOTS'.out' > 'perm-cbs_random_'$SIZE'_a'$ROBOTS'.txt'
done
