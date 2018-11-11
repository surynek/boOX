ROBOT_LIST=`cat robots_16`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
    echo 'Solving path instance '$SIZE' with '$ROBOTS' agents ...'
    ./perm_solver_boOX --timeout=1024 --algorithm=cbs++ '--input-file=path_'$SIZE'_a'$ROBOTS'.mpf' '--output-file=perm-cbs_path_'$SIZE'_a'$ROBOTS'.out' > 'perm-cbs_path_'$SIZE'_a'$ROBOTS'.txt'
done
