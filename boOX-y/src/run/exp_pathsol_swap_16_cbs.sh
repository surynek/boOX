ROBOT_LIST=`cat robots_16`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
    echo 'Solving path instance '$SIZE' with '$ROBOTS' agents ...'
    ../main/swap_solver_boOX --timeout=128 --algorithm=cbs++ '--input-file=path_'$SIZE'_a'$ROBOTS'.mpf' '--output-file=swap-cbs_path_'$SIZE'_a'$ROBOTS'.out' > 'swap-cbs_path_'$SIZE'_a'$ROBOTS'.txt'
done
