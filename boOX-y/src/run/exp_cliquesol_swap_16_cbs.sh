ROBOT_LIST=`cat robots_16`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
    echo 'Solving clique instance '$SIZE' with '$ROBOTS' agents ...'
    ../main/swap_solver_boOX --timeout=64 --algorithm=cbs+ '--input-file=clique_'$SIZE'_a'$ROBOTS'.mpf' '--output-file=clique_'$SIZE'_a'$ROBOTS'.out' > 'swap-cbs_clique_'$SIZE'_a'$ROBOTS'.txt'
done
