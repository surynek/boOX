ROBOT_LIST=`cat robots_16`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
    echo 'Solving clique instance '$SIZE' with '$ROBOTS' agents ...'
    ../main/insolver_reLOC --total-timeout=64 --minisat-timeout=64 --encoding=temdd '--input-file=clique_'$SIZE'_a'$ROBOTS'.cpf' '--output-file=clique_'$SIZE'_a'$ROBOTS'.out' > 'swap-mdd_clique_'$SIZE'_a'$ROBOTS'.txt'
done
