ROBOT_LIST=`cat robots_16`
SIZE_LIST=`cat sizes_16`

for SIZE in $SIZE_LIST;
do
    ROBOTS=$SIZE
    echo 'Solving path instance '$SIZE' with '$ROBOTS' agents ...'
    ../main/insolver_reLOC --total-timeout=64 --minisat-timeout=64 --encoding=temdd '--input-file=path_'$SIZE'_a'$ROBOTS'.cpf' '--output-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.out' > 'swap-mdd_path_'$SIZE'_a'$ROBOTS'.txt'
done
