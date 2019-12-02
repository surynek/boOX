SIZE=32
NEIGHBOR=3

TIMEOUT=`cat timeout`
KRUHOBOTS_LIST=`cat 'kruhobots_'$SIZE'x'$SIZE`
SEEDS_LIST=`cat seeds`
ALGORITHM=cbsR++

for KRUHOBOTS in $KRUHOBOTS_LIST;
do
    for SEED in $SEEDS_LIST;
    do
        echo 'Solving random ['$SEED'] grid MAPF-R instance '$SIZE'x'$SIZE' with '$KRUHOBOTS' kruhobots ...'    
        ../../main/mapfR_solver_boOX '--timeout='$TIMEOUT '--input-mapR-file=grid_'$SIZE'x'$SIZE'_n'$NEIGHBOR'.mapR' '--input-kruhoR-file=grid_'$SIZE'x'$SIZE'_k'$KRUHOBOTS'_'$SEED'.kruR' --output-file=solution.txt '--algorithm='$ALGORITHM > 'grid_'$SIZE'x'$SIZE'_n'$NEIGHBOR'_k'$KRUHOBOTS'_'$ALGORITHM'_'$SEED'.txt'
    done
done