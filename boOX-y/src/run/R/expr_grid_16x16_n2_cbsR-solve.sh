SIZE=16
NEIGHBOR=2

TIMEOUT=`cat timeout`
KRUHOBOTS_LIST=`cat 'kruhobots_'$SIZE'x'$SIZE`
ALGORITHM=cbsR

for KRUHOBOTS in $KRUHOBOTS_LIST;
do
    echo 'Solving grid MAPF-R instance '$SIZE'x'$SIZE' with '$KRUHOBOTS' kruhobots ...'    
    ../../main/mapfR_solver_boOX '--timeout='$TIMEOUT '--input-mapR-file=grid_'$SIZE'x'$SIZE'_n'$NEIGHBOR'.mapR' '--input-kruhoR-file=grid_'$SIZE'x'$SIZE'_k'$KRUHOBOTS'.kruR' --output-file=solution.txt '--algorithm='$ALGORITHM > 'grid_'$SIZE'x'$SIZE'_n'$NEIGHBOR'_k'$KRUHOBOTS'_'$ALGORITHM'.txt'
done
