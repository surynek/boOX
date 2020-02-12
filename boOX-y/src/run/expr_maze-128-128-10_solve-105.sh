SCENARIO=$1

TIMEOUT=`cat timeout`
PREFIX="maze-128-128-10"
SCENARIO_PREFIX=$PREFIX"-random"

ROBOTS_FILE='robots_'$PREFIX
ROBOTS_LIST=`cat $ROBOTS_FILE`

for ROBOTS in $ROBOTS_LIST;
do
    echo 'Solving '$PREFIX' scenario '$SCENARIO' MAPF instance with '$ROBOTS' agents ...'
    ../main/rota_solver_boOX '--subopt-ratio=1.05' '--timeout='$TIMEOUT  '--input-file='$PREFIX'-'$SCENARIO'_a'$ROBOTS'.mpf' '--algorithm=smtcbs++' '--output-file=solution-105_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'.txt' > 'out-105_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'.txt'
done
