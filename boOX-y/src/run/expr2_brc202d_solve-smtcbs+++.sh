SCENARIO=$1

TIMEOUT=`cat timeout`
PREFIX="brc202d"
SCENARIO_PREFIX=$PREFIX"-random"

ROBOTS_FILE='robots_'$PREFIX
ROBOTS_LIST=`cat $ROBOTS_FILE`

for ROBOTS in $ROBOTS_LIST;
do
    echo 'Solving '$PREFIX' scenario '$SCENARIO' MAPF instance with '$ROBOTS' agents ...'
    ../main/rota_solver_boOX '--timeout='$TIMEOUT  '--input-file='$PREFIX'-'$SCENARIO'_a'$ROBOTS'.mpf' '--algorithm=smtcbs+++' '--output-file=solution_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'.smtcbs+++.txt' > 'out_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'.smtcbs+++.txt'
done
