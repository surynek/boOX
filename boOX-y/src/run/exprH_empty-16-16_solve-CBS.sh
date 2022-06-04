SCENARIO=$1

TIMEOUT=`cat timeout`
PREFIX="empty-16-16"
SCENARIO_PREFIX=$PREFIX"-random"

TASK_FILE='task_'$PREFIX
ROBOTS_FILE='robotsH_'$PREFIX
ROBOTS_LIST=`cat $ROBOTS_FILE`
TASK=`cat $TASK_FILE`

for ROBOTS in $ROBOTS_LIST;
do
    echo 'Solving '$PREFIX' scenario '$SCENARIO' MAPF instance with '$ROBOTS' agents ...'
    ../main/hamilton_solver_boOX '--timeout='$TIMEOUT  '--input-file='$PREFIX'-'$SCENARIO'_a'$ROBOTS'-t'$TASK'.mHpf' '--algorithm=cbs#+' '--output-file=solution-CBS_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'-t'$TASK'.txt' > 'out-CBS_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'-t'$TASK'.txt'
done
