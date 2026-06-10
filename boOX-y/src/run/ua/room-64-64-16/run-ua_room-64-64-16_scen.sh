PREFIX='room-64-64-16'

SCENARIOS_FILE="scenarios_"$PREFIX
SCENARIOS_LIST=`cat $SCENARIOS_FILE`

for SCENARIO in $SCENARIOS_LIST;
do
    './expr-ua_'$PREFIX'_solve.sh' $SCENARIO &
done
