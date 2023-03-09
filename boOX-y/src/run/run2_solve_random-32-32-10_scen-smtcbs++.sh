PREFIX='random-32-32-10'

SCENARIOS_FILE="scenarios_"$PREFIX
SCENARIOS_LIST=`cat $SCENARIOS_FILE`

for SCENARIO in $SCENARIOS_LIST;
do
    './expr2_'$PREFIX'_solve-smtcbs++.sh' $SCENARIO &
done
