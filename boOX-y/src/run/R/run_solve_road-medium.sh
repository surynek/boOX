SCENARIOS_LIST=`cat scenarios_road-medium`

for SCENARIO in $SCENARIOS_LIST;
do
    ./expr_road-medium_solve.sh $SCENARIO &
done
