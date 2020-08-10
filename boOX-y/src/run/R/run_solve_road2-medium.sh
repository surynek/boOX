SCENARIOS_LIST=`cat scenarios_road2-medium`

for SCENARIO in $SCENARIOS_LIST;
do
    ./expr_road2-medium_solve.sh $SCENARIO &
done
