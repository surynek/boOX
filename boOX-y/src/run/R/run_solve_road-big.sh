SCENARIOS_LIST=`cat scenarios_road-big`

for SCENARIO in $SCENARIOS_LIST;
do
    ./expr_road-big_solve.sh $SCENARIO &
done
