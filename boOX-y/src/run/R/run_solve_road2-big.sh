SCENARIOS_LIST=`cat scenarios_road2-big`

for SCENARIO in $SCENARIOS_LIST;
do
    ./expr_road2-big_solve.sh $SCENARIO &
done
