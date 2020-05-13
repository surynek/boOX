PREFIX="road2-big"

KRUHOBOTS_LIST=`cat kruhobots_road2-big`
SCENARIOS_LIST=`cat scenarios_road2-big`

    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Generating '$PREFIX' scenario '$SCENARIO' MAPF-R instances'
	./expr_road2-big_kruR-gen-scen.sh $SCENARIO &
    done
