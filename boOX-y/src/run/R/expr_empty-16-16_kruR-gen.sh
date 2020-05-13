PREFIX="empty-16-16-random"

KRUHOBOTS_LIST=`cat kruhobots_empty`
SCENARIOS_LIST=`cat scenarios_empty`

for KRUHOBOTS in $KRUHOBOTS_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Generating '$PREFIX' scenario '$SCENARIO' MAPF-R instance with '$KRUHOBOTS' kruhobots ...'
	../../main/kruhoR_generate_boOX '--kruho-radius=0.3535533905933' '--input-xml-map-file=map_'$PREFIX'.xml' '--input-xml-agent-file='$PREFIX'-'$SCENARIO'_k'$KRUHOBOTS'.xml' '--output-kruhoR-file='$PREFIX'-'$SCENARIO'_k'$KRUHOBOTS'.kruR' '--N-kruhobots='$KRUHOBOTS
    done
done
