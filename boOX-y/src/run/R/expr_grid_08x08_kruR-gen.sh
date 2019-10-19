SIZE=8
NEIGHBOR=2

KRUHOBOTS_LIST=`cat 'kruhobots_'$SIZE'x'$SIZE`
SEEDS_LIST=`cat seeds`

for KRUHOBOTS in $KRUHOBOTS_LIST;
do
    for SEED in $SEEDS_LIST;
    do
	echo 'Generating random ['$SEED'] grid MAPF-R instance '$SIZE'x'$SIZE' with '$KRUHOBOTS' kruhobots ...'
	../../main/kruhoR_generate_boOX '--kruho-radius=0.3535533905933' '--seed='$SEED '--input-mapR-file=grid_'$SIZE'x'$SIZE'_n'$NEIGHBOR'.mapR' '--output-kruhoR-file=grid_'$SIZE'x'$SIZE'_k'$KRUHOBOTS'_'$SEED'.kruR' '--N-kruhobots='$KRUHOBOTS
    done
done
