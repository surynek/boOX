SIZE=8
NEIGHBOR=2

KRUHOBOTS_LIST=`cat 'kruhobots_'$SIZE'x'$SIZE`

for KRUHOBOTS in $KRUHOBOTS_LIST;
do
    echo 'Generating grid MAPF-R instance '$SIZE'x'$SIZE' with '$KRUHOBOTS' kruhobots ...'    
    ../../main/kruhoR_generate_boOX '--input-mapR-file=grid_'$SIZE'x'$SIZE'_n'$NEIGHBOR'.mapR' '--output-kruhoR-file=grid_'$SIZE'x'$SIZE'_k'$KRUHOBOTS'.kruR' '--N-kruhobots='$KRUHOBOTS
done
