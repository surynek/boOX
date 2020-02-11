AGENTS_LIST=`cat agents_maps`
MAP=den520d

for AGENTS in $AGENTS_LIST;
do
    ./gridgen_boOX '--map-file=../../maps/'$MAP'.map' '--mpf-file=map_'$MAP'_a'$AGENTS'.mpf' '--N-agents='$AGENTS
    ./gridgen_boOX '--map-file=../../maps/'$MAP'.map' '--usc-map-file=map_'$MAP'.usc.map' '--usc-agents-file=map_'$MAP'_a'$AGENTS'.usc.agent' '--N-agents='$AGENTS
done
