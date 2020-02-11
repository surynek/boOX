SIZE=16
AGENTS_LIST=`cat 'agents_'$SIZE'x'$SIZE`

for AGENTS in $AGENTS_LIST;
  do
    ../main/gridgen_boOX '--x-size='$SIZE '--y-size='$SIZE '--mpf-file=grid_'$SIZE'x'$SIZE'_a'$AGENTS'.mpf' '--N-agents='$AGENTS
    #../main/gridgen_boOX '--x-size='$SIZE '--y-size='$SIZE '--usc-map-file=grid_'$SIZE'x'$SIZE'.usc.map' '--usc-agents-file=grid_'$SIZE'x'$SIZE'_a'$AGENTS'.usc.agent' '--N-agents='$AGENTS
done
