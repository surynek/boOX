AGENTS_LIST=`cat agents_maps`
SEEDS=`cat seeds`
MAP=brc202d

for AGENTS in $AGENTS_LIST;
  do
  for SEED in $SEEDS;
    do
      ./rota_solver_boOX --algorithm=smtcbs+ '--input-file=map_'$MAP'_a'$AGENTS'_'$SEED'.mpf' '--output-file=solution.txt' '--timeout=1024'
  done
done
