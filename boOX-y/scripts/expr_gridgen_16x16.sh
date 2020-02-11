ROBOT_LIST=`cat robots_16x16`
SEED_LIST=`cat seeds_10`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do    
    echo 'Generating grid '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
    ../main/gridgen_boOX --walk '--x-size=16' '--y-size=16' '--N-agents='$ROBOTS '--seed='$SEED '--mpf-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.mpf' '--N-agents='$ROBOTS '--cpf-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.cpf'
  done
done
