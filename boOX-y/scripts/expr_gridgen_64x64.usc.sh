ROBOT_LIST=`cat robots_64x64`
SEED_LIST=`cat seeds_10`
SIZE=64

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do    
    echo 'Generating grid '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
    ../main/gridgen_boOX --obstacle-probability=0.1 --walk '--x-size=64' '--y-size=64' '--N-agents='$ROBOTS '--seed='$SEED '--mpf-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.mpf' '--N-agents='$ROBOTS '--cpf-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.cpf'
  done
done
