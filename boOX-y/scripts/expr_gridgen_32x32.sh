ROBOT_LIST=`cat robots_32x32`
SEED_LIST=`cat seeds_10`
SIZE=32

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do    
    echo 'Generating grid '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
    ../main/gridgen_boOX --obstacle-probability=0.1 --walk '--x-size=32' '--y-size=32' '--N-agents='$ROBOTS '--seed='$SEED '--mpf-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.mpf' '--N-agents='$ROBOTS '--cpf-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'_'$SEED'.cpf'
  done
done
