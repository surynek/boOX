ROBOT_LIST=`cat robots_08x08_c`
SEED_LIST=`cat seeds_10`
SIZE=8

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do    
    echo 'Generating grid '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
    ../main/gridgen_boOX --capacity=2 --walk '--x-size=8' '--y-size=8' '--N-agents='$ROBOTS '--seed='$SEED '--mpf-file=grid_'$SIZE'x'$SIZE'_c2_a'$ROBOTS'_'$SEED'.mpf' '--N-agents='$ROBOTS '--cpf-file=grid_'$SIZE'x'$SIZE'_c2_a'$ROBOTS'_'$SEED'.cpf'
  done
done

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do    
    echo 'Generating grid '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
    ../main/gridgen_boOX --capacity=3 --walk '--x-size=8' '--y-size=8' '--N-agents='$ROBOTS '--seed='$SEED '--mpf-file=grid_'$SIZE'x'$SIZE'_c3_a'$ROBOTS'_'$SEED'.mpf' '--N-agents='$ROBOTS '--cpf-file=grid_'$SIZE'x'$SIZE'_c3_a'$ROBOTS'_'$SEED'.cpf'
  done
done

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do    
    echo 'Generating grid '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
    ../main/gridgen_boOX --capacity=4 --walk '--x-size=8' '--y-size=8' '--N-agents='$ROBOTS '--seed='$SEED '--mpf-file=grid_'$SIZE'x'$SIZE'_c4_a'$ROBOTS'_'$SEED'.mpf' '--N-agents='$ROBOTS '--cpf-file=grid_'$SIZE'x'$SIZE'_c4_a'$ROBOTS'_'$SEED'.cpf'
  done
done
