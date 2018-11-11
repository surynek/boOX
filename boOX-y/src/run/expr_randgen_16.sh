ROBOT_LIST=`cat robots_16`
SEED_LIST=`cat seeds_10`
SIZE=16

for ROBOTS in $ROBOT_LIST;
  do	      	      
  for SEED in $SEED_LIST;	
    do
      echo 'Generating random '$SIZE' with '$ROBOTS' agents ...'
      ../main/randgen_boOX '--walk' '--edge-probability=0.2' '--N-vertices='$SIZE '--seed='$SEED '--N-agents='$ROBOTS '--mpf-file=rand_'$SIZE'_a'$ROBOTS'_'$SEED'.mpf' '--cpf-file=rand_'$SIZE'_a'$ROBOTS'_'$SEED'.cpf'
  done
done
