ROBOT_LIST=`cat robots_maps_c`
SEED_LIST=`cat seeds_10`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do                
    echo 'Generating map den520 with '$ROBOTS' agents ...'
   ../main/gridgen_boOX --capacity=2 --map-file=../../maps/den520d.map '--N-agents='$ROBOTS '--mpf-file=den520d_c2_a'$ROBOTS'_'$SEED'.mpf' '--cpf-file=den520d_c2_a'$ROBOTS'_'$SEED'.cpf'
  done
done

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do                
    echo 'Generating map den520 with '$ROBOTS' agents ...'
   ../main/gridgen_boOX --capacity=3 --map-file=../../maps/den520d.map '--N-agents='$ROBOTS '--mpf-file=den520d_c3_a'$ROBOTS'_'$SEED'.mpf' '--cpf-file=den520d_c3_a'$ROBOTS'_'$SEED'.cpf'
  done
done

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do                
    echo 'Generating map den520 with '$ROBOTS' agents ...'
   ../main/gridgen_boOX --capacity=4 --map-file=../../maps/den520d.map '--N-agents='$ROBOTS '--mpf-file=den520d_c4_a'$ROBOTS'_'$SEED'.mpf' '--cpf-file=den520d_c4_a'$ROBOTS'_'$SEED'.cpf'
  done
done
