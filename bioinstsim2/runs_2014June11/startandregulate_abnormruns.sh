#!/bin/bash

count=1

exename="bioinstsim2"
exenameontop="bioinstsim2"
maxnumjobs=4

simulationtime=15000



numberoffeatures=6 #3 6 9 12 15
numberofagents=20
threshold=1
tolhd=1


while [ "$count" -le "20" ]; do

    for Behav in AGGREGATION DISPERSION FLOCKING STOP
    do

      for ErrorBehav in STRLN RNDWK CIRCLE STOP
      do

        mkdir -p abnorm${Behav}/${ErrorBehav}/fv_${numberoffeatures}/result_t$count

	#gzip -d  abnorm${Behav}/${ErrorBehav}/fv_${numberoffeatures}/result_t$count/nohup.out.gz
	if [ -s abnorm${Behav}/${ErrorBehav}/fv_${numberoffeatures}/result_t$count/nohup.out ]
	then
		#gzip -c abnorm${Behav}/${ErrorBehav}/fv_${numberoffeatures}/result_t$count/nohup.out > abnorm${Behav}/${ErrorBehav}/fv_${numberoffeatures}/result_t$count/nohup.out.gz
		#rm abnorm${Behav}/${ErrorBehav}/fv_${numberoffeatures}/result_t$count/nohup.out
		ls
	else

		parallel --semaphore -j${maxnumjobs} ${exename} -a sizex=50,sizey=50,resx=10,resy=10,help -e name=TEST,swarmbehav=${Behav},errorbehav=${ErrorBehav},misbehavestep=0,tracknormalagent=1,trackabnormalagent=15,switchnormalbehav=0,durationofswitch=0,help -T maxspeed=0.1,count=${numberofagents},featuresenserange=6,selectnumnearestnbrs=10 -M numberoffeatures=${numberoffeatures},th=$threshold,tolhd=$tol,help -s ${count}${count}${count},help -n $simulationtime,help -z > abnorm${Behav}/${ErrorBehav}/fv_${numberoffeatures}/result_t$count/nohup.out 2> abnorm${Behav}/${ErrorBehav}/fv_${numberoffeatures}/result_t$count/nohup.time &


	fi

      done

    done

    count=`expr $count + 1`
done

# parallel --semaphore -j${maxnumjobs} $commandstring

sem --wait
