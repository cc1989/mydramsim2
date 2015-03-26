#!/bin/bash

benchMark=("gobmk" "lbm" "sjeng" "libquantum" "bzip2" "mcf" "povray" "gcc" "soplex" "omnetpp" "xalancbmk" "namd" "milc" "dealII" "perlbench" "hmmer")
#workLoad=("hmmer;perlbench;dealII;namd;milc;omnetpp;soplex;xalancbmk" "hmmer;perlbench;dealII;namd;milc;omnetpp;libquantum;bzip2" "hmmer;milc;dealII;namd;libquantum;bzip2;libquantum2;bzip22" "hmmer;namd;gobmk;mcf;bzip2;libquantum;bzip22;libquantum2")
#workLoad=("dealII;milc;libquantum;bzip2")
workLoad=("dealII;namd;libquantum;lbm")
maxIns=300000000
cpuCount=4
date=`date -d today +"%Y-%m-%d:%H:%M:%S"`
declare -A aloneWrokTime 

statisticsMPKI()
{
	for name in ${benchMark[@]};
	do
		./build/X86/gem5.opt -d ./m5out/${date}/${name} configs/example/se.py -c $name --caches --l1d_size=64kB --l1i_size=64kB --l2cache --l2_size=8MB  -I $maxIns --mem-type=SimpleMemory --cpu-type=DerivO3CPU &
	done
}

resultMPKI()
{
	for name in  ${benchMark[@]};
	do
		for file in m5out/$1/${name}/*
		do
			#echo $file
			if [ "`basename $file`" == "stats.txt" ] && [ -s $file ] 
			then
				totalIns=`grep sim_insts $file | awk  '{print $2}'`
				readIns=`grep system.mem_ctrls.num_reads::total $file | awk  '{print $2}'`
				if [ "$readIns" == "" ]
				then
					readIns=0
				fi
				writeIns=`grep system.mem_ctrls.num_writes::total $file | awk  '{print $2}'`
				if [ "$writeIns" == "" ]
				then
					writeIns=0
				fi
				writeM=`echo $writeIns"/ ("$readIns"+"$writeIns")" | bc -l` 
				MPKI=`echo  "("$readIns"+"$writeIns")*1000/"$totalIns | bc -l`
				echo $name": totalIns="$totalIns" readIns="$readIns" writeIns="$writeIns" writeM="$writeM" MPKI="$MPKI
			fi
		done 
	done
	
}

doExperiment()
{
	count=1
	for work in  ${workLoad[@]};
	do
		mkdir -p ./m5out/${date}/${count}
		./build/X86/gem5.opt -d ./m5out/${date}/${count} configs/example/se.py --cmd="$work" --caches --l1d_size=64kB --l1i_size=64kB --l2cache --l2_size=8MB  --mem-type=dramsim2 -n $cpuCount --cpu-clock=4.0GHz --mem-size=4096MB  --cpu-type=DerivO3CPU  --sys-clock=4.0GHz > ./m5out/${date}/${count}/out &
		count=$(($count+1))
	done
}
resultExperiment()
{
	name=1
	for work in  ${workLoad[@]};
	do
		WS=0
		MS=0
		echo $work":"
		#splitWork=`echo $work | awk   -F ";" '{print $1" "$2" "$3" "$4" "$5" "$6" "$7" "$8}'`		
		splitWork=`echo $work | awk   -F ";" '{print $1" "$2" "$3" "$4"}'`		
		count=0
		for workLoad in ${splitWork};
		do
			tick=`grep "workload$count end. tick" m5out/$1/$name/out | awk -F "[=,]" '{print $2}'`
			insCount=`grep "workload$count end. tick" m5out/$1/$name/out | awk -F "=" '{print $3}'`
			if [ -n "${aloneWrokTime[$workLoad]}" ] 
			then
				IPC=`echo "$insCount*250/$tick" | bc -l`
				IPCB=`echo "$IPC/${aloneWrokTime[$workLoad]}" | bc -l`
				WS=`echo "$WS+$IPCB" | bc -l`
				re=`echo "${aloneWrokTime[$workLoad]}/$IPC > $MS" | bc`
				if [ "$re" == "1" ];
				then
					MS=`echo "${aloneWrokTime[$workLoad]}/$IPC" | bc -l`
				fi
				temp=`echo "${aloneWrokTime[$workLoad]}/$IPC" | bc -l`
				echo $workLoad":"$tick":"${aloneWrokTime[$workLoad]}":"$IPC":"$IPCB":"$temp
			fi	
			count=$(($count+1))	
		done
		name=$(($name+1))
		echo "WS:"$WS"MS:"$MS
		echo ""
	done
}

doExperimentOne()
{
	count=1
	for work in  ${benchMark[@]};
	do
		if [ $count -ge $1 ] && [ $count -le $2 ];
		then
			mkdir -p ./m5out/${date}/${work}
			./build/X86/gem5.opt -d ./m5out/${date}/${work} configs/example/se.py --cmd="$work" --caches --l1d_size=64kB --l1i_size=64kB --l2cache --l2_size=8MB --mem-size=4096MB   --mem-type=dramsim2 --cpu-type=DerivO3CPU --cpu-clock=4.0GHz --sys-clock=4.0GHz -I $maxIns > ./m5out/${date}/${work}/out &
		fi
		count=$(($count+1))
	done
}

resultExperimentOne()
{
	for work in  ${benchMark[@]};	
	do
		tick=`grep "Exiting @ tick" m5out/$1/$work/out | awk '{print $4}'`
		totalIns=`grep sim_insts m5out/$1/$work/stats.txt | awk '{print $2}'`
		aloneWrokTime[$work]=`echo "$totalIns*250/$tick" | bc -l`
		echo $work":"$totalIns":"$tick":${aloneWrokTime[$work]}"
	done
}

lastResut()
{
	resultExperimentOne $1
	resultExperiment $2
}
stopAll()
{
	if [ "$1" == "" ]
	then
		pids=`ps aux | grep build/X86/gem5.opt | awk '{print $2}'`
		echo $pids
		for pid in $pids;
		do
			kill $pid
		done
	else
		pids=`ps aux | grep $1 | awk '{print $2}'`
		echo $pids
		for pid in $pids;
		do
			kill $pid
		done
	fi
}
if [ "$1" == "--MPKI" ]  
then
	statisticsMPKI
elif [ "$1" == "--RMPKI" ] 
then
	resultMPKI $2
elif [ "$1" == "--EXP" ] 
then
	doExperiment	
elif [ "$1" == "--EXPONE" ] 
then
	doExperimentOne $2 $3
elif [ "$1" == "--REXPONE" ] 
then
	resultExperimentOne $2
elif [ "$1" == "--REXP" ] 
then
	resultExperiment $2
elif [ "$1" == "--STOP" ] 
then
	stopAll $2
elif [ "$1" == "--RLAST" ] 
then
	lastResut $2 $3
fi
