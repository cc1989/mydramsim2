#!/bin/bash

benchMark=("bzip2" "mcf" "libquantum" "soplex" "lbm" "milc" "leslie3d" "GemsFDTD" "sphinx3" "xalancbmk" "omnetpp" "cactusADM" "astar" "hmmer" "h264ref" "gromacs" "gobmk" "sjeng" "gcc" "dealII" "wrf" "namd" "perlbench" "calculix" "tonto" "povray" "specrandi" "zeusmp" "gamess" "bwaves")
maxIns=1000000000
date=`date -d today +"%Y-%m-%d:%H:%M:%S"`

statisticsMPKI()
{
	for name in ${benchMark[@]};
	do
		./build/X86/gem5.opt -d ./m5out/${date}/${name} configs/example/se.py -c $name --caches --l1d_size=64kB --l1i_size=64kB --l2cache --l2_size=8MB  -I $maxIns --mem-type=SimpleMemory &
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

if [ "$1" == "--MPKI" ]  
then
	statisticsMPKI
elif [ "$1" == "--RMPKI" ] 
then
	resultMPKI $2
fi
