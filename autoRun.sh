#!/bin/bash

benchMark=("bzip2" "mcf" "libquantum" "soplex" "lbm" "milc")
maxIns = 1000000000
date = `date -d today +"%Y-%m-%d %H:%M:%S"`

statisticsMPKI()
{
	for name in ${benchMark[@]};
	do
		./build/X86/gem5.opt -d ./m5out/${date}/${name} configs/example/se.py -c $name --caches --l1d_size=64kB --l1i_size=64kB --l2cache --l2_size=8MB  -I $maxIns --mem-type=SimpleMemory &
	done
}

resultMPKI()
{
	for file in /m5out/${date}/${name}/*
	do
		if [ "$file" == "stats.txt" ]
		then
			readIns=`grep system.mem_ctrls.num_reads::total $file | awk  '{print $2}'`
			writeIns=`grep system.mem_ctrls.num_writes::total $file | awk  '{print $2}'`
			echo $name":"$readIns" "$writeIns" "`echo $writeIns"/ ("$readIns"+"$writeIns")" | bc -l`
		fi
	done 
	
}

if [ "$1" == "--MPKI" ]  #运行测试程序来获取MPKI和读写密度
then
	statisticsMPKI
elif [ "$1" == "--RMPKI" ] #获取每个测试程序的MPKI和读写密度
fi
