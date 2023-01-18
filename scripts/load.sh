#!/bin/bash

dir=$(dirname "${BASH_SOURCE[0]}")

#needs type_name_sn in array format like:
# type_name_sn=(motor_aksim_J1_SERIAL_NUMBER motor_aksim_J2_SERIAL_NUMBER)
cd $dir
type_name_sn=($(ls -d */))
len=${#type_name_sn[@]}
echo loading $len motors

for item in ${type_name_sn[@]%/}; do
    sn+=(${item##*-})
    tmp=${item%-*}
    name+=(${tmp##*-})
    type+=(${tmp%-*})
done

for (( i=0 ; i<$len ; i++)); do
    echo ${name[i]}
    [[ $(./${type_name_sn[i]}/load_${type[i]}.sh -S ${sn[i]} | grep "File downloaded successfully" | wc -l) -eq 2 ]] &
    pids+=($!)
    sleep 1 # usb enumerating too many devices at onces causes this to fail
done

for (( i=0 ; i<$len ; i++)); do
    wait ${pids[$i]}
    val=$?
    if [ $val -ne 0 ]; then
        failed+=(${name[$i]})
    else
        success+=(${name[$i]})
    fi
done

echo ${#failed[@]} failed
echo "-------------"
echo ${failed[*]}
echo
echo ${#success[@]} succeeded
echo "-------------"
echo ${success[*]}

if [[ ${#failed[@]} -gt 0 ]]; then
    exit 1
fi
