#!/bin/bash

#hacky script to archive an esp32 build
# nkruzan
#hacky stuff to get board name
blank=""
string="# using /home/user/ardupilot/modules/waf/waf-light configure --board="
input="/home/user/ardupilot/build/config.log"
while read -r line
do
    if [[ "$line" == *"$string"* ]]; then
        boardname=$(echo "${line/"$string"/"$blank"}")
        break
    fi
done < "$input"
echo creating archive for "$boardname"
#hacky stuff to get vehicle type
input=/home/user/ardupilot/build/"$boardname"/esp-idf_build/CMakeCache.txt
string="ARDUPILOT_CMD:UNINITIALIZED="
while read -r line
do
    if [[ "$line" == *"$string"* ]]; then
        vehicle=$(echo "${line/"$string"/"$blank"}")
        break
    fi
done < "$input"
echo vehicle "$vehicle"
githash=$(echo `git rev-parse HEAD`)
echo git hash "$githash"

build_dir=/home/user/ardupilot/build/"$boardname"/esp-idf_build


flash_args_brd="esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 --before default_reset --after hard_reset write_flash -z "
flash_args_proj="$build_dir"/flash_project_args
flash_args_data=$(cat "$flash_args_proj")
flash_args_data=${flash_args_data//partition_table\//$empty}
flash_args_data=${flash_args_data//bootloader\//$empty}
flash_cmd_full=$(echo "$flash_args_brd""$flash_args_data" | tr '\n' ' ')



#set up our file names
outfile="$boardname-$vehicle-$githash.apj"

#echo "generating .apj"
apbin_data=$(base64 "$build_dir"/ardupilot.bin)
parttable_data=$(base64 "$build_dir"/partition_table/partition-table.bin)
bootloader_data=$(base64 "$build_dir"/bootloader/bootloader.bin)
flashinfo_data=$(base64 $flash_cmd_full)

cat > $outfile <<APJDATA
{
    'board_id': $boardname, 
    'magic': 'APJFWAv1', 
    'description': 'ArduPilot Firmware for $boardname - $vehicle',
    'vehicle':'$vehicle', 
    'firmware': '$apbin_data', 
    'parttable': '$parttable_data', 
    'bootloader': '$bootloader_data', 
    'flashinfo': '$flashinfo_data', 
    'image_maxsize': xxx, 
    'summary': $boardname, 
    'version': '0.1', 
    'git_identity': $githash, 
}
APJDATA
echo ".apj created: $outfile"
