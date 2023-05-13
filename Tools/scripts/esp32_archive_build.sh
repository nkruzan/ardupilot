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

#set up our file names
outname="$boardname-$vehicle-$githash"
outfile="$outname.tar.gz"
#echo outfile: "$outfile"
ardupilot_bin="$build_dir"/ardupilot.bin
#echo bin file "$ardupilot_bin"
parttable_bin="$build_dir"/partition_table/partition-table.bin
#echo partition table file "$parttable_bin"
bootloader_bin="$build_dir"/bootloader/bootloader.bin
#echo bootloader file "$bootloader_bin"

#flash commands
flash_args_brd="esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 --before default_reset --after hard_reset write_flash -z "
flash_args_proj="$build_dir"/flash_project_args
flash_args_data=$(cat "$flash_args_proj")
flash_args_data=${flash_args_data//partition_table\//$empty}
flash_args_data=${flash_args_data//bootloader\//$empty}
flash_cmd_full=$(echo "$flash_args_brd""$flash_args_data" | tr '\n' ' ')

echo "creating temporary archive structure"
temp_dir="$build_dir"/temp
archive_dir="$temp_dir/$outname"

cd "$build_dir"
mkdir temp
cd temp
mkdir "$outname"
cp "$ardupilot_bin" "$outname/ardupilot.bin"
cp "$parttable_bin" "$outname/partition-table.bin"
cp "$bootloader_bin" "$outname/bootloader.bin"
echo "$flash_cmd_full" >> "$outname/flash_command.txt"

echo "creating output archive"
tar -czvf "$outfile" "$outname"
echo "done."

echo "deleting temporary archive structure"
rm -rf "$archive_dir"
echo "done."
echo "-----------------------"
echo "archive created: $temp_dir/$outfile"
echo "extract with: tar -xzvf $outfile"
echo "see flash_command.txt for flashing"

