#!/bin/ash
CSV_FILE="/overlay/upper/root/monitoring_data.csv"
COMMAND="gsmctl -oqt"
COMMAND_SENT_DATA="gsmctl -e qmimux0"
COMMAND_RECEIVED_DATA="gsmctl -r qmimux0"
COMMAND_NETWORK="gsmctl -F"
# Check if the CSV file already exists, and if not, create one with header row and specific column names
#if [ ! -f "$CSV_FILE" ]; then
#  echo "RSSI,RSRP,SINR,RSRQ" > "$CSV_FILE"
#fi
# Loop save to the CSV file in each iteration
k=0
while true
do
  # Get the current time using the `date` command and extract the time portion only
  CURRENT_TIME=$(date +%T)
  echo -n "$CURRENT_TIME" >> "$CSV_FILE"
  # Get gsmctl output and go over each line
  OUTPUT=$(eval "$COMMAND")
  echo "$OUTPUT\n"
  # Extract values using awk
  rssi=$(echo "$OUTPUT" | awk '/RSSI:/{print $2}')
  rsrp=$(echo "$OUTPUT" | awk '/RSRP:/{print $2}')
  sinr=$(echo "$OUTPUT" | awk '/SINR:/{print $2}')
  rsrq=$(echo "$OUTPUT" | awk '/RSRQ:/{print $2}')
  data_sent=$(eval "$COMMAND_SENT_DATA")
  data_received=$(eval "$COMMAND_RECEIVED_DATA")
  OUTPUT_NETWORK=$(eval "$COMMAND_NETWORK")
  echo $OUTPUT_NETWORK
  # Remove leading and trailing whitespaces
  trimmed_output="$(echo "$OUTPUT_NETWORK" | sed 's/^ *//;s/ *$//')"
  # Split the trimmed output using the pipe character as the delimiter
  IFS="|"
  set -- $OUTPUT_NETWORK  # Split the output using the IFS and assign values to positional parameters
  # Trim leading and trailing whitespaces from the values
  connection_type=$(echo "$1" | awk '{$1=$1;print}')
  band=$(echo "$2" | awk '{$1=$1;print}')
  plmn=$(echo "$3" | awk '{$1=$1;print}')
  # Get current timestamp in InfluxDB format
  timestamp=$(date +%s%N):
  # Prepare CSV data and save to file
  csv_data="RSSI,RSRP,SINR,RSRQ\n$rssi,$rsrp,$sinr,$rsrq"
  echo -e "$csv_data" > "$CSV_FILE"
  # Prepare string for influxdb post
  data="gsmctl,tag=openwrt RSSI=$rssi,RSRP=$rsrp,SINR=$sinr,RSRQ=$rsrq,SENT=$data_sent,RECEIVED=$data_received,Conn_Type=\"$connection_type\",Band=\"$band\""
  #echo $data
  # Send CSV data to InfluxDB using curl XPOST request
  influxdb_url="http://192.168.1.200:8086/write?db=openwrt"
  curl -X POST "$influxdb_url" --data-binary "$data"
  # Wait for 1 second before looping again
  sleep 1
  echo "$k"
  let k++
done
