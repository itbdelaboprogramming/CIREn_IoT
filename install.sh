#!/bin/bash
# Installation scripts for setting up the environment
# 1. Install and setup MQTT broker
# 2. Install and setup InfluxDB
# 3. Expose port for MQTT
# 4. Create InfluxDB database
# 5. Activate Telegraf MQTT and other plugins

# Ubuntu and Debian
# Add the InfluxData key to verify downloads and add the repository
curl --silent --location -O https://repos.influxdata.com/influxdata-archive.key
gpg --show-keys --with-fingerprint --with-colons ./influxdata-archive.key 2>&1 \
| grep -q '^fpr:\+24C975CBA61A024EE1B631787C3D57159FC2F927:$' \
&& cat influxdata-archive.key \
| gpg --dearmor \
| sudo tee /etc/apt/keyrings/influxdata-archive.gpg > /dev/null \
&& echo 'deb [signed-by=/etc/apt/keyrings/influxdata-archive.gpg] https://repos.influxdata.com/debian stable main' \
| sudo tee /etc/apt/sources.list.d/influxdata.list

sudo apt-get update -y
sudo apt-get upgrade -y

sudo apt-get install mosquitto mosquitto-clients openssl influxdb2 telegraf jq -y

# wget https://dl.influxdata.com/influxdb/releases/influxdb2-client-2.7.5-linux-amd64.tar.gz
# tar xvzf ./influxdb2-client-2.7.5-linux-amd64.tar.gz
# sudo cp ./influx /usr/local/bin/

sudo service influxdb start
sudo service telegraf start

# Wait for services to start
sleep 10

# Check if InfluxDB is running
if ! sudo service influxdb status | grep -q "active (running)"; then
    echo "InfluxDB is not running. Please check the installation."
    exit 1
fi

## InfluxDB Setup
# Initial setup of InfluxDB. Replace with your desired credentials and bucket.
echo "Setting up InfluxDB..."
influx setup --host http://localhost:8086 \
  --username influxuser \
  --password your_secure_password \
  --org deLabo \
  --bucket deLabo \
  --force \
  --skip-verify

# Retrieve the InfluxDB token for subsequent commands
INFLUX_TOKEN=$(influx auth list --json | jq -r '.[0].token')

## MQTT Setup
echo "Configuring Mosquitto MQTT broker..."
# Expose the default MQTT port (1883) by adding a listener to the mosquitto config
echo "listener 1883" | sudo tee -a /etc/mosquitto/mosquitto.conf
echo "allow_anonymous true" | sudo tee -a /etc/mosquitto/mosquitto.conf

# Restart the Mosquitto service to apply the new configuration
sudo service mosquitto restart
# sudo service mosquitto status

## Telegraf Setup
# This section creates a new telegraf configuration file for MQTT.
# It uses the token retrieved from the setup to authenticate with InfluxDB.
echo "Configuring Telegraf for MQTT..."
cat << EOF | sudo tee /etc/telegraf/telegraf.conf > /dev/null
# Global tags, useful for host or location information
[global_tags]
  # host = "\$HOSTNAME"

# Configuration for Telegraf Agent
[agent]
  interval = "10s"
  round_interval = true
  metric_batch_size = 1000
  metric_buffer_limit = 10000
  collection_jitter = "0s"
  flush_interval = "10s"
  flush_jitter = "0s"
  precision = ""
  hostname = ""
  omit_hostname = false

# Output plugin for InfluxDB
[[outputs.influxdb_v2]]
  urls = ["http://localhost:8086"]
  token = "${INFLUX_TOKEN}"
  organization = "deLabo"
  bucket = "deLabo"
  timeout = "5s"

# Input plugin for MQTT
[[inputs.mqtt_consumer]]
  servers = ["tcp://localhost:1883"]
  topics = [
    "sensors/temperature",
    "sensors/humidity"
  ]
  data_format = "influxdb_v2"
EOF

sudo service telegraf restart
echo "Telegraf configured and restarted."
echo "Script finished successfully!"