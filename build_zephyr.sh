#!/bin/bash

echo "Building project..."

west build -b xiao_ble/nrf52840/sense --pristine
