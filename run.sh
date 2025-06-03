#!/bin/bash

gnome-terminal -- bash -c "cd build && ./k4a_process; exec bash"

sleep 2

gnome-terminal -- bash -c "cd build && ./rs_process; exec bash"