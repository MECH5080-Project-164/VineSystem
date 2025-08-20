#!/bin/bash

# Script to monitor ROS2 temperature topics from a Raspberry Pi Pico
# Displays a dashboard with current temperature, max temperature, and a visual bar.

# --- Configuration ---
WARN_TEMP=40.0
MIN_TEMP=20.0
BAR_WIDTH=20

# --- ANSI Color Codes ---
COLOR_RED='\033[0;31m'
COLOR_YELLOW='\033[1;33m'
COLOR_GREEN='\033[0;32m'
COLOR_RESET='\033[0m'

# --- State Tracking ---
declare -A current_temps
declare -A max_temps
declare -A last_update_time

# --- Functions ---

# Function to handle script exit
cleanup() {
    echo -e "\n${COLOR_RESET}Stopping temperature monitor. Restoring cursor."
    tput cnorm # Restore cursor
    exit 0
}

# Set up trap for Ctrl+C
trap cleanup INT

# Function to draw a temperature bar
draw_bar() {
    local temp=$1
    local bar_percentage
    
    # Calculate the percentage of the bar to fill
    bar_percentage=$(echo "scale=4; ($temp - $MIN_TEMP) / ($WARN_TEMP - $MIN_TEMP)" | bc)

    # Clamp percentage between 0 and 1
    if (( $(echo "$bar_percentage < 0" | bc -l) )); then bar_percentage=0; fi
    if (( $(echo "$bar_percentage > 1" | bc -l) )); then bar_percentage=1; fi

    local filled_width=$(echo "($bar_percentage * $BAR_WIDTH)/1" | bc)
    local empty_width=$((BAR_WIDTH - filled_width))

    # Build the bar string
    local bar="["
    for ((i=0; i<filled_width; i++)); do bar+="#"; done
    for ((i=0; i<empty_width; i++)); do bar+="-"; done
    bar+="]"
    
    echo -n "$bar"
}

# Main display function
redraw_display() {
    tput clear # Clear the terminal
    echo -e "${COLOR_YELLOW}--- Pico Temperature Monitor --- (Press Ctrl+C to exit)${COLOR_RESET}"
    echo "-----------------------------------------------------------------"
    
    # Loop through all known sensors and display their data
    for sensor in "${!current_temps[@]}"; do
        local temp=${current_temps[$sensor]}
        local max_temp=${max_temps[$sensor]}
        
        local color=$COLOR_GREEN
        if (( $(echo "$temp >= $WARN_TEMP" | bc -l) )); then
            color=$COLOR_RED
        fi

        local bar=$(draw_bar "$temp")

        printf "${color}%-20s: %s %5.1f°C (Max: %5.1f°C)${COLOR_RESET}\n" "$sensor" "$bar" "$temp" "$max_temp"
    done
    echo "-----------------------------------------------------------------"
    echo -e "Last update: $(date)"
}

# --- Main Logic ---

# Hide cursor for a cleaner look
tput civis

# Find all temperature topics
temp_topics=$(ros2 topic list | grep '/temperature')

if [ -z "$temp_topics" ]; then
    echo -e "${COLOR_RED}No temperature topics found. Is the Pico running and publishing?${COLOR_RESET}"
    cleanup
fi

echo -e "${COLOR_GREEN}Found topics: ${temp_topics//$'\n'/, }${COLOR_RESET}"
echo "Starting monitor..."
sleep 2

# Subscribe to all found topics and process the data
ros2 topic echo $temp_topics --no-daemon | while read -r line; do
    if [[ $line == "---" ]]; then
        # Identify the topic name from the line before the '---'
        topic_name=$(echo "$prev_line" | tr -d ':')
        sensor_name=$(echo "$topic_name" | awk -F'/' '{print $NF}')
    elif [[ $line =~ "data:" ]]; then
        # Extract temperature value
        current_temp=$(echo "$line" | awk '{print $2}')
        
        # Update current temperature
        current_temps[$sensor_name]=$current_temp
        
        # Update max temperature if needed
        if [ -z "${max_temps[$sensor_name]}" ] || (( $(echo "$current_temp > ${max_temps[$sensor_name]}" | bc -l) )); then
            max_temps[$sensor_name]=$current_temp
        fi
        
        # Redraw the entire display with new data
        redraw_display
    fi
    prev_line=$line
done

# Cleanup on natural exit (though unlikely with the loop)
cleanup
