#!/bin/bash

# Script to monitor ROS2 temperature topics from a Raspberry Pi Pico
# Displays a dashboard with current temperature, max temperature, and a visual bar.

# --- Configuration ---
WARN_TEMP=60.0
MAX_DISPLAY_TEMP=80.0
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
    
    # Calculate the percentage of the bar to fill, now spanning from MIN_TEMP to MAX_DISPLAY_TEMP
    bar_percentage=$(echo "scale=4; ($temp - $MIN_TEMP) / ($MAX_DISPLAY_TEMP - $MIN_TEMP)" | bc)

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
    # Move cursor to top-left corner instead of clearing the screen
    tput cup 0 0
    
    echo -e "${COLOR_YELLOW}--- Pico Temperature Monitor --- (Press Ctrl+C to exit)${COLOR_RESET}$(tput el)"
    echo -e "Bar Range: ${MIN_TEMP}°C to ${MAX_DISPLAY_TEMP}°C | Warning: >${WARN_TEMP}°C$(tput el)"
    
    # Group sensors by their parent directory (e.g., PumpChassis)
    declare -A grouped_sensors
    for key in "${!current_temps[@]}"; do
        group=$(dirname "$key")
        # Use a simple space-separated list of keys for each group
        grouped_sensors["$group"]+="$key "
    done

    # Get sorted group names
    sorted_groups=($(for group in "${!grouped_sensors[@]}"; do echo "$group"; done | sort))

    # Loop through sorted groups and display their data
    for group in "${sorted_groups[@]}"; do
        echo "-----------------------------------------------------------------$(tput el)"
        echo -e "${COLOR_YELLOW}Sensor Group: $group${COLOR_RESET}$(tput el)"
        
        # Read keys into an array and sort them
        read -r -a group_keys <<< "${grouped_sensors[$group]}"
        sorted_keys=($(for key in "${group_keys[@]}"; do echo "$key"; done | sort))

        for key in "${sorted_keys[@]}"; do
            local temp=${current_temps[$key]}
            local max_temp=${max_temps[$key]}
            # The display name is the final part of the key (e.g., AmbientAir)
            local display_name=$(basename "$key")
            
            local color=$COLOR_GREEN
            if (( $(echo "$temp >= $WARN_TEMP" | bc -l) )); then
                color=$COLOR_RED
            fi

            local bar=$(draw_bar "$temp")

            printf "${color}%-20s: %s %5.1f°C (Max: %5.1f°C)${COLOR_RESET}\n" "$display_name" "$bar" "$temp" "$max_temp"
        done
    done
    echo "-----------------------------------------------------------------$(tput el)"
    echo -e "Last update: $(date)$(tput el)"
    
    # Clear from the cursor to the end of the screen to remove old artifacts
    tput ed
}

# --- Main Logic ---

# Hide cursor for a cleaner look
tput civis

# Find all temperature topics and their types
mapfile -t topic_info < <(ros2 topic list -t | grep '/temperature')

if [ ${#topic_info[@]} -eq 0 ]; then
    echo -e "${COLOR_RED}No temperature topics found. Is the Pico running and publishing?${COLOR_RESET}"
    cleanup
fi

# Declare an associative array to store topic names and their types
declare -A topics_and_types
for info in "${topic_info[@]}"; do
    # Extract topic name and type
    topic_name=$(echo "$info" | awk '{print $1}')
    topic_type=$(echo "$info" | awk '{print $2}' | tr -d '[]')
    topics_and_types["$topic_name"]=$topic_type
done

echo -e "${COLOR_GREEN}Found topics: $(IFS=, ; echo "${!topics_and_types[*]}")"${COLOR_RESET}
echo "Starting monitor..."
sleep 2

# Data processing function - runs in the background
process_data_pipe() {
    local unique_key data_keyword temp
    while read -r unique_key data_keyword temp; do
        # Line is formatted as: "PumpChassis/AmbientAir: data: 25.0"
        # We need to remove the trailing colon from the unique key
        unique_key=${unique_key%:}

        current_temps[$unique_key]=$temp
        
        if [ -z "${max_temps[$unique_key]}" ] || (( $(echo "$temp > ${max_temps[$unique_key]}" | bc -l) )); then
            max_temps[$unique_key]=$temp
        fi
    done
}

# This block runs a separate 'ros2 topic echo' for each topic in the background.
# The entire output is piped to the data processor.
{
    for topic in "${!topics_and_types[@]}"; do
        {
            # The unique key is the topic path after /temperature/
            # e.g., /temperature/PumpChassis/AmbientAir -> PumpChassis/AmbientAir
            unique_key=${topic#"/temperature/"}
            topic_type=${topics_and_types[$topic]}
            
            # Echo the topic with its type and filter for data lines, prepending the unique key
            ros2 topic echo "$topic" "$topic_type" --no-daemon | while read -r line; do
                if [[ $line =~ "data:" ]]; then
                    echo "$unique_key: $line"
                fi
            done
        } &
    done
    wait
} | process_data_pipe & # Run the data processing in the background

# Foreground loop for redrawing the screen at a fixed rate
while true; do
    redraw_display
    sleep 0.1 # 10Hz refresh rate
done

# Cleanup on natural exit (though unlikely with the loop)
cleanup
