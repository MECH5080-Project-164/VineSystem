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
    
    echo -e "${COLOR_YELLOW}--- Pico Temperature Monitor --- (Press Ctrl+C to exit)${COLOR_RESET}"
    echo -e "Bar Range: ${MIN_TEMP}°C to ${MAX_DISPLAY_TEMP}°C | Warning: >${WARN_TEMP}°C"
    
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
        echo "-----------------------------------------------------------------"
        echo -e "${COLOR_YELLOW}Sensor Group: $group${COLOR_RESET}"
        
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
    echo "-----------------------------------------------------------------"
    echo -e "Last update: $(date)"
}

# --- Main Logic ---

# Hide cursor for a cleaner look
tput civis

# Find all temperature topics using a bash array for safety
mapfile -t temp_topics < <(ros2 topic list | grep '/temperature')

if [ ${#temp_topics[@]} -eq 0 ]; then
    echo -e "${COLOR_RED}No temperature topics found. Is the Pico running and publishing?${COLOR_RESET}"
    cleanup
fi

echo -e "${COLOR_GREEN}Found topics: $(IFS=, ; echo "${temp_topics[*]}")"${COLOR_RESET}
echo "Starting monitor..."
sleep 2

# Main processing loop. It reads lines in the format "SensorName: data: value"
# and updates the state, then redraws the screen.
main_processor() {
    local unique_key data_keyword temp
    while read -r unique_key data_keyword temp; do
        # Line is formatted as: "PumpChassis/AmbientAir: data: 25.0"
        # We need to remove the trailing colon from the unique key
        unique_key=${unique_key%:}

        current_temps[$unique_key]=$temp
        
        if [ -z "${max_temps[$unique_key]}" ] || (( $(echo "$temp > ${max_temps[$unique_key]}" | bc -l) )); then
            max_temps[$unique_key]=$temp
        fi
        
        redraw_display
    done
}

# This block runs a separate 'ros2 topic echo' for each topic in the background.
# Each output is tagged with a unique key and piped to the main processor.
{
    # Create a background process for each topic
    for topic in "${temp_topics[@]}"; do
        {
            # The unique key is the topic path after /temperature/
            # e.g., /temperature/PumpChassis/AmbientAir -> PumpChassis/AmbientAir
            unique_key=${topic#"/temperature/"}
            
            # Echo the topic and filter for data lines, prepending the unique key
            ros2 topic echo "$topic" --no-daemon | while read -r line; do
                if [[ $line =~ "data:" ]]; then
                    echo "$unique_key: $line"
                fi
            done
        } &
    done
    # Wait for all background processes to finish (which they won't, until Ctrl+C)
    wait
} | main_processor

# Cleanup on natural exit (though unlikely with the loop)
cleanup
