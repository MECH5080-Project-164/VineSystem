#!/bin/bash

# Verify that we're in the Vine Container
if [ -z "$VINE_CONTAINER" ]; then
    echo "This script must be run inside the Vine Container."
    exit 1
fi
