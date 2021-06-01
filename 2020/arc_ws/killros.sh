#!/bin/bash

ps | grep roslaunch | awk '{print $1}' | xargs -I@ kill @
