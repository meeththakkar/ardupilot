#!/bin/bash

# Try to run a command in an appropriate type of terminal window
# depending on whats available
# Sigh: theres no common way of handling command line args :-(
name="$1"
shift
echo "RiTW: Starting $name : $*"
# default to xterm as it has the most consistent options and can start minimised
if [ -n "$DISPLAY" -a -n "$(which osascript)" ]; then
  echo "A"
  osascript -e 'tell application "Terminal" to do script "'"$* "'"'
elif [ -n "$DISPLAY" -a -n "$(which xterm)" ]; then
  echo "B"
  xterm -iconic -xrm 'XTerm*selectToClipboard: true' -xrm 'XTerm*initialFont: 6' -n "$name" -name "$name" -T "$name" -hold -e "$* 2>&1> /tmp/hex.log" &
elif [ -n "$DISPLAY" -a -n "$(which konsole)" ]; then
  echo "c"
  konsole --hold -e $*
elif [ -n "$DISPLAY" -a -n "$(which gnome-terminal)" ]; then
  echo "D"
  gnome-terminal -e "$*"
elif [ -n "$STY" ]; then
  echo "E"
  # We are running inside of screen, try to start it there
  screen -X screen -t "$name" $*
else
  echo "F"
  filename="/tmp/$name.log"
  echo "RiTW: Window access not found, logging to $filename"
  cmd="$1"
  shift
# the following "true" is to avoid bash optimising the following call
# to avoid creating a subshell.  We need that subshell, or
# _fdm_input_step sees ArduPilot has no parent and kills ArduPilot!
  ( : ; $cmd $* &>"$filename" < /dev/null ) &
fi
exit 0
