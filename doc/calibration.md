# Using the C-file method for calibration

 ```console
 # Make a directory outside of the repo to store files
 mkdir ~/cal
 
 # Copy the example file and rename
 cp ../motorlib/calibration_example.c ~/cal/cal1.c

 # Edit the file (for example with vscode)
 code ~/cal/cal1.c

 # Build and load
 make CONFIG=motor_enc CALIBRATION_FILE=~/cal/cal1.c -j
 ./build/motor_enc/load_motor_enc.sh

 # Some calibration values are printed at startup in the log 
 # for validation
 motor_util read --text
 ```
 