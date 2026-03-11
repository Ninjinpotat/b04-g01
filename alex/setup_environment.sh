#!/bin/bash

LOGFILE="setup_environment.log"

exec > >(tee -i $LOGFILE)
exec 2>&1

############################################
############# CHECKING FOLDERS #############
############################################
# This script is meant to be run from the root of the project
# It checks if the libraries folder exists, and if the epp2 and external subfolders exist
# If they don't, it exits with an error message

# First we make sure that the CWD is in the folder containing this script
echo '==============================='
echo '= Setting up the environment ='
echo '==============================='
echo ''

echo "Changing directory to the location of the script..."
cd "$(dirname "$0")"
echo "Directory changed to $(pwd)"

# Then we make sure that the libraries folder exists
# We check if it exists, and if it doesn't, we exit
if [ ! -d "libraries" ]; then
    echo "ERROR: The libraries folder does not exist. Is this script in the right location?"
    # print where we suppose the libraries folder should be
    echo "Exiting..."
    exit 1
fi

# Within the libraries folder, we check if epp2 and external subfolders exists
# If they don't, we exit
if [ ! -d "libraries/epp2" ]; then
    echo "ERROR: The epp2 folder does not exist. Is this script in the right location?"
    # print where we suppose the epp2 folder should be
    echo "Exiting..."
    exit 1
fi

if [ ! -d "libraries/external" ]; then
    echo "ERROR: The external folder does not exist. Is this script in the right location?"
    # print where we suppose the external folder should be
    echo "Exiting..."
    exit 1
fi

echo ''
echo ''
echo ''

############################################
############# INSTALLING PYTHON ############
############################################
# We check if python and the python3-dev python3-venv packages are installed
# If they are not, we install them

echo '==============================='
echo '= Installing Python          ='
echo '==============================='
echo ''

# Skip the python install if it is already installed
if ! command -v python &> /dev/null; then
    echo "Python is not installed. Installing..."
    sudo apt-get install python
    echo ''
fi

# Install pip
# Skip the python3-pip install if it is already installed
if ! dpkg -s python3-pip &> /dev/null; then
    echo "python3-pip is not installed. Installing..."
    sudo apt-get install python3-pip
    echo ''
fi

# Skip the python3-venv install if it is already installed
if ! dpkg -s python3-venv &> /dev/null; then
    echo "python3-venv is not installed. Installing..."
    sudo apt-get install python3-venv
    echo ''
fi

# Skip the python3-dev install if it is already installed
if ! dpkg -s python3-dev &> /dev/null; then
    echo "python3-dev is not installed. Installing..."
    sudo apt-get install python3-dev
    echo ''
fi

# Install TK addon for PIL
if ! dpkg -s python3-pil.imagetk &> /dev/null; then
    echo "python3-pil.imagetk is not installed. Installing..."
    sudo apt-get install python3-pil.imagetk
    echo ''
fi

echo ''
echo ''
echo ''

##################################################
############ Virtual Environment Setup ###########
##################################################
# We create a virtual environment in the root of the project
# We activate the virtual environment and install the required packages

echo '==============================='
echo '= Installing Dependencies    ='
echo '==============================='
echo ''


# Check if the env folder exists, if it is, skip the virtual environment setup
# we provide access to the system site packages to allow for the use of -dev packages
if [ ! -d "env" ]; then
    echo "Creating virtual environment..."
    python3 -m venv ./env --system-site-packages
    echo "Virtual environment created."
    echo ''
fi

# Activate the virtual environment
source env/bin/activate

# check if in the virtual environment
if [ -z "$VIRTUAL_ENV" ]; then
    echo "ERROR: Virtual environment not activated. Exiting..."
    exit 1
fi

# Update pip to the latest version
python -m pip install --upgrade pip
echo ''

##################################################
############ INSTALLING EXTERNAL DEPS ############
##################################################
# We install global dependencies in the virtual environment
pip install numpy matplotlib pyserial
echo ''

# We install the any additional dependencies in the external folder
# Check if BreezySLAM is installed via pip, if not install it
# if ! pip show BreezySLAM &> /dev/null; then
#     echo "Installing BreezySLAM..."
#     pip install -e ./libraries/external/BreezySLAM/python --config-settings editable_mode=compat
#     echo ''
# fi

# Check if the pyrplidar library is installed via pip, if not install it
if ! pip show pyrplidar &> /dev/null; then
    echo "Installing pyrplidar..."
    pip install -e ./libraries/external/pyrplidar --config-settings editable_mode=compat
    echo ''
fi

# We install the epp2 specific libraries
# Check if the epp2 library is installed via pip
# we iterate through the epp2 folder and install each library
# (i.e. pip install -e ./libraries/epp2/lidar --config-settings editable_mode=compat)
# (i.e. pip install -e ./libraries/epp2/control --config-settings editable_mode=compat)
# (i.e. pip install -e ./libraries/epp2/slam --config-settings editable_mode=compat)
# (i.e. pip install -e ./libraries/epp2/pubsub --config-settings editable_mode=compat)
# (i.e. pip install -e ./libraries/epp2/display --config-settings editable_mode=compat)
for library in ./libraries/epp2/*; do
    if [ -d "$library" ]; then
        if ! pip show $(basename $library) &> /dev/null; then
            echo "Installing $(basename $library)..."
            pip install -e $library --config-settings editable_mode=compat
            echo ''
        fi
    fi
done

# Return control to the user
echo "Setup complete. Exiting..."
exit 0
