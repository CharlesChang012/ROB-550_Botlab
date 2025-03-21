#!/bin/bash
set -e  # Quit on error.

BRIDGE_VERSION="v1.0.0"  # The MBot Bridge version to download if no path is passed.
BRIDGE_PATH=""           # The path to a local version of MBot Bridge. Overrides downloading a release.
REBUILD_APP=true         # Whether to rebuild the app.

# Directory where the script is executed from
SCRIPT_DIR=$(pwd)

# Function to show usage information
usage() {
  echo "Usage: $0 [--bridge-path PATH/TO/BRIDGE] [--no-rebuild]"
  exit 1
}

# Parse the arguments
while [[ "$#" -gt 0 ]]; do
  case $1 in
    --bridge-path)
      if [ -z "$2" ]; then
        echo "Error: Path not provided."
        usage
      fi
      BRIDGE_PATH="$2"
      shift
      ;;
    --no-rebuild)
      REBUILD_APP=false
      ;;
    *)
      echo "Unknown parameter: $1"
      usage
      ;;
  esac
  shift
done

if $REBUILD_APP; then
  # Check if bridge path was provided
  if [ -n "$BRIDGE_PATH" ]; then
    # Check if the provided path exists
    if [ ! -d "$BRIDGE_PATH/mbot_js" ]; then
      echo "Error: The MBot Bridge JS API does not exist in the provided path: $BRIDGE_PATH"
      usage
      exit 1
    fi
    echo "##############################################################"
    echo "Installing and linking the MBot Bridge from provided path..."
    echo "##############################################################"
    cd $BRIDGE_PATH/mbot_js
    npm install
    npm link
  else
    echo "##############################################################"
    echo "Installing and linking the MBot Bridge from release $BRIDGE_VERSION..."
    echo "##############################################################"
    echo "Downloading MBot Bridge $BRIDGE_VERSION"
    wget https://github.com/mbot-project/mbot_bridge/archive/refs/tags/$BRIDGE_VERSION.tar.gz
    tar -xzf $BRIDGE_VERSION.tar.gz
    cd mbot_bridge-${BRIDGE_VERSION#v}/mbot_js
    npm install
    npm link
  fi

  # Build the webapp.
  echo
  echo "#############################"
  echo "Building the webapp..."
  echo "#############################"
  cd $SCRIPT_DIR
  npm install
  npm link mbot-js-api
  npm run build

  # Clean up.
  if [ -f "$BRIDGE_VERSION.tar.gz" ]; then
    echo
    echo "Cleaning up downloaded files..."
    rm $BRIDGE_VERSION.tar.gz
    rm -rf mbot_bridge-${BRIDGE_VERSION#v}/
  fi
else
  # No rebuilding from source.
  echo "Webapp will be installed from the dist/ folder without rebuild."
  if [ -n "$BRIDGE_PATH" ]; then
    echo "Provided MBot Bridge path will be ignored."
  fi
fi

echo
echo "Installing the web app..."
echo
# Remove any old files to ensure we don't make duplicates.
sudo rm -rf /data/www/mbot/*
# Move the build files into the public repo.
sudo cp -r dist/* /data/www/mbot/

echo "Restarting Nginx..."
echo
sudo systemctl restart nginx

echo "#############################"
echo "Setting up Python server..."
echo "#############################"
echo

ENVS_ROOT="/home/$USER/.envs"
MBOT_APP_ENV="$ENVS_ROOT/mbot-app-env/"  # Virtual env where app is run.

if [ ! -d "/data/www/mbot/api" ]; then
    sudo mkdir /data/www/mbot/api
fi

# Copy over all the needed Python code.
sudo cp mbot_omni_app.py /data/www/mbot/api

if [ ! -f "/etc/systemd/system/mbot-web-server.service" ]; then
  # This is the first time installing.
  sudo cp config/mbot-web-server.service /etc/systemd/system/
  # Fill in the path to this env and the correct Python path.
  sudo sed -i "s#WEBAPP_ENV_PATH#$MBOT_APP_ENV#" /etc/systemd/system/mbot-web-server.service

  echo "Enabling MBot Web App service."
  # Reload the service.
  sudo systemctl daemon-reload
  sudo systemctl enable mbot-web-server.service
  sudo systemctl start mbot-web-server.service
else
  # This service has already been installed. Pull new changes then restart it.
  sudo cp config/mbot-web-server.service /etc/systemd/system/
  # Fill in the path to this env.
  sudo sed -i "s#WEBAPP_ENV_PATH#$MBOT_APP_ENV#" /etc/systemd/system/mbot-web-server.service

  echo "MBot Web App service is already enabled. Restarting it."
  sudo systemctl daemon-reload
  sudo systemctl restart mbot-web-server.service
fi

echo
echo "Done! The webapp is now available at http://localhost on this computer or http://[MBOT_IP] on the network."
