# Obstsalat - Open Data Camp 2019

## Setup
```
# Clone via ssh
git clone git@github.com:vroland/obstsalat-odc19.git

# Create virtual environment
python -m venv env

# Activate environment
source env/bin/activate

# Install dependencies
pip install -r requirements.txt
```

## Running

### Setting up Openrouteservice
* clone the openrouteservice repository
* To disable openrouteservice rate limiting, apply `./openrouteservice_config.patch` to the sample config of the openrouteservice docker container.
* Download an appropriate open streetmap dataset (e.g. saxony)
* delete everything in `docker/graphs`! This will force openrouteservice to rebuild the map data. Otherwise, you might get unexpected `Error Code 2099` errors, because the requested coordinates are off map.
* make sure the container has enough RAM available. Increase RAM limit as described in https://github.com/GIScience/openrouteservice/blob/master/docker/README.md. Using a VM with 3G of ram did not work for me, while 6G worked just fine.

### Running locally
* Make sure you have openrouteservice running under localhost (follow https://github.com/GIScience/openrouteservice/blob/master/docker/README.md and load the dataset for saxony)
* Start the backend service: `cd obst/tsopt/ && python3 optimize.py`. It should load the pre-computed dataset graphs.
* Open the front-end file in `obst/ui/index.html`
