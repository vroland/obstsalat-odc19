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

* Make sure you have openrouteservice running under localhost (follow https://github.com/GIScience/openrouteservice/blob/master/docker/README.md and load the dataset for saxony)
* Start the backend service: `cd obst/tsopt/ && python3 optimize.py`. It should load the pre-computed dataset graphs.
* Open the front-end file in `obst/ui/index.html`

* To disable openrouteservice rate limiting, apply `./openrouteservice_config.patch` to the sample config of the openrouteservice docker container.
