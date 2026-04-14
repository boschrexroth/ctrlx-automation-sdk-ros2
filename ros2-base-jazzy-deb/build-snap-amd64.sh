#!/bin/bash
set -e

# 'sudo' is required independently of the snapcraft version (here 8.x)
sudo snapcraft clean
sudo snapcraft pack --build-for=amd64 --verbosity=verbose
