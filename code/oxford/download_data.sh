#!/bin/bash
DATADIR='data/'
if [ -d "$DATADIR" ]; then
    echo "Data already exists"
else
    # Here if $DIRECTORY exists
    echo "Downloading data"
    mkdir data/
    wget https://www.dropbox.com/s/6szzv6o7hutxwy3/sample.tar?dl=0
    echo "Downloaded data"
    mv sample.tar?dl=0 $DATADIR/sample.tar
    cd $DATADIR
    tar xvf sample.tar
fi
