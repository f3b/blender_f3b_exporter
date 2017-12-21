#!/bin/bash

rm -Rf build/tmp
mkdir -p build/tmp

if [ ! -f "build/bash_colors.sh" ];
then
    wget  -q https://raw.githubusercontent.com/maxtsepkov/bash_colors/738f82882672babfaa21a2c5e78097d9d8118f91/bash_colors.sh -O build/bash_colors.sh
fi
source build/bash_colors.sh

function checkErrors {
    if [ $? -ne 0 ]; then 
        clr_red "Build failed."
        exit 1; 
    fi
}



function compilePy99 {
    if [ ! -f "build/py99.py" ];
    then
        wget -q https://raw.githubusercontent.com/riccardobl/py99/master/py99.py -O build/py99.py
    fi
   base=`basename $1`
   noext="${base%.*}"
   python3 build/py99.py $1 build/tmp/f3b_exporter/$noext.py
}
export -f compilePy99


function build {
    mkdir -p build/tmp/f3b_exporter

    cp src/*.py build/tmp/f3b_exporter/
    find src -type f -name '*.py99' -exec bash -c 'compilePy99 {}' \;

    cp -R docs build/tmp/f3b_exporter/
    cp README.md build/tmp/f3b_exporter/
    cp LICENSE build/tmp/f3b_exporter/

    for i in "$@" ; do
        if [ "$i" = "+ddswriter" ]; then
            echo "Bundle dds support"
            cp -R bin build/tmp/f3b_exporter/
            break
        fi
    done

    mkdir build/tmp/f3b_exporter/libs


    # Dependencies #

    # Protobuf
    if [ -f libs/protobuf.pylib ]; then
        cp -f libs/protobuf.pylib build/tmp/f3b_exporter/libs/
    else # Note: this won't work because it's incompatible with python3
        mkdir build/tmp/protobuf
        if [ ! -f build/protobuf.tar.gz ]; then
            wget "https://pypi.python.org/packages/2c/05/10c2611da9149677abfae24e208761794561e406c37d78c36bd8dda8ea80/protobuf-2.6.1.tar.gz"\
            -O  build/protobuf.tar.gz
        fi
        cp -f build/protobuf.tar.gz build/tmp/protobuf/pb.tar.gz
        cd  build/tmp/protobuf
        tar -xzf pb.tar.gz
        rm pb.tar.gz
        echo "" > */google/__init__.py
        mkdir ../protobuflib
        cp -Rvf */google ../protobuflib/
        cd ../protobuflib
        zip -r protobuf.pylib .
        cp -f protobuf.pylib ../f3b_exporter/libs/
        cd ../../../
    fi

    #F3b
    if [ "$1" = "dev" ]; then
      clr_green "Build dev version"
      cp -f "$HOME/.m2/repository/wf/frk/f3b/f3b/dev/f3b-dev-python.zip" build/tmp/f3b_exporter/libs/f3b.pylib
    else
      clr_green "Build production version"
      latest_version=`curl https://dl.bintray.com/riccardo/f3b/wf/frk/f3b/version.txt`
      clr_green "Use version $latest_version"
      rm -f  build/tmp/f3b_exporter/libs/f3b.pylib
      wget "https://dl.bintray.com/riccardo/f3b/wf/frk/f3b/f3b/$latest_version/f3b-$latest_version-python.zip" -O build/tmp/f3b_exporter/libs/f3b.pylib
    fi
    ###


    mkdir -p build/release
    rm -Rf build/release/f3b_exporter
    cp -Rf build/tmp/f3b_exporter build/release/
    cd build/release
    rm -f  f3b_exporter.zip
    zip -r f3b_exporter.zip f3b_exporter
}

function clean {
    rm -Rf build 
}

if [ "$1" = "" ];
then
    echo "Usage: make.sh target"
    echo " - Targets: build, build dev, clean"
    exit 0
fi
clr_magenta "Run $1 ${*:2}..."
$1 ${*:2}
clr_magenta "+ Done!"