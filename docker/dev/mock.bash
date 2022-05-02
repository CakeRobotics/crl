#!/bin/bash
# Installs a fake apt package.
# Usage: ./mock.bash libhello libgoodbye libseeyou=4.3.2

set -e
apt install -y equivs
for pkgfullname in "$@"; do
    pkg=$(echo ${pkgfullname} | grep -oP "^[^=]+")
    ver=$(echo ${pkgfullname} | grep -oP "=\K.+\$" || echo "1.0.0")
    mkdir -p /tmp/$pkg
    pushd /tmp/$pkg
    equivs-control $pkg.control
    sed -i "s/<package name; defaults to equivs-dummy>/$pkg/g" $pkg.control
    echo "Version: ${ver}" >> $pkg.control
    equivs-build $pkg.control
    dpkg -i ${pkg}_${ver}_all.deb
    popd
done
