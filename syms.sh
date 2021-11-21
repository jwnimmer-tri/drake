#!/bin/bash

set -ex

cd /home/jwnimmer/jwnimmer-tri/drake
# readelf -W --dyn-syms bazel-bin/tools/install/libdrake/libdrake.so | c++filt > syms-libdrake.txt
cat syms-libdrake.txt |
    perl -ne 'print if (!m#^.{59}drake#)' |
    perl -ne 'print if (!m#^.{59}Eigen#)' |
    perl -ne 'print if (!m#[< ]drake::#)' |
    perl -ne 'print if (!m#[< ]Eigen::#)' |
    perl -ne 'print if (!m#^.{46}DEFAULT  UND#)' |
    perl -pe 's#^.{31}##; s#^(.{24})...#\1#;' |
    perl -pe 's#^(.{25})(.*)#\2 (\1)#' |
    sort

# switch to hidden / change linking?
# dreal, ccd, picosat, (filib?)
# conex -- patch the BUILD file
# ign math on its way out Dec 1st
# optitrack dtors wtf?
# ThreadPool wtf?
