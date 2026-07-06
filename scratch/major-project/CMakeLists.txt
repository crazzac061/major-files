cmake_minimum_required(VERSION 3.10)

set(scratch_src
    sim/slicing-sim.cc
    sim/nr-mac-scheduler-wpf.cc
)

build_exec(
        EXECNAME slicing-sim
        EXECNAME_PREFIX scratch-major-project_
        SOURCE_FILES "${scratch_src}"
        LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
        EXECUTABLE_DIRECTORY_PATH ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/
)
