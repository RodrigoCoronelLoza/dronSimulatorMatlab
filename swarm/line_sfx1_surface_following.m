mex -v CXX=g++ ...
    -IModules/libColibry/Sources/utils ...
    -I../../../Sources ...
    -I../../../Sources/controllers/sfx1 ...
    -I/usr/local/include ...
    -I/usr/lib ...
    -I../../../../eigen ...
    -I../../../../Tools/libvar ...
    -I../../../../Tools/libmaths/ ...
    -DTRACING_ON ...
    -DENABLE_LIBVAR2 ...
    -L../build CXXFLAGS="\$CXXFLAGS -v -std=c++0x -fPIC -DSFX1" ...
    ../../../Sources/controllers/mexes/mex_sfx1_surface_following.cpp ...
    ../../../Sources/controllers/sfx1/sfx1_surface_following.cpp ...
    ../../utils/tracing.cpp ...
    ../../../../Tools/libmaths/maths/orientation.cpp