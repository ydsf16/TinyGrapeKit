## Build third party -- Pangolin
cd third_party
cd Pangolin
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8

## Build third party -- GeographicLib
cd ../../
cd GeographicLib
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8

## Build TinyGrapeKit library.
cd ../../../
cd library
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make install -j8

## Build VWO-MSCKF
cd ../../
cd app/VWO_MSCKF
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8