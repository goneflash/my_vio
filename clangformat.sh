alias clang-format='clang-format-3.6'

clang-format -style=file -i feature_tracker/*.cpp
clang-format -style=file -i feature_tracker/include/*.hpp
clang-format -style=file -i feature_tracker/src/*.cpp
clang-format -style=file -i feature_tracker/test/*.cpp

clang-format -style=file -i camera_model/*.cpp
clang-format -style=file -i camera_model/include/*.hpp
clang-format -style=file -i camera_model/src/*.cpp
clang-format -style=file -i camera_model/test/*.cpp

clang-format -style=file -i imu_integrator/include/*.hpp
clang-format -style=file -i imu_integrator/src/*.cpp

clang-format -style=file -i map_initializer/include/*.hpp
clang-format -style=file -i map_initializer/src/*.cpp
clang-format -style=file -i map_initializer/test/*.cpp

clang-format -style=file -i pnp_estimator/include/*.hpp
clang-format -style=file -i pnp_estimator/src/*.cpp

clang-format -style=file -i mapdata/include/*.hpp
clang-format -style=file -i mapdata/src/*.cpp
clang-format -style=file -i mapdata/test/*.cpp

clang-format -style=file -i multiview_helper/include/*.hpp
clang-format -style=file -i multiview_helper/src/*.cpp

clang-format -style=file -i orientation_tracking/include/*.hpp
clang-format -style=file -i orientation_tracking/src/*.cpp

clang-format -style=file -i simulator/include/*.hpp
clang-format -style=file -i simulator/src/*.cpp
clang-format -style=file -i simulator/*.cpp

clang-format -style=file -i util/include/*.hpp
clang-format -style=file -i util/src/*.cpp
clang-format -style=file -i util/test/*.cpp

clang-format -style=file -i visual_inertial_odometry/include/*.hpp
clang-format -style=file -i visual_inertial_odometry/src/*.cpp
clang-format -style=file -i visual_inertial_odometry/test/*.cpp
clang-format -style=file -i visual_inertial_odometry/*.cpp
