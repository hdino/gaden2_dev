INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD

SOURCES += \
    $$PWD/src/environment_model_plane.cpp \
    $$PWD/src/filament_model.cpp \
    $$PWD/src/gas_source.cpp \
    $$PWD/src/gas_source_filament_model.cpp \
    $$PWD/src/helpers/interpolation.cpp \
    $$PWD/src/logger.cpp \
    $$PWD/src/sensors/open_path.cpp \
    $$PWD/src/simulation_element.cpp \
    $$PWD/src/simulator.cpp \
    $$PWD/src/wind_models/farrell.cpp \
    $$PWD/src/wind_models/farrell_noise.cpp \
    $$PWD/src/wind_models/lattice_boltzmann_2d.cpp

HEADERS += \
    $$PWD/include/gaden2/environment_model.hpp \
    $$PWD/include/gaden2/environment_model_plane.hpp \
    $$PWD/include/gaden2/filament.hpp \
    $$PWD/include/gaden2/filament_model.hpp \
    $$PWD/include/gaden2/gas_dispersion_model.hpp \
    $$PWD/include/gaden2/gas_source.hpp \
    $$PWD/include/gaden2/gas_source_filament_model.hpp \
    $$PWD/include/gaden2/gases.hpp \
    $$PWD/include/gaden2/helpers/ideal_gas.hpp \
    $$PWD/include/gaden2/helpers/interpolation.hpp \
    $$PWD/include/gaden2/logger.hpp \
    $$PWD/include/gaden2/sensors/open_path.hpp \
    $$PWD/include/gaden2/simulation_element.hpp \
    $$PWD/include/gaden2/simulator.hpp \
    $$PWD/include/gaden2/wind_models/farrell.hpp \
    $$PWD/include/gaden2/wind_models/farrell_noise.hpp \
    $$PWD/include/gaden2/wind_models/lattice_boltzmann_2d.hpp \
    $$PWD/include/gaden2/wind_models/wind_model_base.hpp
