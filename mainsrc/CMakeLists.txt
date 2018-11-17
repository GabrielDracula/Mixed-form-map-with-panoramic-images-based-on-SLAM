ADD_LIBRARY( IMGReader KITTIReader.cpp ladybugimg.cpp )
TARGET_LINK_LIBRARIES( IMGReader ${THIRD_PARTY_LIBS})

ADD_EXECUTABLE( mixmap mixmap.cpp )
TARGET_LINK_LIBRARIES( mixmap vocore densemapper IMGReader map2d)

ADD_EXECUTABLE( test test.cpp )
TARGET_LINK_LIBRARIES( test vocore densemapper IMGReader map2d)