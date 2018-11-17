ADD_LIBRARY( vocore SHARED
			config.cpp
			camera.cpp
			frame.cpp
			mappoint3d.cpp
			localmap.cpp
			g2o_types.cpp
			visualodometry.cpp
)
			
TARGET_LINK_LIBRARIES( vocore
					${THIRD_PARTY_LIBS})
#TODO MAP2D