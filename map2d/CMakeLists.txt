ADD_LIBRARY( map2d SHARED
			map2D.cpp
			map2Dsence.cpp
)
			
TARGET_LINK_LIBRARIES( map2d
					${THIRD_PARTY_LIBS})