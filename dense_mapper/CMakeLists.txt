ADD_LIBRARY( densemapper SHARED
			densematcher.cpp
			densemapper.cpp
			densefusion.cpp
)
			
TARGET_LINK_LIBRARIES( densemapper
					${THIRD_PARTY_LIBS})