INCLUDE_DIRECTORIES(
        inc/
)

file(GLOB VHACD_SRCS "src/*.cpp")
file(GLOB VHACD_HDRS "inc/*.h")


ADD_LIBRARY(VHACD_local ${VHACD_SRCS} ${VHACD_HDRS})

