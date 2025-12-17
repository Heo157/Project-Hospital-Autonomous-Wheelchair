cmake_minimum_required(VERSION 3.10)
project(Hospital_Server C)

# C 표준 설정 (C99 또는 C11)
set(CMAKE_C_STANDARD 99)

# [중요] 헤더 파일(.h)들이 있는 경로를 컴파일러에게 알려줌
# ${CMAKE_SOURCE_DIR}는 CMakeLists.txt가 있는 루트 폴더를 의미합니다.
# 따라서 소스 파일이 어디에 있든 include 폴더를 헤더 경로로 인식하게 됩니다.
include_directories(${CMAKE_SOURCE_DIR}/include)

# 소스 파일 목록 지정
set(SOURCES
    src/main.c
    src/server_db.c
    src/server_network.c
    src/server_utils.c
)

# 실행 파일 생성 (이름: hospital_server)
add_executable(hospital_server ${SOURCES})

# 리눅스 소켓/스레드 라이브러리 연결 (필요시 사용)
# target_link_libraries(hospital_server pthread)

# --- 추가 ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(MARIADB REQUIRED libmariadb)

target_include_directories(hospital_server PRIVATE ${MARIADB_INCLUDE_DIRS})
target_link_directories(hospital_server PRIVATE ${MARIADB_LIBRARY_DIRS})
target_link_libraries(hospital_server PRIVATE ${MARIADB_LIBRARIES})
#-------------