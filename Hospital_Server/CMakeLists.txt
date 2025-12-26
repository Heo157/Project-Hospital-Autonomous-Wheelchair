cmake_minimum_required(VERSION 3.10)
project(Hospital_Server C)

# C 표준 설정 (C99)
set(CMAKE_C_STANDARD 99)

# 헤더 파일 경로 설정
include_directories(${CMAKE_SOURCE_DIR}/include)

# 소스 파일 목록 지정
# (server_utils.c가 실제로 없다면 이 줄에서 지워야 에러가 안 납니다)
set(SOURCES
    src/main.c
    src/server_db.c
    src/robot_manager.c
    # src/server_utils.c  <-- 만약 이 파일이 없으면 주석 처리하세요!
)

# 실행 파일 생성
add_executable(hospital_server ${SOURCES})

# ------------------------------------------------------------------
# [1. DB 라이브러리 설정]
# MariaDB/MySQL 커넥터 찾기 (libmariadb 또는 libmysqlclient)
# ------------------------------------------------------------------
find_package(PkgConfig REQUIRED)

# Ubuntu에서는 보통 libmariadb-dev 또는 libmysqlclient-dev를 설치함
# 둘 중 하나를 찾도록 설정 (우선순위: libmariadb)
pkg_check_modules(MARIADB QUIET libmariadb)
if(NOT MARIADB_FOUND)
    pkg_check_modules(MARIADB REQUIRED mysqlclient)
endif()

target_include_directories(hospital_server PRIVATE ${MARIADB_INCLUDE_DIRS})
target_link_directories(hospital_server PRIVATE ${MARIADB_LIBRARY_DIRS})

# ------------------------------------------------------------------
# [2. 필수 시스템 라이브러리 링크]
# pthread: DB 클라이언트가 내부적으로 스레드를 쓰므로 필수
# m: 수학 라이브러리 (math.h), DB 라이브러리가 의존할 수 있음
# ------------------------------------------------------------------
target_link_libraries(hospital_server PRIVATE ${MARIADB_LIBRARIES} pthread m)

# ------------------------------------------------------------------
# [3. 핵심! Python 스크립트 자동 복사]
# 이유: C 코드가 execlp("./tcp_bridge.py", ...) 로 실행함.
# 빌드된 실행 파일(hospital_server)이 있는 곳에 파이썬 파일도 같이 있어야 함.
# ------------------------------------------------------------------
# 가정: tcp_bridge.py가 소스 코드의 src 폴더 안에 있다고 가정
file(COPY ${CMAKE_SOURCE_DIR}/src/tcp_bridge.py 
     DESTINATION ${CMAKE_BINARY_DIR})

# 만약 tcp_bridge.py가 프로젝트 최상위 루트에 있다면 아래 줄 주석 해제 후 위 코드 주석 처리
# file(COPY ${CMAKE_SOURCE_DIR}/tcp_bridge.py DESTINATION ${CMAKE_BINARY_DIR})