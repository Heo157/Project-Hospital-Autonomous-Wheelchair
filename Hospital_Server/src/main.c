/**
 * ============================================================================
 * @file    main.c
 * @brief   C 언어 기반 중계 서버 (Protocol Aware)
 * @details common_defs.h에 정의된 프로토콜을 해석하여 로그를 출력하는 서버
 * Qt 클라이언트와 ROS 로봇 사이의 통신을 중계하기 위한 기초 서버입니다.
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>      // close(), fork() 등을 사용하기 위해 필요
#include <arpa/inet.h>   // sockaddr_in, inet_ntoa() 등 네트워크 주소 관련
#include <sys/socket.h>  // socket(), bind(), accept() 등 소켓 핵심 함수
#include <sys/types.h>
#include <sys/wait.h>    // waitpid()를 사용하기 위해 필요 (좀비 프로세스 처리)
#include <signal.h>      // signal() 함수 사용
#include "server_db.h"

 // 우리가 정의한 공통 헤더 (패킷 구조체, 상수 등)
 // 이 파일은 클라이언트(Qt/ROS)와 서버가 반드시 동일한 버전을 가지고 있어야 합니다.
#include "../include/common_defs.h"

/**
 * @brief 좀비 프로세스(Zombie Process) 처리 핸들러
 * * [개념 설명]
 * fork()를 통해 생성된 자식 프로세스가 할 일을 다 하고 종료(exit)하면,
 * 부모 프로세스가 그 종료 상태를 확인해줄 때까지 커널에 '좀비' 상태로 남아 리소스를 차지합니다.
 * 이를 방지하기 위해 자식이 죽었다는 신호(SIGCHLD)가 오면 waitpid로 청소해줍니다.
 */
void handle_sigchld(int sig) {
    (void)sig; // 컴파일러의 '사용되지 않은 변수' 경고 방지

    // waitpid(-1, ...): 종료된 임의의 자식 프로세스를 기다림
    // WNOHANG: 자식 프로세스가 아직 종료되지 않았으면 기다리지 않고 바로 리턴 (Non-blocking)
    while (waitpid(-1, NULL, WNOHANG) > 0);
}

/**
 * @brief 클라이언트 1:1 전담 마크 함수 (핵심 로직)
 * * [동작 원리]
 * 이 함수는 fork()된 자식 프로세스에서 실행됩니다.
 * 연결된 하나의 클라이언트와 계속 대화하며 패킷을 주고받습니다.
 */
void handle_client(int client_sock, struct sockaddr_in client_addr) {
    PacketHeader header;           // 패킷의 머리(Header)를 담을 구조체
    char buffer[MAX_BUFFER_SIZE];  // 패킷의 몸통(Payload)을 담을 버퍼
    ssize_t read_len;              // 읽어들인 바이트 수

    //------- 추가 ------
    // 로봇 이름 저장 (MSG_LOGIN_REQ에서 받아서 저장)
    char robot_name[64] = {0};

    // DB 연결 (각 클라이언트 프로세스마다 독립적으로 연결)
    DBContext db;
    int db_ok = (db_open(&db) == 0);
    if (!db_ok) {
        fprintf(stderr, "[DB] open failed. DB update will be skipped.\n");
    }
    //-----------------

    // 접속한 클라이언트의 IP 주소를 문자열로 변환하여 출력
    printf("[Server] New client connected: %s\n", inet_ntoa(client_addr.sin_addr));

    while (1) {
        // ====================================================================
        // 1단계: 헤더 읽기 (고정 길이 4바이트)
        // ====================================================================
        // MSG_WAITALL: TCP는 데이터가 스트림(물줄기)처럼 흐르기 때문에,
        // 4바이트를 요청해도 네트워크 상황에 따라 1~2바이트만 올 수 있습니다.
        // 이 옵션은 4바이트가 꽉 찰 때까지 기다렸다가 리턴하라고 지시합니다.
        read_len = recv(client_sock, &header, sizeof(PacketHeader), MSG_WAITALL);

        if (read_len <= 0) {
            // 0이면 클라이언트가 정상 종료(close), -1이면 에러 발생
            printf("[Server] Client disconnected or error.\n");
            break; // while 루프 탈출 -> 함수 종료 -> 프로세스 종료
        }

        // ====================================================================
        // 2단계: 유효성 검사 (Magic Number)
        // ====================================================================
        // 데이터가 중간에 깨졌거나, 우리 프로토콜을 모르는 이상한 놈이 접속했는지 확인합니다.
        // common_defs.h에 정의된 MAGIC_NUMBER(0xAB)와 다르면 무시합니다.
        if (header.magic != MAGIC_NUMBER) {
            printf("[Warning] Invalid magic number: 0x%02X\n", header.magic);
            continue; // 이번 패킷은 버리고 다음 패킷 대기
        }

        // ====================================================================
        // 3단계: 페이로드(Payload) 읽기
        // ====================================================================
        // 헤더에 적힌 payload_len(데이터 길이)만큼 추가로 데이터를 읽어옵니다.
        if (header.payload_len > 0) {
            read_len = recv(client_sock, buffer, header.payload_len, MSG_WAITALL);

            // 헤더에는 10바이트 보낸다고 했는데, 실제로는 덜 왔다면? -> 에러 처리
            if (read_len != header.payload_len) {
                printf("[Error] Payload read mismatch.\n");
                break;
            }
        }

        // ====================================================================
        // 4단계: 메시지 해석 및 처리
        // ====================================================================
        // 여기까지 왔으면 완전한 패킷 하나(Header + Payload)를 손에 쥔 것입니다.
        printf("[Packet] Device: 0x%02X | Type: 0x%02X | Len: %d -> ",
            header.device_type, header.msg_type, header.payload_len);

        switch (header.msg_type) {
        case MSG_LOGIN_REQ:
            printf("Login Request received.\n");
            //------- 추가 ------
            // 로봇 이름 등록 (로그인)
            if (header.payload_len > 0 && header.payload_len < sizeof(robot_name)) {
                memcpy(robot_name, buffer, header.payload_len);
                robot_name[header.payload_len] = '\0';  // Null 종료
                printf("Login Request: Robot name = '%s'\n", robot_name);
            } else {
                printf("Login Request received (invalid payload).\n");
            }
            //-----------------
            // 나중에는 여기서 DB를 조회하거나 로그인 승인 패킷을 보내야 함
            break;

        case MSG_MOVE_FORWARD:
            printf("Command: MOVE FORWARD\n");
            // 나중에는 이 명령을 연결된 ROS 로봇 프로세스에게 전달해야 함 (IPC 필요)
            break;

        case MSG_MOVE_BACKWARD:
            printf("Command: MOVE BACKWARD\n");
            break;

        case MSG_STOP:
            printf("Command: STOP!\n");
            break;

        case MSG_MOVE_TO_GOAL:
            // 페이로드를 WaypointData 구조체 모양으로 해석(Casting)
            if (header.payload_len == sizeof(WaypointData)) {
                WaypointData* wp = (WaypointData*)buffer;
                printf("Goal: X=%.2f, Y=%.2f, Theta=%.2f\n", wp->x, wp->y, wp->theta);
            }
            break;

        case MSG_ROBOT_STATE:
            // 페이로드를 RobotStateData 구조체 모양으로 해석
            if (header.payload_len == sizeof(RobotStateData)) {
                RobotStateData* st = (RobotStateData*)buffer;
                printf("State: Battery=%d%%, Pos=(%.2f, %.2f), Theta=%.2f\n",
                    st->battery_level, st->current_x, st->current_y, st->theta);

                //------- 추가 ------
                // 로봇이 DB에 등록되어 있는지 확인
                if (db_ok) {
                    if (robot_name[0] == '\0') {
                        printf("[DB] robot_name empty. Send MSG_LOGIN_REQ first.\n");
                        break;
                    }
                    
                    // 등록되어 있으면 상태 업데이트
                    const char *op = (st->is_moving ? "RUNNING" : "STOP");
                    int batt_i = st->battery_level;
                    if (batt_i < 0) batt_i = 0;
                    if (batt_i > 100) batt_i = 100;
                    
                    db_upsert_robot_status(&db, robot_name, op, (uint32_t)batt_i, 0,
                                        (double)st->current_x, 
                                        (double)st->current_y,
                                        (double)st->theta);  // theta 추가
                }
                // ========================================================
                // [추가된 부분] DB에서 명령 확인 후 로봇에게 전송 (Push)
                // ========================================================
                double gx = 0.0, gy = 0.0;
                
                // DB 조회: "이 로봇한테 내려진 명령(order>=1) 있어?"
                if (db_get_order_and_goal(&db, robot_name, &gx, &gy) == 0) {
                    printf("[Server] Order detected for '%s' -> Go to (%.2f, %.2f)\n", 
                            robot_name, gx, gy);

                    // 1. 보낼 패킷 데이터 준비
                    WaypointData wp;
                    wp.x = (float)gx;
                    wp.y = (float)gy;
                    wp.theta = 0.0f; // 이동 명령엔 보통 좌표만 주므로 각도는 0

                    // 2. 헤더 준비
                    PacketHeader cmd_header;
                    cmd_header.magic = MAGIC_NUMBER;
                    cmd_header.device_type = DEVICE_ADMIN_QT; // 서버(혹은 관리자)가 보냄
                    cmd_header.msg_type = MSG_MOVE_TO_GOAL;   // 0x15
                    cmd_header.payload_len = sizeof(WaypointData);

                    // 3. 전송 (Header + Payload)
                    // send() 함수는 소켓이 유효할 때만
                    if (send(client_sock, &cmd_header, sizeof(cmd_header), 0) > 0) {
                        send(client_sock, &wp, sizeof(wp), 0);
                        
                        printf("[Server] Command sent to robot.\n");

                        // 4. DB 정리: 명령을 보냈으니 order를 0으로 리셋 (중복 전송 방지)
                        db_reset_order(&db, robot_name);
                    }
                }
                // ========================================================
                //-----------------
            }
            break;

        default:
            printf("Unknown Message Type.\n");
            break;
        }
    }

    // DB 연결 종료 (메모리 누수 방지)
    if (db_ok) {
        db_close(&db);
    }

    // while 루프를 빠져나오면 소켓을 닫고 프로세스를 종료합니다.
    close(client_sock);
    exit(0); // 자식 프로세스 종료 (이때 SIGCHLD 신호가 부모에게 날아감)
}

int main() {
    int server_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    // [Signal 등록] 자식 프로세스가 죽으면 handle_sigchld 함수 실행
    signal(SIGCHLD, handle_sigchld);

    // 1. 소켓 생성 (IPv4, TCP 스트림 방식)
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock == -1) {
        perror("socket error");
        return 1;
    }

    // [중요 옵션] "Address already in use" 에러 방지
    // 서버를 강제 종료하고 바로 다시 켤 때, 포트가 아직 점유 중이라며 에러가 나는 것을 막아줍니다.
    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 2. 주소 설정 (IP와 Port 지정)
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY); // 내 컴퓨터의 모든 IP로 접속 허용
    server_addr.sin_port = htons(SERVER_PORT);       // common_defs.h에 정의된 포트 (8080)

    // 3. Bind (소켓에 주소표 붙이기)
    if (bind(server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        perror("bind error");
        return 1;
    }

    // 4. Listen (접속 대기열 생성, 최대 5명 대기 가능)
    if (listen(server_sock, 5) == -1) {
        perror("listen error");
        return 1;
    }

    printf("=== Middle Server Started on Port %d ===\n", SERVER_PORT);

    while (1) {
        // 5. Accept (클라이언트가 올 때까지 블로킹/대기)
        // 누군가 접속하면 새로운 소켓(client_sock)을 만들어줍니다.
        client_sock = accept(server_sock, (struct sockaddr*)&client_addr, &addr_len);
        if (client_sock == -1) {
            continue; // 에러 나면 다시 대기
        }

        // 6. Fork (프로세스 복제 - 병렬 처리)
        // 부모(서버 본체)는 계속 새로운 손님을 받아야 하므로,
        // 지금 들어온 손님은 복제인간(자식 프로세스)에게 맡깁니다.
        pid_t pid = fork();

        if (pid == 0) {
            // [자식 프로세스 영역]
            // 자식은 리슨 소켓(입구)이 필요 없습니다.
            close(server_sock);
            // 클라이언트 담당 함수 실행 (여기서 무한루프 돌며 대화함)
            handle_client(client_sock, client_addr);
        }
        else if (pid > 0) {
            // [부모 프로세스 영역]
            // 부모는 클라이언트와 대화할 필요가 없습니다(자식이 하니까).
            // 따라서 클라이언트 소켓 핸들만 닫고, 다시 accept()하러 루프 위로 올라갑니다.
            close(client_sock);
        }
        else {
            perror("fork error");
        }
    }

    // 서버 종료 시 리슨 소켓 닫기 (사실상 도달하지 않음)
    close(server_sock);
    return 0;
}