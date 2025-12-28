/**
 * ============================================================================
 * @file    main.c
 * @brief   ROS 2 - Qt 중계 서버 (Multi-Process Architecture)
 * @details
 * 1. Main Process: TCP 연결 수락 (Accept) 및 자식 프로세스 분기
 * 2. Child 1 (Dispatch): 배차 알고리즘 수행 (DB Polling)
 * 3. Child 2 (RobotMgr): 로봇 프로세스 자동 실행 관리 (DB Sync)
 * 4. Child N (Client): 개별 클라이언트와 1:1 통신 및 DB 업데이트
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>

// 사용자 정의 헤더
#include "server_db.h"
#include "robot_manager.h"      // 로봇 프로세스 관리자 (run_robot_manager_loop)
#include "../include/common_defs.h"

// ============================================================================
// [1] 유틸리티 함수
// ============================================================================

// 좀비 프로세스 처리 핸들러 (자식 프로세스 종료 시 리소스 회수)
void handle_sigchld(int sig) {
    (void)sig; // 경고 방지
    // 종료된 자식 프로세스가 있으면 모두 회수 (Non-blocking)
    while (waitpid(-1, NULL, WNOHANG) > 0);
}

// 상태 코드를 문자열로 변환 (로그용)
const char* get_state_str(uint8_t state_id) {
    switch (state_id) {
        case 0: return "WAITING";
        case 1: return "HEADING";
        case 2: return "BOARDING";
        case 3: return "RUNNING";
        case 4: return "STOP";
        case 5: return "ARRIVED";
        case 6: return "EXITING";
        case 7: return "CHARGING";
        case 99: return "ERROR";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// [2] 클라이언트 1:1 담당 프로세스 (Handle Client)
// ============================================================================
void handle_client(int client_sock, struct sockaddr_in client_addr) {
    PacketHeader header;
    char buffer[MAX_BUFFER_SIZE];
    ssize_t read_len;
    
    // 로봇 이름 저장용 (MSG_LOGIN_REQ 수신 시 설정됨)
    char robot_name[64] = {0};

    // [DB 연결] 각 클라이언트 프로세스마다 독립적인 DB 연결 생성
    DBContext db;
    int db_ok = (db_open(&db) == 0);
    if (!db_ok) {
        fprintf(stderr, "[ClientHandler] DB Open Failed. Updates will be skipped.\n");
    }

    printf("[Server] Client Connected: %s\n", inet_ntoa(client_addr.sin_addr));

    while (1) {
        // 1. 헤더 읽기 (4바이트)
        read_len = recv(client_sock, &header, sizeof(PacketHeader), MSG_WAITALL);
        if (read_len <= 0) {
            printf("[Server] Client disconnected (%s)\n", robot_name);
            break;
        }

        // 2. 매직 넘버 검사
        if (header.magic != MAGIC_NUMBER) {
            printf("[Warning] Invalid Magic Number: 0x%02X\n", header.magic);
            continue;
        }

        // 3. 페이로드 읽기
        if (header.payload_len > 0) {
            read_len = recv(client_sock, buffer, header.payload_len, MSG_WAITALL);
            if (read_len != header.payload_len) {
                printf("[Error] Payload mismatch\n");
                break;
            }
        }

        // 4. 메시지 처리
        switch (header.msg_type) {
            case MSG_LOGIN_REQ:
                // 로봇 이름 등록
                if (header.payload_len > 0 && header.payload_len < sizeof(robot_name)) {
                    memcpy(robot_name, buffer, header.payload_len);
                    robot_name[header.payload_len] = '\0';
                    printf("[Login] Robot Identified: '%s'\n", robot_name);
                }
                break;

            case MSG_ROBOT_STATE:
                if (header.payload_len == sizeof(RobotStateData)) {
                    RobotStateData* st = (RobotStateData*)buffer;
                    
                    // DB가 연결되어 있고, 로봇 이름이 식별된 경우에만 처리
                    if (db_ok && robot_name[0] != '\0') {
                        // A. 로봇 상태 DB 업데이트 (UPSERT)
                        const char* op_str = get_state_str(st->is_moving);
                        
                        db_upsert_robot_status(&db, robot_name, op_str, 
                                               st->battery_level, 0,
                                               (double)st->current_x, 
                                               (double)st->current_y, 
                                               (double)st->theta);

                        // ---------------------------------------------------------
                        // B. [수정됨] 배차된 새 명령(Order)이 있는지 확인 (Start + Goal)
                        // ---------------------------------------------------------
                        int new_order = 0;
                        double sx = 0, sy = 0; // 출발지 (Start)
                        double gx = 0, gy = 0; // 목적지 (Goal)

                        // 주의: db_check_new_order 함수도 server_db.c에서 매개변수를 5개 받도록 수정되어야 합니다.
                        if (db_check_new_order(&db, robot_name, &new_order, &sx, &sy, &gx, &gy) == 1) {
                            
                            printf(">> [Command] Order %d for %s: Start(%.2f, %.2f) -> Goal(%.2f, %.2f)\n", 
                                   new_order, robot_name, sx, sy, gx, gy);

                            // B-1. 명령 패킷 생성 (MSG_ASSIGN_GOAL)
                            PacketHeader res_header;
                            GoalAssignData goal_data; // 구조체 정의가 수정되어 있어야 함 (20바이트)

                            res_header.magic = MAGIC_NUMBER;
                            res_header.device_type = DEVICE_ADMIN_QT; // 서버 권한
                            res_header.msg_type = MSG_ASSIGN_GOAL;
                            res_header.payload_len = sizeof(GoalAssignData);

                            // B-2. 데이터 채우기 (Order + Start + Goal)
                            goal_data.order   = new_order;
                            goal_data.start_x = (float)sx;
                            goal_data.start_y = (float)sy;
                            goal_data.goal_x  = (float)gx;
                            goal_data.goal_y  = (float)gy;

                            // B-3. 전송
                            send(client_sock, &res_header, sizeof(PacketHeader), 0);
                            send(client_sock, &goal_data, sizeof(GoalAssignData), 0);

                            // B-4. DB 주문 상태 초기화 (중복 전송 방지)
                            db_reset_order(&db, robot_name);
                        }
                    }
                }
                break;

            // 기타 메시지들 (로그만 출력)
            case MSG_MOVE_FORWARD: printf("[%s] CMD: Forward\n", robot_name); break;
            case MSG_MOVE_BACKWARD: printf("[%s] CMD: Backward\n", robot_name); break;
            case MSG_STOP: printf("[%s] CMD: Stop\n", robot_name); break;
            
            default:
                // printf("Unknown Msg Type: 0x%02X\n", header.msg_type);
                break;
        }
    }

    // 종료 처리
    if (db_ok) db_close(&db);
    close(client_sock);
    exit(0); // 자식 프로세스 종료
}

// ============================================================================
// [3] 배차 관리자 프로세스 (Dispatch Manager)
// ============================================================================
void run_dispatch_manager() {
    DBContext db_ctx;
    
    // 독립적인 DB 연결
    if (db_open(&db_ctx) != 0) {
        fprintf(stderr, "[Dispatch] DB Connection Failed. Terminating.\n");
        exit(1);
    }

    printf("[Dispatch Manager] Service Started (PID: %d). Monitoring Queue...\n", getpid());

    while (1) {
        // 1. 배차 로직 수행 (server_db.c에 정의됨)
        //    (우선순위 호출 조회 -> 가용 로봇 조회 -> 매칭 -> DB 업데이트)
        db_process_dispatch_cycle(&db_ctx);

        // 2. 폴링 주기 (1초) - 너무 빠르면 DB 부하, 너무 느리면 반응성 저하
        sleep(1);
    }

    db_close(&db_ctx);
    exit(0);
}


// ============================================================================
// [4] 메인 함수 (Main Entry)
// ============================================================================
int main() {
    int server_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    // 1. 좀비 프로세스 방지 핸들러 등록
    signal(SIGCHLD, handle_sigchld);

    // 2. 소켓 생성
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock == -1) { perror("socket"); return 1; }

    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 3. 바인딩
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY); // 모든 IP 허용
    server_addr.sin_port = htons(SERVER_PORT);

    if (bind(server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        perror("bind"); return 1;
    }

    // 4. 리슨
    if (listen(server_sock, 10) == -1) { perror("listen"); return 1; }

    printf("\n==================================================\n");
    printf("   ROS 2 - Qt Central Server Started (Port: %d)\n", SERVER_PORT);
    printf("==================================================\n");

    // ========================================================================
    // Fork 1: 배차 관리자 실행 (Dispatch Manager)
    // ========================================================================
    pid_t dispatch_pid = fork();
    if (dispatch_pid == 0) {
        close(server_sock);     // 자식은 리슨 소켓 불필요
        run_dispatch_manager(); // 무한 루프 진입
        exit(0);
    }
    printf("[System] Dispatch Manager Spawned (PID: %d)\n", dispatch_pid);

    // ========================================================================
    // Fork 2: 로봇 프로세스 관리자 실행 (Robot Process Manager)
    // ========================================================================
    pid_t robot_mgr_pid = fork();
    if (robot_mgr_pid == 0) {
        close(server_sock);       // 자식은 리슨 소켓 불필요
        run_robot_manager_loop(); // robot_manager.c 의 함수 호출 (무한 루프)
        exit(0);
    }
    printf("[System] Robot Process Manager Spawned (PID: %d)\n", robot_mgr_pid);


    // ========================================================================
    // Main Loop: 클라이언트 접속 대기
    // ========================================================================
    while (1) {
        // 클라이언트 접속 수락 (Blocking)
        client_sock = accept(server_sock, (struct sockaddr*)&client_addr, &addr_len);
        if (client_sock == -1) {
            continue; // 에러 발생 시 다시 대기
        }

        // Fork 3: 접속한 클라이언트 처리 (1:1 Handler)
        pid_t client_pid = fork();

        if (client_pid == 0) {
            // [자식 프로세스]
            close(server_sock); // 리슨 소켓 닫기 (메인 프로세스가 가지고 있음)
            handle_client(client_sock, client_addr); // 통신 시작 (종료 시 exit)
            // handle_client 내부에서 exit(0) 하므로 여기까지 안 옴
        } 
        else if (client_pid > 0) {
            // [부모 프로세스]
            close(client_sock); // 핸들러 소켓 닫기 (자식에게 넘김)
            // 다시 루프 상단으로 이동하여 다음 접속 대기
        } 
        else {
            perror("Fork Client Failed");
        }
    }

    close(server_sock);
    return 0;
}