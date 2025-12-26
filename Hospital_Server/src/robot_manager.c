/**
 * @file    robot_manager.c
 * @brief   로봇 프로세스(Python Bridge) 자동 실행 관리자 (Pure C)
 */

#include "robot_manager.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <stdbool.h> // C99 bool 타입 지원

#define MAX_ROBOTS 100

// 내부 관리용 구조체
typedef struct {
    char name[64];
    pid_t pid;
    bool alive; // DB와 비교를 위한 체크 플래그
} RobotProc;

// 전역 변수로 관리 (이 프로세스 내에서만 유효)
static RobotProc process_list[MAX_ROBOTS];
static int proc_count = 0;

// [내부 함수] 프로세스 리스트에서 이름으로 인덱스 찾기
static int find_proc_index(const char* name) {
    for (int i = 0; i < proc_count; i++) {
        if (strncmp(process_list[i].name, name, 64) == 0) {
            return i;
        }
    }
    return -1;
}

// [내부 함수] 파이썬 브리지 실행 (Fork & Exec)
static pid_t spawn_python_bridge(const char* robot_name) {
    pid_t pid = fork();
    
    if (pid < 0) {
        perror("[RobotManager] Fork failed");
        return -1;
    }

    if (pid == 0) {
        // [자식 프로세스]
        // 실행 명령: python3 ./tcp_bridge.py [robot_name]
        // 주의: tcp_bridge.py 경로가 정확해야 함
        execlp("python3", "python3", "./tcp_bridge.py", robot_name, (char *)NULL);
        
        // 실패 시
        perror("[RobotManager] execlp failed");
        exit(1);
    }
    
    return pid; // 부모에게 PID 반환
}

// [핵심 로직] DB와 프로세스 목록 동기화
void sync_processes_with_db(DBContext *ctx) {
    // 1. 모든 프로세스를 '사망 예정(false)'으로 마킹
    for (int i = 0; i < proc_count; i++) {
        process_list[i].alive = false;
    }

    // 2. DB 조회 (robot_status 테이블에 있는 로봇들은 실행되어야 함)
    // 필요시 WHERE 조건 추가 (예: WHERE active=1)
    if (mysql_query(ctx->conn, "SELECT name FROM robot_status")) {
        fprintf(stderr, "[RobotManager] DB Query Fail: %s\n", mysql_error(ctx->conn));
        return;
    }
    
    MYSQL_RES *res = mysql_store_result(ctx->conn);
    if (!res) return;

    MYSQL_ROW row;
    while ((row = mysql_fetch_row(res))) {
        if (!row[0]) continue;
        char *db_name = row[0];
        
        int idx = find_proc_index(db_name);

        if (idx != -1) {
            // 이미 실행 중임 -> 생존 마킹
            process_list[idx].alive = true;
        } else {
            // 실행 중이지 않음 -> 새로 실행 (Spawn)
            if (proc_count < MAX_ROBOTS) {
                printf("[RobotManager] ✨ New Robot found: '%s'. Spawning...\n", db_name);
                pid_t new_pid = spawn_python_bridge(db_name);
                
                if (new_pid > 0) {
                    strncpy(process_list[proc_count].name, db_name, 63);
                    process_list[proc_count].pid = new_pid;
                    process_list[proc_count].alive = true;
                    proc_count++;
                }
            } else {
                fprintf(stderr, "[RobotManager] Max robots limit reached.\n");
            }
        }
    }
    mysql_free_result(res);

    // 3. DB에 없는(alive=false) 프로세스 종료 (Kill)
    int remaining_count = 0;
    RobotProc temp_list[MAX_ROBOTS]; // 살아남은 애들을 옮겨담을 임시 배열

    for (int i = 0; i < proc_count; i++) {
        if (!process_list[i].alive) {
            printf("[RobotManager] 🚫 Robot '%s' removed from DB. Killing PID %d...\n", 
                   process_list[i].name, process_list[i].pid);
            kill(process_list[i].pid, SIGTERM);
            // waitpid는 메인 루프나 시그널 핸들러가 처리하도록 둠
        } else {
            // 살아남은 프로세스 유지
            temp_list[remaining_count++] = process_list[i];
        }
    }

    // 리스트 갱신
    memcpy(process_list, temp_list, sizeof(RobotProc) * remaining_count);
    proc_count = remaining_count;
}

// [공개 함수] 메인 루프
void run_robot_manager_loop() {
    DBContext db_ctx;
    
    if (db_open(&db_ctx) != 0) {
        fprintf(stderr, "[RobotManager] DB Connect Failed.\n");
        exit(1);
    }

    printf("[RobotManager] Service Started (PID: %d)\n", getpid());

    while (1) {
        // 1. 동기화
        sync_processes_with_db(&db_ctx);
        
        // 2. 좀비 프로세스 정리 (자식이 죽었을 때 즉시 회수)
        while (waitpid(-1, NULL, WNOHANG) > 0);
        
        // 3. 주기 대기 (2초)
        sleep(2);
    }
    
    db_close(&db_ctx);
}