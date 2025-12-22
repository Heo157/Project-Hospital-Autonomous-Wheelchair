#ifndef SERVER_DB_H
#define SERVER_DB_H

#include <stdint.h>
#include <mariadb/mysql.h>   // 시스템에 따라 <mysql/mysql.h> 일 수도 있음 (libmariadb-dev)

/**
 * @brief DB 연결 컨텍스트 구조체
 */
typedef struct {
    MYSQL *conn;             // MySQL 연결 핸들
    MYSQL_STMT *upsert_stmt; // 로봇 상태 업데이트용 Prepared Statement
    int connected;           // 연결 여부 플래그 (0: 끊김, 1: 연결됨)
} DBContext;

/* ============================================================================
 * [1] DB 연결 및 해제 (Core)
 * ============================================================================ */

/**
 * @brief DB 연결 및 Prepared Statement 초기화
 * @return 0: 성공, -1: 실패
 */
int db_open(DBContext *ctx);

/**
 * @brief DB 연결 종료 및 리소스 해제
 */
void db_close(DBContext *ctx);


/* ============================================================================
 * [2] 로봇 상태 관리 (Robot Status)
 * ============================================================================ */

/**
 * @brief 로봇 상태 정보를 DB에 저장/업데이트 (UPSERT)
 * @param ctx DB 컨텍스트
 * @param name 로봇 이름 (PK)
 * @param op_status 상태 문자열 (RUNNING, STOP, WAITING ...)
 * @param battery_percent 배터리 잔량
 * @param is_charging 충전 중 여부
 * @param current_x 현재 X 좌표
 * @param current_y 현재 Y 좌표
 * @param current_theta 현재 방향(각도)
 * @return 0: 성공, -1: 실패
 */
int db_upsert_robot_status(
    DBContext *ctx,
    const char *name,
    const char *op_status,
    uint32_t battery_percent,
    uint8_t is_charging,
    double current_x,
    double current_y,
    double current_theta
);

/**
 * @brief 로봇 이름이 DB에 존재하는지 확인
 * @return 1: 존재함, 0: 없음, -1: 에러
 */
int db_check_robot_exists(DBContext *ctx, const char *name);


/* ============================================================================
 * [3] 배차 알고리즘 (Dispatch Logic)
 * - 배차 관리자 프로세스(run_dispatch_manager)에서 주로 사용
 * ============================================================================ */

/**
 * @brief 1. 우선순위 공식에 따라 대기 호출 1개 조회
 * (응급 > 질병위중도 > 대기시간 순)
 * @return 1: 찾음, 0: 대기열 없음, -1: 에러
 */
int db_get_priority_call(DBContext *ctx, int *call_id, char *start_loc, char *caller_name);

/**
 * @brief 2. 가용한(WAITING/STOP) 로봇 하나 찾기
 * @return 1: 찾음, 0: 없음, -1: 에러
 */
int db_get_available_robot(DBContext *ctx, char *robot_name);

/**
 * @brief 3. 장소 이름("정형외과")을 좌표(x, y)로 변환
 * @return 1: 성공, 0: 못 찾음(DB에 없음), -1: 에러
 */
int db_get_location_coords(DBContext *ctx, const char *loc_name, double *x, double *y);

/**
 * @brief 4. 로봇에게 작업 할당 및 대기열 상태 변경 (트랜잭션급 처리)
 * - robot_status 업데이트 (order=1, goal 설정)
 * - call_queue 업데이트 (배차완료 처리)
 */
int db_assign_job_to_robot(DBContext *ctx, const char *robot_name, int call_id, double x, double y, const char *caller);

/**
 * @brief [메인 로직] 배차 사이클 실행
 * 위 함수들을 조합하여 실제 배차를 수행하는 함수
 */
void db_process_dispatch_cycle(DBContext *ctx);


/* ============================================================================
 * [4] 로봇 명령 확인 및 제어 (Robot Control)
 * - 로봇이 자신에게 할당된 명령을 확인하거나 완료 처리할 때 사용
 * ============================================================================ */

/**
 * @brief 로봇 이름으로 현재 할당된 order 및 goal 좌표 조회
 * @param goal_x, goal_y 결과값을 저장할 포인터
 * @return 0: 성공(order=1), 1: 대기중(order=0), -1: 에러
 */
int db_get_order_and_goal(
    DBContext *ctx,
    const char *name,
    double *goal_x,
    double *goal_y
);

/**
 * @brief 로봇 임무 완료 처리 (order를 0으로 리셋)
 * @return 0: 성공, -1: 실패
 */
int db_reset_order(DBContext *ctx, const char *name);

#endif // SERVER_DB_H