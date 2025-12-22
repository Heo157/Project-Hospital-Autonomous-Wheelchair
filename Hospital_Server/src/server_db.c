/**
 * ============================================================================
 * @file    server_db.c
 * @brief   MariaDB/MySQL 데이터베이스 연동 모듈
 * @author  Team Hospital
 * @date    2025-12-04
 * @details
 * - 로봇 상태 정보를 DB에 저장/업데이트하는 기능 제공
 * - Prepared Statement 방식으로 안전하고 효율적인 DB 작업 수행
 * - UPSERT 패턴으로 로봇이 처음이면 INSERT, 이미 있으면 UPDATE
 * ============================================================================
 */

#include "server_db.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ============================================================================
 * [데이터베이스 접속 정보]
 * - 실제 운영 환경에서는 설정 파일(config.ini)로 분리하는 것을 권장
 * - 보안상 코드에 비밀번호를 하드코딩하는 것은 좋지 않음
 * ============================================================================ */
#define DB_HOST "10.10.14.138"   // DB 서버 IP 주소 (localhost면 127.0.0.1)
#define DB_PORT 3306             // MariaDB/MySQL 기본 포트 (표준)
#define DB_USER "admin"          // DB 접속 사용자 이름
#define DB_PASS "1234"           // DB 접속 비밀번호
#define DB_NAME "hospital_db"    // 사용할 데이터베이스 이름

/* ============================================================================
 * [UPSERT SQL 쿼리]
 * - UPSERT = INSERT + UPDATE 합성어
 * - 동작 방식:
 *   1) name이 처음이면 → INSERT (새로운 로봇 등록)
 *   2) name이 이미 있으면 → UPDATE (기존 로봇 정보 갱신)
 * - '?' 는 Placeholder로, 실행 시 실제 값으로 치환됨
 * - ON DUPLICATE KEY UPDATE: UNIQUE 제약 위반 시 UPDATE로 전환
 * ============================================================================ */
static const char *SQL_UPSERT =
    "INSERT INTO robot_status "
    // 삽입할 컬럼 목록 (7개)
    "(name, op_status, battery_percent, is_charging, current_x, current_y, current_theta) "
    // 삽입할 값들 (? = 나중에 바인딩할 자리)
    "VALUES (?, ?, ?, ?, ?, ?, ?) "
    // 중복 키(name)가 있으면 아래 UPDATE 실행
    "ON DUPLICATE KEY UPDATE "               
    "op_status=VALUES(op_status), "           // 운영 상태 갱신
    "battery_percent=VALUES(battery_percent), " // 배터리 잔량 갱신
    "is_charging=VALUES(is_charging), "       // 충전 상태 갱신
    "current_x=VALUES(current_x), "           // X 좌표 갱신
    "current_y=VALUES(current_y),"
    "current_theta=VALUES(current_theta)";            // Y 좌표 갱신

/**
 * ============================================================================
 * @brief DB 연결 및 Prepared Statement 초기화
 * @param ctx DB 컨텍스트 구조체 포인터 (연결 정보를 담을 곳)
 * @return 0: 성공, -1: 실패
 * 
 * @details
 * 1) MySQL 클라이언트 라이브러리 초기화
 * 2) DB 서버 연결
 * 3) SQL 쿼리를 미리 컴파일 (Prepared Statement)
 * 
 * [Prepared Statement란?]
 * - SQL 쿼리 구조를 미리 파싱/컴파일해서 메모리에 저장
 * - 실행 시마다 값만 바인딩해서 재사용 → 빠르고 안전
 * - SQL Injection 공격 완벽 차단 (문자열 조작 불가능)
 * ============================================================================
 */
int db_open(DBContext *ctx)
{
    // 입력 검증: NULL 포인터면 에러
    if (!ctx) return -1;
    
    // 구조체 메모리 초기화 (쓰레기값 제거)
    memset(ctx, 0, sizeof(*ctx));

    /* ------------------------------------------------------------------------
     * 1단계: MySQL 클라이언트 핸들 생성
     * - mysql_init(): MySQL 라이브러리 초기화 함수
     * - NULL 전달 시 내부에서 메모리 자동 할당
     * ------------------------------------------------------------------------ */
    ctx->conn = mysql_init(NULL);
    if (!ctx->conn) {
        fprintf(stderr, "[DB] mysql_init failed\n");
        return -1;
    }

    /* ------------------------------------------------------------------------
     * 2단계: 자동 재연결 옵션 설정
     * - 네트워크 끊김이나 타임아웃 발생 시 자동으로 재연결 시도
     * - 장시간 연결 유지하는 서버에 필수
     * ------------------------------------------------------------------------ */
    my_bool reconnect = 1;
    mysql_options(ctx->conn, MYSQL_OPT_RECONNECT, &reconnect);

    /* ------------------------------------------------------------------------
     * 3단계: 실제 DB 서버 연결
     * - mysql_real_connect(): TCP 소켓으로 DB 서버와 연결
     * - 파라미터: (핸들, 호스트, 사용자, 비번, DB명, 포트, 유닉스소켓, 플래그)
     * - 실패 시 mysql_error()로 에러 메시지 확인 가능
     * ------------------------------------------------------------------------ */
    if (!mysql_real_connect(ctx->conn, DB_HOST, DB_USER, DB_PASS, 
                            DB_NAME, DB_PORT, NULL, 0)) {
        fprintf(stderr, "[DB] connect error: %s\n", mysql_error(ctx->conn));
        mysql_close(ctx->conn);
        ctx->conn = NULL;
        return -1;
    }

    /* ------------------------------------------------------------------------
     * 4단계: 문자 인코딩 설정
     * - utf8mb4: 한글, 이모지 등 모든 유니코드 문자 지원
     * - 기본 utf8은 3바이트까지만 지원 (이모지 X)
     * ------------------------------------------------------------------------ */
    mysql_set_character_set(ctx->conn, "utf8mb4");

    /* ------------------------------------------------------------------------
     * 5단계: Prepared Statement 초기화
     * - mysql_stmt_init(): Statement 객체 생성
     * - 이 객체로 SQL 쿼리를 컴파일하고 실행함
     * ------------------------------------------------------------------------ */
    ctx->upsert_stmt = mysql_stmt_init(ctx->conn);
    if (!ctx->upsert_stmt) {
        fprintf(stderr, "[DB] stmt_init failed\n");
        db_close(ctx);
        return -1;
    }

    /* ------------------------------------------------------------------------
     * 6단계: SQL 쿼리 준비 (컴파일)
     * - mysql_stmt_prepare(): SQL 문법 검사 및 실행 계획 생성
     * - 한 번만 준비하면 여러 번 실행 가능 (성능 향상)
     * - '?' 위치를 파악해서 나중에 값을 바인딩할 준비
     * ------------------------------------------------------------------------ */
    if (mysql_stmt_prepare(ctx->upsert_stmt, SQL_UPSERT, 
                          (unsigned long)strlen(SQL_UPSERT)) != 0) {
        fprintf(stderr, "[DB] stmt_prepare error: %s\n", 
                mysql_stmt_error(ctx->upsert_stmt));
        db_close(ctx);
        return -1;
    }

    // 연결 성공 플래그 설정
    ctx->connected = 1;
    return 0;
}

/**
 * ============================================================================
 * @brief DB 연결 종료 및 리소스 해제
 * @param ctx DB 컨텍스트 구조체 포인터
 * 
 * @details
 * - Prepared Statement 닫기
 * - DB 연결 닫기
 * - 메모리 누수 방지를 위해 반드시 호출 필요
 * ============================================================================
 */
void db_close(DBContext *ctx)
{
    if (!ctx) return;

    // Prepared Statement 리소스 해제
    if (ctx->upsert_stmt) {
        mysql_stmt_close(ctx->upsert_stmt);
        ctx->upsert_stmt = NULL;  // 댕글링 포인터 방지
    }
    
    // DB 연결 종료
    if (ctx->conn) {
        mysql_close(ctx->conn);
        ctx->conn = NULL;  // 댕글링 포인터 방지
    }
    
    // 연결 상태 플래그 초기화
    ctx->connected = 0;
}

/**
 * ============================================================================
 * @brief 로봇 상태 정보를 DB에 저장/업데이트 (UPSERT)
 * @param ctx          DB 컨텍스트
 * @param name         로봇 이름 (예: "wc1", "wc2") - UNIQUE KEY
 * @param op_status    운영 상태 ("RUNNING", "STOP", "ERROR")
 * @param battery_percent  배터리 잔량 (0~100)
 * @param is_charging  충전 중 여부 (0: 아님, 1: 충전중)
 * @param current_x    현재 X 좌표 (미터 단위)
 * @param current_y    현재 Y 좌표 (미터 단위)
 * @return 0: 성공, -1: 실패
 * 
 * @details
 * [동작 방식]
 * 1) name이 DB에 없으면 → INSERT (새 로봇 등록)
 * 2) name이 이미 있으면 → UPDATE (정보 갱신)
 * 
 * [바인딩 과정]
 * SQL의 '?'를 실제 값으로 치환하는 과정
 * - 타입 안정성: 각 값의 타입을 명시 (문자열, 정수, 실수 등)
 * - SQL Injection 방지: 문자열을 직접 쿼리에 넣지 않음
 * ============================================================================
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
){
    /* ------------------------------------------------------------------------
     * 입력값 검증
     * - ctx: DB 컨텍스트가 유효한지
     * - connected: DB에 연결되어 있는지
     * - upsert_stmt: Prepared Statement가 준비되었는지
     * - name, op_status: 필수 문자열이 NULL이 아닌지
     * ------------------------------------------------------------------------ */
    if (!ctx || !ctx->connected || !ctx->upsert_stmt || !name || !op_status) {
        return -1;
    }

    /* ------------------------------------------------------------------------
     * MYSQL_BIND 구조체 배열 준비
     * - SQL의 6개 '?'에 각각 대응하는 값 정보를 담음
     * - 각 원소는 (타입, 데이터 주소, 길이) 정보를 포함
     * ------------------------------------------------------------------------ */
    MYSQL_BIND b[7];
    memset(b, 0, sizeof(b));  // 구조체 초기화 (중요!)

    // 문자열 길이 미리 계산 (바인딩 시 필요)
    unsigned long name_len = (unsigned long)strlen(name);
    unsigned long st_len   = (unsigned long)strlen(op_status);

    /* ========================================================================
     * 바인딩 1: name (문자열)
     * - VALUES의 첫 번째 '?'에 해당
     * - 예: "wc1", "wc2" 등 로봇 이름
     * ======================================================================== */
    b[0].buffer_type   = MYSQL_TYPE_STRING;  // 타입: 문자열
    b[0].buffer        = (char*)name;        // 데이터가 있는 메모리 주소
    b[0].buffer_length = name_len;           // 버퍼 크기
    b[0].length        = &name_len;          // 실제 데이터 길이 포인터

    /* ========================================================================
     * 바인딩 2: op_status (문자열)
     * - VALUES의 두 번째 '?'에 해당
     * - 예: "RUNNING", "STOP", "ERROR"
     * ======================================================================== */
    b[1].buffer_type   = MYSQL_TYPE_STRING;
    b[1].buffer        = (char*)op_status;
    b[1].buffer_length = st_len;
    b[1].length        = &st_len;

    /* ========================================================================
     * 바인딩 3: battery_percent (부호 없는 정수)
     * - VALUES의 세 번째 '?'에 해당
     * - MYSQL_TYPE_LONG: 4바이트 정수 (int32)
     * - 예: 0, 50, 100
     * ======================================================================== */
    b[2].buffer_type = MYSQL_TYPE_LONG;
    b[2].buffer      = &battery_percent;  // 주소만 전달 (복사 안 함)

    /* ========================================================================
     * 바인딩 4: is_charging (작은 정수, boolean)
     * - VALUES의 네 번째 '?'에 해당
     * - MYSQL_TYPE_TINY: 1바이트 정수 (int8)
     * - 예: 0 (충전 안 함), 1 (충전 중)
     * ======================================================================== */
    b[3].buffer_type = MYSQL_TYPE_TINY;
    b[3].buffer      = &is_charging;

    /* ========================================================================
     * 바인딩 5: current_x (실수, 좌표)
     * - VALUES의 다섯 번째 '?'에 해당
     * - MYSQL_TYPE_DOUBLE: 8바이트 부동소수점 (double)
     * - 예: 1.5, -3.2, 0.0
     * ======================================================================== */
    b[4].buffer_type = MYSQL_TYPE_DOUBLE;
    b[4].buffer      = &current_x;

    /* ========================================================================
     * 바인딩 6: current_y (실수, 좌표)
     * - VALUES의 여섯 번째 '?'에 해당
     * - MYSQL_TYPE_DOUBLE: 8바이트 부동소수점
     * ======================================================================== */
    b[5].buffer_type = MYSQL_TYPE_DOUBLE;
    b[5].buffer      = &current_y;

    /* ========================================================================
     * 바인딩 7: current_theta (실수, 방향)
     * - VALUES의 일곱 번째 '?'에 해당
     * - MYSQL_TYPE_DOUBLE: 8바이트 부동소수점
     * ======================================================================== */
    b[6].buffer_type = MYSQL_TYPE_DOUBLE;
    b[6].buffer      = &current_theta;

    /* ------------------------------------------------------------------------
     * Statement에 바인딩 정보 등록
     * - mysql_stmt_bind_param(): 준비된 Statement에 값 연결
     * - 이제 '?'들이 실제 값으로 치환될 준비 완료
     * ------------------------------------------------------------------------ */
    if (mysql_stmt_bind_param(ctx->upsert_stmt, b) != 0) {
        fprintf(stderr, "[DB] bind error: %s\n", 
                mysql_stmt_error(ctx->upsert_stmt));
        return -1;
    }

    /* ------------------------------------------------------------------------
     * 쿼리 실행
     * - mysql_stmt_execute(): 바인딩된 값으로 SQL 실행
     * - 실제로 DB에 데이터가 저장/업데이트됨
     * - INSERT 또는 UPDATE가 내부적으로 결정됨
     * ------------------------------------------------------------------------ */
    if (mysql_stmt_execute(ctx->upsert_stmt) != 0) {
        fprintf(stderr, "[DB] execute error: %s\n", 
                mysql_stmt_error(ctx->upsert_stmt));
        return -1;
    }

    /* ------------------------------------------------------------------------
     * Statement 초기화 (다음 실행을 위한 준비)
     * - mysql_stmt_reset(): 바인딩 정보 지우고 재사용 가능 상태로
     * - 메모리 효율: Statement 객체를 재생성 안 하고 재사용
     * ------------------------------------------------------------------------ */
    mysql_stmt_reset(ctx->upsert_stmt);
    
    return 0;  // 성공
}


/**
 * @brief 1. 우선순위 공식에 따라 대기 호출 1개 조회
 * * [정렬 기준]
 * 1. p.is_emergency DESC : 응급 환자(1) 우선
 * 2. d.base_priority DESC : 질병 점수가 높은 순 (뇌졸중 > 골절 > 타박상)
 * 3. c.call_time ASC     : 먼저 호출한 순서
 * * IFNULL/COALESCE: 환자 정보가 없으면 0점 처리하여 맨 뒤로 보냄
 */
int db_get_priority_call(DBContext *ctx, int *call_id, char *start_loc, char *caller_name) {
    char query[1024];

    // 복잡한 쿼리이므로 snprintf 대신 문자열 상수로 작성하거나 안전하게 복사
    // 주의: call_queue의 caller_name과 patient_info의 name을 조인 조건으로 사용
    const char *sql = 
        "SELECT c.call_id, c.start_loc, c.caller_name "
        "FROM call_queue c "
        "LEFT JOIN patient_info p ON c.caller_name = p.name "
        "LEFT JOIN disease_types d ON p.disease_code = d.disease_code "
        "WHERE c.is_dispatched = 0 "
        "ORDER BY "
        "  COALESCE(p.is_emergency, 0) DESC, "
        "  COALESCE(d.base_priority, 0) DESC, "
        "  c.call_time ASC "
        "LIMIT 1";

    if (mysql_query(ctx->conn, sql)) {
        fprintf(stderr, "[DB] Select Priority Call Error: %s\n", mysql_error(ctx->conn));
        return -1;
    }

    MYSQL_RES *res = mysql_store_result(ctx->conn);
    if (!res) return -1;

    MYSQL_ROW row = mysql_fetch_row(res);
    int found = 0;
    
    if (row) {
        *call_id = atoi(row[0]);
        // 안전한 문자열 복사
        if(row[1]) strncpy(start_loc, row[1], 63); else start_loc[0] = '\0';
        if(row[2]) strncpy(caller_name, row[2], 63); else caller_name[0] = '\0';
        
        // Null termination 보장
        start_loc[63] = '\0';
        caller_name[63] = '\0';
        
        found = 1;
    }

    mysql_free_result(res);
    return found; // 1: 찾음, 0: 대기열 없음
}

/**
 * @brief 2. 'WAITING' 상태인 로봇 하나 찾기
 * 조건: 
 * 1) op_status가  'WAITING' (놀고 있는 상태)
 * 2) 배터리가 20% 이상 (작업 수행 가능)
 */
int db_get_available_robot(DBContext *ctx, char *robot_name) {
    if (!ctx || !ctx->conn) return -1;

    // [수정] 오직 'WAITING' 상태인 로봇만 조회
    const char *query = "SELECT name FROM robot_status "
                        "WHERE op_status = 'WAITING' "
                        "AND battery_percent > 20 "
                        "LIMIT 1";

    if (mysql_query(ctx->conn, query)) {
        fprintf(stderr, "[DB] Select Robot Error: %s\n", mysql_error(ctx->conn));
        return -1;
    }

    MYSQL_RES *res = mysql_store_result(ctx->conn);
    if (!res) return -1;

    MYSQL_ROW row = mysql_fetch_row(res);
    int found = 0;
    
    if (row) {
        strncpy(robot_name, row[0], 63);
        robot_name[63] = '\0';
        found = 1;
    }
    
    mysql_free_result(res);
    return found;
}

/**
 * @brief 3. 장소 이름("정형외과")을 좌표(x, y)로 변환
 * map_location 테이블에서 조회
 */
int db_get_location_coords(DBContext *ctx, const char *loc_name, double *x, double *y) {
    if (!ctx || !ctx->conn) return -1;

    char query[512];
    // map_location 테이블이 존재해야 합니다.
    snprintf(query, sizeof(query), 
             "SELECT x, y FROM map_location WHERE location_name = '%s'", loc_name);

    if (mysql_query(ctx->conn, query)) {
        // 테이블이 없거나 쿼리 오류 시
        fprintf(stderr, "[DB] Get Location Error: %s\n", mysql_error(ctx->conn));
        return -1;
    }

    MYSQL_RES *res = mysql_store_result(ctx->conn);
    if (!res) return -1;

    MYSQL_ROW row = mysql_fetch_row(res);
    int found = 0;
    
    if (row) {
        *x = atof(row[0]);
        *y = atof(row[1]);
        found = 1;
    } else {
        // 맵에 없는 장소일 경우
        fprintf(stderr, "[DB Warning] Unknown location: %s\n", loc_name);
    }
    
    mysql_free_result(res);
    return found;
}

/**
 * @brief 4. 로봇에게 작업 할당 및 대기열 상태 변경 (트랜잭션급 처리)
 * 1) robot_status 업데이트 (order=1, goal 설정)
 * 2) call_queue 업데이트 (배차완료 처리)
 */
int db_assign_job_to_robot(DBContext *ctx, const char *robot_name, int call_id, double x, double y, const char *caller) {
    if (!ctx || !ctx->conn) return -1;

    char query[1024];
    
    // 1) 로봇 상태 업데이트: order=1, goal 좌표 설정, 상태=BUSY
    // 주의: `order`는 SQL 예약어이므로 백틱(`)으로 감싸야 합니다.
    snprintf(query, sizeof(query),
             "UPDATE robot_status SET `order` = 1, goal_x = %f, goal_y = %f, "
             "op_status = 'BUSY', who_called = '%s' WHERE name = '%s'",
             x, y, caller, robot_name);
    
    if (mysql_query(ctx->conn, query)) {
        fprintf(stderr, "[Dispatch] Update Robot Failed: %s\n", mysql_error(ctx->conn));
        return -1;
    }

    // 2) 대기열 상태 업데이트: is_dispatched=1
    // 필요하다면 eta 컬럼에 '이동중' 등의 텍스트를 넣을 수 있음
    snprintf(query, sizeof(query),
             "UPDATE call_queue SET is_dispatched = 1, eta = '이동중' WHERE call_id = %d",
             call_id);
             
    if (mysql_query(ctx->conn, query)) {
        fprintf(stderr, "[Dispatch] Update Queue Failed: %s\n", mysql_error(ctx->conn));
        return -1;
    }

    return 0; // 성공
}

/**
 * @brief [메인 로직] 배차 사이클 실행
 * 배차 관리자 프로세스가 이 함수를 반복 호출합니다.
 */
void db_process_dispatch_cycle(DBContext *ctx) {
    int call_id;
    char start_loc[64], caller[64];
    char robot_name[64];
    double goal_x, goal_y;

    // 1. 우선순위 높은 대기 호출 확인 (에러(-1)가 아니고 찾았을 때(1)만 진입)
    if (db_get_priority_call(ctx, &call_id, start_loc, caller) == 1) {
        
        // 2. 가용한 로봇 확인
        // [수정] 단순히 if(...)라고 쓰면 -1(에러)도 true가 됩니다. == 1을 꼭 붙여주세요.
        if (db_get_available_robot(ctx, robot_name) == 1) {
            
            // 3. 좌표 변환
            if (db_get_location_coords(ctx, start_loc, &goal_x, &goal_y) == 1) {
                
                printf("[Dispatch] Matched! Call #%d (%s) -> Robot '%s' (Goal: %.2f, %.2f)\n",
                       call_id, start_loc, robot_name, goal_x, goal_y);

                // 4. 배차 실행
                db_assign_job_to_robot(ctx, robot_name, call_id, goal_x, goal_y, caller);
                
            } else {
                printf("[Dispatch] Error: Coordinates not found for location '%s'\n", start_loc);
            }
        } 
    }
}

