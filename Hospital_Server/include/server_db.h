#ifndef SERVER_DB_H
#define SERVER_DB_H

#include <stdint.h>
#include <mariadb/mysql.h>   // libmariadb-dev 설치 필요

typedef struct {
    MYSQL *conn;
    MYSQL_STMT *upsert_stmt;
    int connected;
} DBContext;

/**
 * @brief DB 연결 + UPSERT prepared statement 준비
 * @return 0 성공, -1 실패
 */
int db_open(DBContext *ctx);

/**
 * @brief DB 리소스 해제
 */
void db_close(DBContext *ctx);

/**
 * @brief robot_status 최신 상태 UPSERT
 * @return 0 성공, -1 실패
 */
int db_upsert_robot_status(
    DBContext *ctx,
    const char *name,
    const char *op_status,
    uint32_t battery_percent,
    uint8_t is_charging,
    double current_x,
    double current_y,
    double current_theta     // 추가
);

/**
 * @brief 로봇 이름으로 order 및 goal 조회
 * @return 0: 성공(goal 있음), 1: order=0, -1: 에러
 */
int db_get_order_and_goal(
    DBContext *ctx,
    const char *name,
    double *goal_x,
    double *goal_y
);

/**
 * @brief order를 0으로 리셋
 */
int db_reset_order(DBContext *ctx, const char *name);

/**
 * @brief 로봇 이름이 DB에 존재하는지 확인
 * @return 1: 존재, 0: 없음, -1: 에러
 */
int db_check_robot_exists(DBContext *ctx, const char *name);

#endif
