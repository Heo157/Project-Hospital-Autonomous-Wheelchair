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
    const char *op_status,           // "RUNNING"/"STOP"/"ERROR"
    uint32_t battery_percent,        // 0~100
    uint8_t  is_charging,            // 0/1
    double current_x,
    double current_y
);

#endif
