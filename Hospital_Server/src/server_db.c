#include "server_db.h"
#include <stdio.h>
#include <string.h>

// ================= DB CONFIG =================
// 서버가 실행되는 PC 기준 DB 주소/계정으로 수정하세요.
#define DB_HOST "10.10.14.138"
#define DB_PORT 3306
#define DB_USER "robot_user"
#define DB_PASS "robot_pw"
#define DB_NAME "hospital_db"
// =============================================

static const char *SQL_UPSERT =
    "INSERT INTO robot_status "
    "(name, op_status, battery_percent, is_charging, current_x, current_y) "
    "VALUES (?, ?, ?, ?, ?, ?) "
    "ON DUPLICATE KEY UPDATE "
    "op_status=VALUES(op_status), "
    "battery_percent=VALUES(battery_percent), "
    "is_charging=VALUES(is_charging), "
    "current_x=VALUES(current_x), "
    "current_y=VALUES(current_y)";

int db_open(DBContext *ctx)
{
    if (!ctx) return -1;
    memset(ctx, 0, sizeof(*ctx));

    ctx->conn = mysql_init(NULL);
    if (!ctx->conn) {
        fprintf(stderr, "[DB] mysql_init failed\n");
        return -1;
    }

    my_bool reconnect = 1;
    mysql_options(ctx->conn, MYSQL_OPT_RECONNECT, &reconnect);

    if (!mysql_real_connect(ctx->conn, DB_HOST, DB_USER, DB_PASS, DB_NAME, DB_PORT, NULL, 0)) {
        fprintf(stderr, "[DB] connect error: %s\n", mysql_error(ctx->conn));
        mysql_close(ctx->conn);
        ctx->conn = NULL;
        return -1;
    }

    mysql_set_character_set(ctx->conn, "utf8mb4");

    ctx->upsert_stmt = mysql_stmt_init(ctx->conn);
    if (!ctx->upsert_stmt) {
        fprintf(stderr, "[DB] stmt_init failed\n");
        db_close(ctx);
        return -1;
    }

    if (mysql_stmt_prepare(ctx->upsert_stmt, SQL_UPSERT, (unsigned long)strlen(SQL_UPSERT)) != 0) {
        fprintf(stderr, "[DB] stmt_prepare error: %s\n", mysql_stmt_error(ctx->upsert_stmt));
        db_close(ctx);
        return -1;
    }

    ctx->connected = 1;
    return 0;
}

void db_close(DBContext *ctx)
{
    if (!ctx) return;

    if (ctx->upsert_stmt) {
        mysql_stmt_close(ctx->upsert_stmt);
        ctx->upsert_stmt = NULL;
    }
    if (ctx->conn) {
        mysql_close(ctx->conn);
        ctx->conn = NULL;
    }
    ctx->connected = 0;
}

int db_upsert_robot_status(
    DBContext *ctx,
    const char *name,
    const char *op_status,
    uint32_t battery_percent,
    uint8_t is_charging,
    double current_x,
    double current_y
){
    if (!ctx || !ctx->connected || !ctx->upsert_stmt || !name || !op_status) return -1;

    MYSQL_BIND b[6];
    memset(b, 0, sizeof(b));

    unsigned long name_len = (unsigned long)strlen(name);
    unsigned long st_len   = (unsigned long)strlen(op_status);

    // 1) name
    b[0].buffer_type   = MYSQL_TYPE_STRING;
    b[0].buffer        = (char*)name;
    b[0].buffer_length = name_len;
    b[0].length        = &name_len;

    // 2) op_status
    b[1].buffer_type   = MYSQL_TYPE_STRING;
    b[1].buffer        = (char*)op_status;
    b[1].buffer_length = st_len;
    b[1].length        = &st_len;

    // 3) battery_percent
    b[2].buffer_type = MYSQL_TYPE_LONG;
    b[2].buffer      = &battery_percent;

    // 4) is_charging
    b[3].buffer_type = MYSQL_TYPE_TINY;
    b[3].buffer      = &is_charging;

    // 5) current_x
    b[4].buffer_type = MYSQL_TYPE_DOUBLE;
    b[4].buffer      = &current_x;

    // 6) current_y
    b[5].buffer_type = MYSQL_TYPE_DOUBLE;
    b[5].buffer      = &current_y;

    // 바인드/실행
    if (mysql_stmt_bind_param(ctx->upsert_stmt, b) != 0) {
        fprintf(stderr, "[DB] bind error: %s\n", mysql_stmt_error(ctx->upsert_stmt));
        return -1;
    }

    if (mysql_stmt_execute(ctx->upsert_stmt) != 0) {
        fprintf(stderr, "[DB] execute error: %s\n", mysql_stmt_error(ctx->upsert_stmt));
        return -1;
    }

    // 다음 실행을 위해 reset(선택이지만 권장)
    mysql_stmt_reset(ctx->upsert_stmt);
    return 0;
}
