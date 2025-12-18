#include "database_manager.h"

DatabaseManager& DatabaseManager::instance()
{
    static DatabaseManager instance;
    return instance;
}

DatabaseManager::DatabaseManager(QObject *parent) : QObject(parent)
{
}

DatabaseManager::~DatabaseManager()
{
    closeDb();
}

bool DatabaseManager::deleteRobot(int id)
{
    QSqlQuery query;
    // SQL: robot_status 테이블에서 해당 id를 가진 행 삭제
    query.prepare("DELETE FROM robot_status WHERE robot_id = :id");
    query.bindValue(":id", id);

    if (!query.exec()) {
        qDebug() << "Delete robot failed:" << query.lastError();
        return false;
    }
    return true;
}

bool DatabaseManager::connectToDb()
{
    if(db.isOpen()) return true;

    db = QSqlDatabase::addDatabase("QMYSQL");
    // 라즈베리파이 IP (환경에 맞게 수정)
    db.setHostName("10.10.14.138");
    db.setPort(3306);
    db.setDatabaseName("hospital_db");
    
    // DB 접속 계정
    db.setUserName("root");
    db.setPassword("1234");

    if (!db.open()) {
        qDebug() << "DB Connect Error:" << db.lastError().text();
        return false;
    }
    return true;
}

bool DatabaseManager::isOpen()
{
    return db.isOpen();
}

void DatabaseManager::closeDb()
{
    if (db.isOpen()) db.close();
}

// [핵심 변경] 헤더와 똑같이 함수 이름과 반환형을 맞춰줍니다.
QString DatabaseManager::getUserRole(const QString &id, const QString &pw)
{
    if (!db.isOpen()) {
        if (!connectToDb()) return "";
    }

    QSqlQuery query;
    // role 컬럼도 같이 가져오도록 쿼리 수정
    query.prepare("SELECT role FROM users WHERE id = :id AND pw = :pw");
    query.bindValue(":id", id);
    query.bindValue(":pw", pw);

    if (query.exec()) {
        if (query.next()) {
            // 조회된 역할(role)을 문자열로 반환 (예: "admin", "medical")
            return query.value(0).toString();
        }
    }
    
    return ""; // 로그인 실패 시 빈 문자열 반환
}

bool DatabaseManager::addRobot(const QString &robot_name, const QString &ip, double x, double y, double theta)
{
    if (!db.isOpen()) {
        if (!connectToDb()) return false;
    }

    QSqlQuery query;

    // [중요 수정] 'order' (X) -> `order` (O)
    // 작은따옴표가 아니라 키보드 숫자 1 왼쪽에 있는 ` (백틱) 입니다.
    query.prepare("INSERT INTO robot_status "
                      "(name, ip_address, current_x, current_y, current_theta, op_status) "
                      "VALUES "
                      "(:name, :ip, :x, :y, :theta, 'STOP')");

    // 값 바인딩
    //query.bindValue(":id", id);
    query.bindValue(":name", robot_name);
    query.bindValue(":ip", ip);
    query.bindValue(":x", x);
    query.bindValue(":y", y);
    query.bindValue(":theta", theta);

    if (!query.exec()) {
        // 에러 확인용 로그
        qDebug() << "Insert Robot Error:" << query.lastError().text();
        return false;
    }
    return true;
}

QSqlQuery DatabaseManager::getRobotStatusQuery()
{
    // DB가 닫혀있으면 연결 시도
    if (!db.isOpen()) {
        if(!connectToDb()) {
            qDebug() << "DB is not open!";
            return QSqlQuery(); // 빈 쿼리 반환
        }
    }

    // [수정] 쿼리 객체 생성 시 db를 인자로 전달 (가장 안전함)
    QSqlQuery query(db);

    // 쿼리 실행
    if (!query.exec("SELECT * FROM robot_status")) {
        // 에러가 났다면 로그 출력
        qDebug() << "Select Query Error:" << query.lastError().text();
    } else {
        qDebug() << "Select Query Executed. Active:" << query.isActive();
    }

    return query;
}
