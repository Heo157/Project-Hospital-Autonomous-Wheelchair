#include "database_manager.h"

// 싱글톤 패턴: 어디서든 DatabaseManager::instance()로 이 객체를 불러올 수 있음
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

bool DatabaseManager::connectToDb()
{
    // 이미 문이 열려있으면(연결됨) 다시 열 필요 없음
    if(db.isOpen()) {
        return true;
    }

    // MariaDB(MySQL) 드라이버 사용
    db = QSqlDatabase::addDatabase("QMYSQL");

    // [라즈베리파이 DB 접속 정보] -> "병원 정문 열쇠"
    db.setHostName("10.10.14.138");  // 라즈베리파이 IP
    db.setPort(3306);                // 포트 번호
    db.setDatabaseName("hospital");  // DB 이름

    // ※ 주의: 아래는 라즈베리파이 DB 자체에 접속하기 위한 계정입니다.
    // 사용자 로그인(의사/간호사) ID와는 다릅니다.
    db.setUserName("root");          // DB 접속 아이디
    db.setPassword("1234");          // DB 접속 비밀번호

    if (!db.open()) {
        qDebug() << "DB Connection Error:" << db.lastError().text();
        return false;
    }

    qDebug() << "Connected to Database (10.10.14.138)";
    return true;
}

bool DatabaseManager::isOpen()
{
    return db.isOpen();
}

void DatabaseManager::closeDb()
{
    if (db.isOpen()) {
        db.close();
        qDebug() << "DB Connection Closed";
    }
}

// 사용자가 입력한 ID/PW가 맞는지 확인하는 함수
bool DatabaseManager::checkIdPassword(const QString &id, const QString &pw)
{
    // DB 연결이 끊겨있으면 재연결 시도
    if (!db.isOpen()) {
        if (!connectToDb()) return false;
    }

    QSqlQuery query;
    // SQL Injection 해킹 방지를 위해 입력값을 안전하게 포장(binding)해서 넣음
    // "users 테이블에서 아이디가 (입력ID)이고, 비번이 (입력PW)인 사람이 몇 명이니?"
    query.prepare("SELECT count(*) FROM users WHERE id = :id AND pw = :pw");
    query.bindValue(":id", id);
    query.bindValue(":pw", pw);

    if (query.exec()) {
        if (query.next()) {
            int count = query.value(0).toInt();
            // 1명이라도 있으면 로그인 성공!
            return (count > 0);
        }
    } else {
        qDebug() << "Query Execution Error:" << query.lastError().text();
    }

    return false;
}
