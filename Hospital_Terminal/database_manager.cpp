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
