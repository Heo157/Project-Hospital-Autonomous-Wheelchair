#ifndef DATABASE_MANAGER_H
#define DATABASE_MANAGER_H

#include <QObject>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QDebug>

class DatabaseManager : public QObject
{
    Q_OBJECT
public:
    // 싱글톤 인스턴스 반환 (어디서든 이 객체 하나만 씀)
    static DatabaseManager& instance();

    // DB 연결 시도 함수
    bool connectToDb();

    // 현재 연결되어 있는지 확인하는 함수 (추가됨)
    bool isOpen();

    // ID/PW가 DB에 있는지 확인하는 함수
    bool checkIdPassword(const QString &id, const QString &pw);

    // 연결 종료 함수
    void closeDb();

private:
    // 외부에서 생성 못하게 숨김
    explicit DatabaseManager(QObject *parent = nullptr);
    ~DatabaseManager();

    QSqlDatabase db;
};

#endif // DATABASE_MANAGER_H
