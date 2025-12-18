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
    static DatabaseManager& instance();

    bool connectToDb();
    bool isOpen();
    void closeDb();

    // [수정됨] 기존 checkIdPassword 대신 역할(Role)을 반환하는 함수 선언이 필요합니다.
    QString getUserRole(const QString &id, const QString &pw);

    bool addRobot(const QString &robot_name, const QString &ip, double x, double y, double theta);

    QSqlQuery getRobotStatusQuery();
    bool deleteRobot(int id);

private:
    explicit DatabaseManager(QObject *parent = nullptr);
    ~DatabaseManager();

    QSqlDatabase db;
};

#endif // DATABASE_MANAGER_H
