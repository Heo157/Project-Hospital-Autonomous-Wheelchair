#ifndef KIOSK_LOGIN_H
#define KIOSK_LOGIN_H

#include <QWidget>

namespace Ui {
class kiosk_login;
}

class kiosk_login : public QWidget
{
    Q_OBJECT

public:
    explicit kiosk_login(QWidget *parent = nullptr);
    ~kiosk_login();

private:
    Ui::kiosk_login *ui;
};

#endif // KIOSK_LOGIN_H
