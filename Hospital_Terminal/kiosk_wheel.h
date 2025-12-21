#ifndef KIOSK_WHEEL_H
#define KIOSK_WHEEL_H

#include <QWidget>
#include <QString>

namespace Ui {
class kiosk_wheel;
}

class kiosk_wheel : public QWidget
{
    Q_OBJECT

public:
    explicit kiosk_wheel(QWidget *parent = nullptr);
    ~kiosk_wheel();

signals:
    void wheelConfirmed();
    void goBack();

private:
    Ui::kiosk_wheel *ui;
    QString selectedDestination;
};

#endif // KIOSK_WHEEL_H
