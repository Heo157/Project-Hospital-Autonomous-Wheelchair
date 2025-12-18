#ifndef KIOSK_WHEEL_H
#define KIOSK_WHEEL_H

#include <QWidget>

namespace Ui {
class kiosk_wheel;
}

class kiosk_wheel : public QWidget
{
    Q_OBJECT

public:
    explicit kiosk_wheel(QWidget *parent = nullptr);
    ~kiosk_wheel();

private:
    Ui::kiosk_wheel *ui;
};

#endif // KIOSK_WHEEL_H
