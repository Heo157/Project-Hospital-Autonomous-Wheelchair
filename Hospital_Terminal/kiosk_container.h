#ifndef KIOSK_CONTAINER_H
#define KIOSK_CONTAINER_H

#include <QWidget>

QT_BEGIN_NAMESPACE
namespace Ui { class kiosk_container; }
QT_END_NAMESPACE

class kiosk_container : public QWidget
{
    Q_OBJECT

public:
    explicit kiosk_container(QWidget *parent = nullptr);
    ~kiosk_container();

private:
    Ui::kiosk_container *ui;
    QWidget *prevPage = nullptr;
};

#endif // KIOSK_CONTAINER_H
