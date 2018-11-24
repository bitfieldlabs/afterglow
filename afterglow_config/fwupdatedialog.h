#ifndef FWUPDATEDIALOG_H
#define FWUPDATEDIALOG_H

#include <QDialog>

namespace Ui {
class FWUpdateDialog;
}

class FWUpdateDialog : public QDialog
{
    Q_OBJECT

public:
    explicit FWUpdateDialog(QWidget *parent = nullptr);
    ~FWUpdateDialog();

    void addOutput(const QString &text);
    void setOutput(const QString &text);

private:
    Ui::FWUpdateDialog *ui;
};

#endif // FWUPDATEDIALOG_H
