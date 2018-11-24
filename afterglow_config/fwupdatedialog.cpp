#include "fwupdatedialog.h"
#include "ui_fwupdatedialog.h"
#include <QFontDatabase>

FWUpdateDialog::FWUpdateDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::FWUpdateDialog)
{
    ui->setupUi(this);

    // close the dialog when the 'close' button is clicked
    connect(ui->closeButton, SIGNAL(clicked()), SLOT(close()));

    // use a monospaced font
    const QFont fixedFont = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    ui->fwUpdateOutput->setFont(fixedFont);
}

FWUpdateDialog::~FWUpdateDialog()
{
    delete ui;
}

void FWUpdateDialog::addOutput(const QString &text)
{
    this->ui->fwUpdateOutput->append(text);
    this->ui->fwUpdateOutput->repaint();
}

void FWUpdateDialog::setOutput(const QString &text)
{
    this->ui->fwUpdateOutput->setText(text);

    // scroll to the bottom
    QTextCursor c =  this->ui->fwUpdateOutput->textCursor();
    c.movePosition(QTextCursor::End);
    this->ui->fwUpdateOutput->setTextCursor(c);

    this->ui->fwUpdateOutput->repaint();
}
