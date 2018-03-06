#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QJsonDocument>
#include <QJsonArray>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void gameChanged(int ix);

private:
    void createGameList();
    void prepareLampMatrix();
    void updateGameDesc(int ix);
    void enumSerialPorts();

    Ui::MainWindow *ui;
    void readGames(void);
    QJsonDocument mGamesDoc;
    QJsonArray mGamesList;
};

#endif // MAINWINDOW_H
