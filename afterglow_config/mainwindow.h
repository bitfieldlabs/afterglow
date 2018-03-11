/***********************************************************************
 *  afterglow configuration tool:
 *      Copyright (c) 2018 Christoph Schmid
 *
 ***********************************************************************
 *  This file is part of the afterglow pinball LED project:
 *  https://github.com/smyp/afterglow
 *
 *  afterglow is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  afterglow is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with afterglow.
 *  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QJsonDocument>
#include <QJsonArray>
#include <QTableWidgetItem>
#include "serialcommunicator.h"
#include "agconfig.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void gameChanged(int ix);
    void connectAG();
    void loadAG();
    void saveAG();
    void defaultAG();
    void updateTable(int parameter);
    void tableChanged(QTableWidgetItem *item);
    void editSelected();
    void selectByValue();

private:
    void createGameList();
    void prepareLampMatrix();
    void updateGameDesc(int ix);
    void enumSerialPorts();
    void setConnected(bool connected);

    Ui::MainWindow *ui;
    void readGames(void);
    QJsonDocument mGamesDoc;
    QJsonArray mGamesList;
    SerialCommunicator mSerialCommunicator;
    bool mConnected;
    int mAGVersion;
    int mAGCfgVersion;
    AFTERGLOW_CFG_t mCfg;
};

#endif // MAINWINDOW_H
