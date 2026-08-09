/***********************************************************************
 *  afterglow configuration tool:
 *      Copyright (c) 2018 Christoph Schmid
 *
 ***********************************************************************
 *  This file is part of the afterglow pinball LED project:
 *  https://github.com/smyp/afterglow
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ***********************************************************************/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QJsonDocument>
#include <QJsonArray>
#include <QTableWidgetItem>
#include <QTimer>
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
    void ticker(const QString &text, const QColor &c, int weight, bool replaceLastLine=false);
    ~MainWindow();

private slots:
    void gameChanged(int ix);
    void connectAG();
    void loadAG();
    void saveAG();
    void defaultAG();
    void recDownload();
    void updateTable(int parameter);
    void tableChanged(QTableWidgetItem *item);
    void editSelected();
    void selectByValue();
    void enumSerialPorts();
    void fetchGameList();
    void updateFW();
    void glow();

private:
    void createGameList();
    void prepareLampMatrix();
    void updateGameDesc(int ix);
    void setConnected(bool connected);
    void initData();

    Ui::MainWindow *ui;
    void readGames(void);
    QJsonDocument mGamesDoc;
    QJsonArray mGamesList;
    SerialCommunicator mSerialCommunicator;
    bool mConnected;
    int mAGVersion;
    int mAGCfgVersion;
    AFTERGLOW_CFG_t mCfg;
    QTimer mTimer;
    QTimer mGlowTimer;
    int mGlowRow;
    int mGlowCol;
    int mGlowFrame;
};

#endif // MAINWINDOW_H
