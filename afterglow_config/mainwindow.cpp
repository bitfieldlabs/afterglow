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

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "fwupdate.h"
#include "filedownload.h"
#include <QAction>
#include <QFile>
#include <QJsonObject>
#include <QSerialPortInfo>
#include <QThread>
#include <QNetworkRequest>
#include <QUrl>
#include <QMessageBox>

// interval for port enumeration [ms]
#define ENUMERATION_INTERVAL 2000

// github games list URL
#define GITHUB_GAMES_LIST_URL "https://raw.githubusercontent.com/smyp/afterglow/master/afterglow_config/games.json"

// local games list file name
#define LOCAL_GAMES_LIST_FILE "games.json"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    mTimer(this)
{
    ui->setupUi(this);
    ui->statusBar->showMessage("Not connected");
    ui->statusBar->setStyleSheet("");

    // deactivate some GUI elements by default
    ui->saveButton->setEnabled(false);
    ui->loadButton->setEnabled(false);
    ui->connectButton->setEnabled(false);
    ui->defaultButton->setEnabled(false);
    ui->updateFWButton->setEnabled(false);
    ui->lampMatrix->setEnabled(false);

    // format the lamp matrix list
    prepareLampMatrix();

    // read the games list
    readGames();

    // populate the games list pulldown
    createGameList();
    if (ui->gameSelection->count() > 0)
    {
        updateGameDesc(ui->gameSelection->currentIndex());
    }

    // add the parameters
    ui->parameterSelection->addItem("Glow duration");
    ui->parameterSelection->addItem("Brightness");

    // enumerate the serial ports
    enumSerialPorts();
    mConnected = false;

    // connect everything
    connect(ui->parameterSelection, SIGNAL(currentIndexChanged(int)), SLOT(updateTable(int)));
    connect(ui->gameSelection, SIGNAL(currentIndexChanged(int)), SLOT(gameChanged(int)));
    connect(ui->gameListFetchButton, SIGNAL(clicked()), SLOT(fetchGameList()));
    connect(ui->connectButton, SIGNAL(clicked()), SLOT(connectAG()));
    connect(ui->loadButton, SIGNAL(clicked()), SLOT(loadAG()));
    connect(ui->saveButton, SIGNAL(clicked()), SLOT(saveAG()));
    connect(ui->defaultButton, SIGNAL(clicked()), SLOT(defaultAG()));
    connect(ui->updateFWButton, SIGNAL(clicked()), SLOT(updateFW()));
    connect(ui->lampMatrix, SIGNAL(itemChanged(QTableWidgetItem*)), SLOT(tableChanged(QTableWidgetItem*)));
    connect(&mTimer, SIGNAL(timeout()), SLOT(enumSerialPorts()));

    initData();

    // start the port enumeration timer
    mTimer.start(ENUMERATION_INTERVAL);
}

MainWindow::~MainWindow()
{
    mSerialCommunicator.disconnect();
    delete ui;
}

void MainWindow::initData()
{
    mAGVersion = 0;
    mAGCfgVersion = 0;
    memset(&mCfg, 0, sizeof(mCfg));
}

void MainWindow::readGames(void)
{
    QFile file("games.json");
    if (!file.exists())
    {
        ui->statusBar->showMessage("Games list not present! Put games.json into root folder.");
        ui->statusBar->setStyleSheet("background-color: rgb(255, 255, 0);");
        return;
    }
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QByteArray bla = file.readAll();
    file.close();
    QJsonParseError err;
    mGamesDoc = QJsonDocument::fromJson(bla, &err);
    mGamesList = mGamesDoc.array();
}

void MainWindow::connectAG()
{
    if (mConnected)
    {
        // disconnect
        mSerialCommunicator.disconnect();
        initData();
        prepareLampMatrix();
        updateGameDesc(ui->gameSelection->currentIndex());
        setConnected(false);
    }
    else
    {
        // connect
        setCursor(Qt::WaitCursor);

        // connect to the selected port
        if (!mSerialCommunicator.openPort(ui->serialPortSelection->currentText()))
        {
            QString warning = "Failed to open port ";
            warning += ui->serialPortSelection->currentText();
            ui->statusBar->showMessage(warning);
            ui->statusBar->setStyleSheet("background-color: rgb(255, 0, 0);");
        }
        else
        {
            // the connection will reset the arduino - allow some time for startup
            ui->statusBar->showMessage("Rebooting the arduino...");
            ui->statusBar->setStyleSheet("");
            QThread::sleep(2);

            // poll the afterglow version to verify the connection
            ui->statusBar->showMessage("Polling the afterglow version...");
            ui->statusBar->setStyleSheet("");

            mAGVersion = mSerialCommunicator.pollVersion(&mAGCfgVersion);
            if (mAGVersion != 0)
            {
                QString connectStr = "ConnectedAG to afterglow revision ";
                connectStr += QString::number(mAGVersion, 10);
                connectStr += " cfg ";
                connectStr += QString::number(mAGCfgVersion, 10);
                ui->statusBar->showMessage(connectStr);
                ui->statusBar->setStyleSheet("background-color: rgb(0, 255, 0);");
                setConnected(true);

                // load the current configuration
                loadAG();
            }
            else
            {
                ui->statusBar->showMessage("No afterglow board detected on this port!");
                ui->statusBar->setStyleSheet("background-color: rgb(255, 0, 0);");
            }
        }
        setCursor(Qt::ArrowCursor);
    }
}

void MainWindow::loadAG()
{
    if (mConnected)
    {
        setCursor(Qt::WaitCursor);
        if (mSerialCommunicator.loadCfg(&mCfg))
        {
            ui->statusBar->showMessage("Configuration successfully loaded");
            ui->statusBar->setStyleSheet("background-color: rgb(0, 255, 0);");
        }
        else
        {
            ui->statusBar->showMessage("Configuration poll failed!");
            ui->statusBar->setStyleSheet("background-color: rgb(255, 0, 0);");
        }
        setCursor(Qt::ArrowCursor);

        // update the GUI with the new configuration
        updateTable(ui->parameterSelection->currentIndex());
    }
    else
    {
        ui->statusBar->showMessage("Not connected to afterglow!");
        ui->statusBar->setStyleSheet("background-color: rgb(255, 0, 0);");
    }
}

void MainWindow::defaultAG()
{
    if (mConnected)
    {
        setCursor(Qt::WaitCursor);
        if (mSerialCommunicator.defaultCfg())
        {
            ui->statusBar->showMessage("Configuration successfully reset");
            ui->statusBar->setStyleSheet("background-color: rgb(0, 255, 0);");

            // load the new configuration from AG
            loadAG();
        }
        else
        {
            ui->statusBar->showMessage("Configuration reset failed!");
            ui->statusBar->setStyleSheet("background-color: rgb(255, 0, 0);");
        }
        setCursor(Qt::ArrowCursor);

        // update the GUI with the new configuration
        updateTable(ui->parameterSelection->currentIndex());
    }
    else
    {
        ui->statusBar->showMessage("Not connected to afterglow!");
        ui->statusBar->setStyleSheet("background-color: rgb(255, 0, 0);");
    }
}

void MainWindow::saveAG()
{
    if (mConnected)
    {
        setCursor(Qt::WaitCursor);
        if (mSerialCommunicator.saveCfg(&mCfg))
        {
            ui->statusBar->showMessage("Configuration successfully saved");
            ui->statusBar->setStyleSheet("background-color: rgb(0, 255, 0);");
        }
        else
        {
            ui->statusBar->showMessage("Configuration save failed!");
            ui->statusBar->setStyleSheet("background-color: rgb(255, 0, 0);");
        }
        setCursor(Qt::ArrowCursor);
    }
    else
    {
        ui->statusBar->showMessage("Not connected to afterglow!");
        ui->statusBar->setStyleSheet("background-color: rgb(255, 0, 0);");
    }
}

void MainWindow::gameChanged(int ix)
{
    updateGameDesc(ix);
}

void MainWindow::createGameList()
{
    // delete all existing items first
    ui->gameSelection->clear();

    // populate the list
    QJsonArray::const_iterator it;
    for (it=mGamesList.begin(); it!=mGamesList.end(); it++)
    {
        QJsonObject obj = (*it).toObject();
        QJsonObject::iterator title = obj.find("Title");
        if (title != obj.end())
        {
            // add the title
            ui->gameSelection->addItem((*title).toString());
        }
    }
}

void MainWindow::updateGameDesc(int ix)
{
    // find the description array
    QJsonValue v = mGamesList.at(ix);
    QJsonObject obj = v.toObject();
    QJsonObject::iterator o = obj.find("Lamps");
    if (o != obj.end())
    {
        // update the matrix descriptions
        QJsonArray lamps = (*o).toArray();
        for (int c=0; c<8; c++)
        {
            for (int r=0; r<8; r++)
            {
                QTableWidgetItem *pTWI = ui->lampMatrix->item(r*2, c);
                if (pTWI)
                {
                    pTWI->setText(lamps.at(c*8+r).toString());
                }
            }
        }
    }
}

void MainWindow::prepareLampMatrix()
{
    ui->lampMatrix->setRowCount(16);
    ui->lampMatrix->setColumnCount(8);
    ui->lampMatrix->setWordWrap(true);
    for (int c=0; c<8; c++)
    {
        for (int r=0; r<16; r++)
        {
            if (c==0)
            {
                ui->lampMatrix->setRowHeight(r,(r%2)?20:36);
            }
            if (r%2)
            {
                // glow duration
                QTableWidgetItem *header = new QTableWidgetItem();
                if (header)
                {
                    header->setText(QString::number(r/2+1));
                    ui->lampMatrix->setVerticalHeaderItem(r,header);
                }
                ui->lampMatrix->setItem(r, c, new QTableWidgetItem(""));
                QTableWidgetItem *pWI = ui->lampMatrix->item(r, c);
                if (pWI)
                {
                    pWI->setTextAlignment(Qt::AlignRight);
                }
            }
            else
            {
                // lamp title
                QTableWidgetItem *header = new QTableWidgetItem();
                if (header)
                {
                    header->setText("");
                    ui->lampMatrix->setVerticalHeaderItem(r,header);
                }
                ui->lampMatrix->setItem(r, c, new QTableWidgetItem("Unknown"));
                QTableWidgetItem *pWI = ui->lampMatrix->item(r, c);
                if (pWI)
                {
                    pWI->setFlags(pWI->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
                    pWI->setTextAlignment(Qt::AlignLeft);
                    QFont f= pWI->font();
                    f.setPointSize(8);
                    pWI->setFont(f);
                    pWI->setTextColor(Qt::darkBlue);
                }
            }
        }
    }

    // add a context menu to the table
    ui->lampMatrix->setContextMenuPolicy(Qt::ActionsContextMenu);
    QAction* selectByValueAction = new QAction("Select by value");
    QAction* editSelectedAction = new QAction("Edit selected");
    ui->lampMatrix->addAction(selectByValueAction);
    ui->lampMatrix->addAction(editSelectedAction);
    connect(selectByValueAction, SIGNAL(triggered()), this, SLOT(selectByValue()));
    connect(editSelectedAction, SIGNAL(triggered()), this, SLOT(editSelected()));
}

void MainWindow::setConnected(bool connected)
{
    mConnected = connected;
    ui->loadButton->setEnabled(connected);
    ui->saveButton->setEnabled(connected);
    ui->defaultButton->setEnabled(connected);
    ui->updateFWButton->setEnabled(connected);
    ui->connectButton->setText(connected ? "Disconnect" : "Connect");
}

void MainWindow::enumSerialPorts()
{
    // no enumeration while connected
    if (mConnected == false)
    {
        // clear the current items
        ui->serialPortSelection->clear();

        // add new items
        const auto infos = QSerialPortInfo::availablePorts();
        for (const QSerialPortInfo &info : infos)
        {
            QString s = info.portName();
            ui->serialPortSelection->addItem(s);
        }

        // enable the connect button
        ui->connectButton->setEnabled(ui->serialPortSelection->count()>0);
    }
}

void MainWindow::updateTable(int parameter)
{
    // populate the table with the values from the configuration
    for (int c=0; c<8; c++)
    {
        for (int r=0; r<8; r++)
        {
            // pick the right parameter
            uint32_t v;
            switch (parameter)
            {
                case 0:  v=(uint32_t)mCfg.lampGlowDur[c][r] * GLOWDUR_CFG_SCALE; break;
                case 1:  v=(uint32_t)mCfg.lampBrightness[c][r]; break;
                default: v=0; break;
            }

            // update the table item
            QTableWidgetItem *pWI = ui->lampMatrix->item(r*2+1, c);
            if (pWI)
            {
                pWI->setText(QString::number(v, 10));
            }
        }
    }

    // activate table
    ui->lampMatrix->setEnabled(true);
}

void MainWindow::tableChanged(QTableWidgetItem *item)
{
    // get the data
    bool ok = true;
    int32_t v = item->text().toInt(&ok);
    int param = ui->parameterSelection->currentIndex();

    // skip header rows
    if ((item->row() % 2) == 0)
    {
        return;
    }

    // verify the value
    if (ok)
    {
        switch (param)
        {
        case 0: // glow duration
        {
            if ((v>65530) || (v%10))
            {
                if (v<0) v=0;
                if (v>65535) v= 65535;
                if (v%10) v=(v/10)*10;
                ui->statusBar->showMessage("Glow duration must be between 0 and 65535 and a multiple of 10!");
                ui->statusBar->setStyleSheet("background-color: rgb(255, 255, 0);");
            }
        }
        break;
        case 1: // brightness
        {
            if (v>7)
            {
                if (v<0) v=0;
                if (v>7) v=7;
                ui->statusBar->showMessage("Brightness must be between 0 and 7!");
                ui->statusBar->setStyleSheet("background-color: rgb(255, 255, 0);");
            }
        }
        break;
        default: ok = false; break;
        }
    }
    else
    {
        ui->statusBar->showMessage("Only numerical values allowed!");
        ui->statusBar->setStyleSheet("background-color: rgb(255, 255, 0);");
    }

    if (ok)
    {
        // apply to all selected cells
        for (uint32_t c=0; c<8; c++)
        {
            for (uint32_t r=0; r<8; r++)
            {
                QTableWidgetItem *pWI =ui->lampMatrix->item(r*2+1,c);
                if (pWI)
                {
                    if (pWI->isSelected())
                    {
                        // set the cell content
                        pWI->setText(QString::number(v,10));

                        // apply the changes to the configuration
                        switch (param)
                        {
                        case 0: mCfg.lampGlowDur[c][r] = (v / GLOWDUR_CFG_SCALE); break;
                        case 1: mCfg.lampBrightness[c][r] = v; break;
                        default: break;
                        }
                    }
                }
            }
        }
    }
    else
    {
        // revert
        updateTable(param);
    }
}

void MainWindow::editSelected()
{
    ui->lampMatrix->editItem(ui->lampMatrix->currentItem());
}

void MainWindow::selectByValue()
{
    QTableWidgetItem *pCWI =ui->lampMatrix->currentItem();
    if (pCWI)
    {
        // select all cells with the same value as the currently selected one
        for (uint32_t c=0; c<8; c++)
        {
            for (uint32_t r=0; r<8; r++)
            {
                QTableWidgetItem *pWI =ui->lampMatrix->item(r*2+1,c);
                if (pWI)
                {
                    if (pWI->text() == pCWI->text())
                    {
                        // select the cell
                        pWI->setSelected(true);
                    }
                    else
                    {
                        pWI->setSelected(false);
                    }
                }
            }
        }
    }
}

void MainWindow::fetchGameList()
{
    ui->statusBar->setStyleSheet("background-color: rgb(255, 255, 0);");
    ui->statusBar->showMessage("Connecting to github...");

    // download the latest games.json from github
    FileDownloader fd;
    if (!fd.download(QUrl(GITHUB_GAMES_LIST_URL), LOCAL_GAMES_LIST_FILE))
    {
        // error
        ui->statusBar->setStyleSheet("background-color: rgb(255, 0, 0);");
        QString errStr = "Update failed: ";
        errStr = fd.errorStr();
        ui->statusBar->showMessage(errStr);
    }
    else
    {
        // success
        ui->statusBar->setStyleSheet("background-color: rgb(0, 255, 0);");
        ui->statusBar->showMessage("Update completed.");

        // update the games list
        readGames();
        createGameList();
    }
}

void MainWindow::updateFW()
{
    FWUpdater fwUpdater;

    // check the version in the repository
    int v = fwUpdater.getRemoteVersion();
    if (v != 0)
    {
        /*
        if (mAGVersion >= v)
        {
            // already at newest version
            ui->statusBar->setStyleSheet("background-color: rgb(127, 255, 0);");
            ui->statusBar->showMessage("Your afterglow board is already up to date!");
        }
        else
        */
        {
            // ask again
            QMessageBox::StandardButton reply;
            QString updStr = "Do you want to update from v";
            updStr += QString::number(mAGVersion);
            updStr += " to v";
            updStr += QString::number(v);
            updStr += "?";
            reply = QMessageBox::question(this, "Confirm", updStr,
                                          QMessageBox::Yes|QMessageBox::No);
            if (reply == QMessageBox::Yes)
            {
                QMessageBox resMsgBox;

                // disconnect from the device
                connectAG();

                // act busy
                QApplication::setOverrideCursor(Qt::WaitCursor);
                ui->statusBar->showMessage("FW update in progress...");
                ui->statusBar->setStyleSheet("background-color: rgb(255, 255, 0);");

                // start the update process
                QString portDeviceName = "/dev/" + ui->serialPortSelection->currentText();
                if (fwUpdater.update(portDeviceName))
                {
                    resMsgBox.setText("Firmware update successful.");
                    ui->statusBar->showMessage("FW update done.");
                    ui->statusBar->setStyleSheet("background-color: rgb(0, 255, 0);");
                }
                else
                {
                    ui->statusBar->setStyleSheet("background-color: rgb(255, 0, 0);");
                    QString errStr = "Update failed: ";
                    errStr += fwUpdater.errorStr();
                    ui->statusBar->showMessage(errStr);
                    resMsgBox.setText("Firmware update failed: "+errStr);
                }

                // idle again
                QApplication::restoreOverrideCursor();

                // show the result dialog
                resMsgBox.setStandardButtons(QMessageBox::Ok);
                resMsgBox.setDefaultButton(QMessageBox::Ok);
                resMsgBox.setDetailedText(fwUpdater.responseStr());
                resMsgBox.exec();
            }
        }
    }
    else
    {
        // error contacting the server
        QString errStr = "Could not retrieve the latest version from github: ";
        errStr = fwUpdater.errorStr();
        ui->statusBar->setStyleSheet("background-color: rgb(255, 0, 0);");
        ui->statusBar->showMessage(errStr);
    }
}
