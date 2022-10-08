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
#include <QStyle>


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

    ui->tickerText->setStyleSheet("background-color: rgb(55, 55, 55);");
    ui->versionLabel->setText("Afterglow Configuration Tool v" + QString(AGCONFIG_VERSION));
    ticker("Afterglow Config " + QString(AGCONFIG_VERSION) + " - Hello pinheads!", QColor("green"), QFont::Bold);
    ticker("Ready.", QColor("green"), QFont::Bold);

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
        ticker("Games list not present! Put games.json into root folder.", QColor("orange"), QFont::Normal);
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
            ticker(warning, QColor("red"), QFont::Normal);
        }
        else
        {
            // the connection will reset the arduino - allow some time for startup
            ticker("Rebooting the arduino...", QColor("orange"), QFont::Normal);
            QThread::sleep(2);

            // poll the afterglow version to verify the connection
            ticker("Polling the afterglow version...", QColor("orange"), QFont::Normal);

            mAGVersion = mSerialCommunicator.pollVersion(&mAGCfgVersion);
            if (mAGVersion != 0)
            {
                QString connectStr = "Afterglow revision ";
                connectStr += QString::number(mAGVersion, 10);
                connectStr += " cfg v";
                connectStr += QString::number(mAGCfgVersion, 10);
                ticker("Connected to "+connectStr, QColor("green"), QFont::Normal);
                ui->statusBar->showMessage("Connected to "+connectStr);
                ui->statusBar->setStyleSheet("background-color: rgb(0, 255, 0);");
                setConnected(true);

                // load the current configuration
                loadAG();
            }
            else
            {
                // disconnect from the port
                mSerialCommunicator.disconnect();

                ticker("No afterglow board detected on this port!", QColor("red"), QFont::Bold);

                // should we try to upload the FW
                QMessageBox::StandardButton reply;
                QString updStr = "No Afterglow detected on this port.\nDo you want to upload the firmware to this device?";
                reply = QMessageBox::question(this, "Confirm", updStr, QMessageBox::Yes|QMessageBox::No);
                if (reply == QMessageBox::Yes)
                {
                    // update the firmware
                    updateFW();
                }
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
            ticker("Configuration successfully loaded", QColor("green"), QFont::Normal);
        }
        else
        {
            ticker("Configuration poll failed!", QColor("red"), QFont::Normal);
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
            ticker("Configuration successfully reset", QColor("green"), QFont::Normal);

            // load the new configuration from AG
            loadAG();
        }
        else
        {
            ticker("Configuration reset failed!", QColor("red"), QFont::Normal);
        }
        setCursor(Qt::ArrowCursor);

        // update the GUI with the new configuration
        updateTable(ui->parameterSelection->currentIndex());
    }
    else
    {
        ticker("Not connected to afterglow!", QColor("red"), QFont::Normal);
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
            ticker("Configuration successfully saved", QColor("green"), QFont::Normal);
        }
        else
        {
            ticker("Configuration saving failed!", QColor("red"), QFont::Normal);
        }
        setCursor(Qt::ArrowCursor);
    }
    else
    {
        ticker("Not connected to afterglow!", QColor("red"), QFont::Normal);
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
    QJsonArray::iterator it;
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
        int numRows = (lamps.size() == 64) ? 8 : 10;
        for (int c=0; c<NUM_COL; c++)
        {
            for (int r=0; r<NUM_ROW; r++)
            {
                QTableWidgetItem *pTWI = ui->lampMatrix->item(r*2, c);
                if (pTWI)
                {
                    if (r<numRows)
                    {
                        pTWI->setText(lamps.at(c*numRows+r).toString());
                    }
                    else
                    {
                        pTWI->setText("");
                    }
                }
            }
        }
    }
}

void MainWindow::prepareLampMatrix()
{
    ui->lampMatrix->setRowCount(2*NUM_ROW);
    ui->lampMatrix->setColumnCount(NUM_COL);
    ui->lampMatrix->setWordWrap(true);
    ui->lampMatrix->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->lampMatrix->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    for (int c=0; c<NUM_COL; c++)
    {
        for (int r=0; r<2*NUM_ROW; r++)
        {
            if (c==0)
            {
                //ui->lampMatrix->setRowHeight(r,(r%2)?20:36);
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
                ui->lampMatrix->setItem(r, c, new QTableWidgetItem("0"));
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
                ui->lampMatrix->setItem(r, c, new QTableWidgetItem("N/A"));
                QTableWidgetItem *pWI = ui->lampMatrix->item(r, c);
                if (pWI)
                {
                    pWI->setFlags(pWI->flags() ^ Qt::ItemIsEditable ^ Qt::ItemIsSelectable);
                    pWI->setTextAlignment(Qt::AlignLeft);
                    QFont f= pWI->font();
                    f.setPointSize(8);
                    pWI->setFont(f);
                    pWI->setForeground(Qt::darkBlue);
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
    ui->connectButton->setIcon(connected ? QIcon(":/icon/icons/network-disconnect.svg"): QIcon(":/icon/icons/network-connect.svg"));
}

void MainWindow::enumSerialPorts()
{
    // remember the current selection
    QString currText = ui->serialPortSelection->currentText();

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

        // restore the previous selection if possible
        ui->serialPortSelection->setCurrentText(currText);

        // enable the connect button
        ui->connectButton->setEnabled(ui->serialPortSelection->count()>0);
    }
}

void MainWindow::updateTable(int parameter)
{
    // clear the selection, otherwise the selected item would adopt the
    // change of the last value in tableChanged()
    ui->lampMatrix->clearSelection();

    // populate the table with the values from the configuration
    for (int c=0; c<NUM_COL; c++)
    {
        for (int r=0; r<NUM_ROW; r++)
        {
            // pick the right parameter
            uint32_t v;
            switch (parameter)
            {
                case 0:  v=static_cast<uint32_t>(mCfg.lampGlowDur[c][r] * GLOWDUR_CFG_SCALE); break;
                case 1:  v=static_cast<uint32_t>(mCfg.lampBrightness[c][r]); break;
                default: v=0; break;
            }

            // update the table item
            QTableWidgetItem *pWI = ui->lampMatrix->item(r*2+1, c);
            if (pWI)
            {
                pWI->setText(QString::number(v, 10));
            }

            // disable unused items
            if ((mCfg.version <= 1) && (r>7))
            {
                pWI->setFlags(pWI->flags() & ~Qt::ItemIsEnabled);
                pWI->setBackground(Qt::Dense6Pattern);
            }
            else
            {
                pWI->setFlags(pWI->flags() | Qt::ItemIsEnabled);
                pWI->setBackground(Qt::NoBrush);
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
                ticker("Glow duration must be between 0 and 65535 and a multiple of 10!", QColor("red"), QFont::Normal);
            }
        }
        break;
        case 1: // brightness
        {
            if (v>7)
            {
                if (v<0) v=0;
                if (v>7) v=7;
                ticker("Brightness must be between 0 and 7!", QColor("red"), QFont::Normal);
            }
        }
        break;
        default: ok = false; break;
        }
    }
    else
    {
        ticker("Only numerical values allowed!", QColor("red"), QFont::Normal);
    }

    if (ok)
    {
        // apply to all selected cells
        for (int32_t c=0; c<NUM_COL; c++)
        {
            for (int32_t r=0; r<NUM_ROW; r++)
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
                        case 0: mCfg.lampGlowDur[c][r] = static_cast<uint8_t>(v / GLOWDUR_CFG_SCALE); break;
                        case 1: mCfg.lampBrightness[c][r] = static_cast<uint8_t>(v); break;
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
        for (int32_t c=0; c<NUM_COL; c++)
        {
            for (int32_t r=0; r<NUM_ROW; r++)
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
    ticker("Connecting to afterglow repository...", QColor("orange"), QFont::Normal);

    // download the latest games.json from github
    FileDownloader fd;
    if (!fd.download(QUrl(GITHUB_GAMES_LIST_URL), LOCAL_GAMES_LIST_FILE))
    {
        // error
        QString errStr = "Update failed: ";
        errStr = fd.errorStr();
        ticker(errStr, QColor("red"), QFont::Normal);
    }
    else
    {
        // success
        ticker("Update completed.", QColor("green"), QFont::Normal);

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
        // ask for FW type if not known from configuration
        bool whitestar = false;
        if (mCfg.version == 0)
        {
            QMessageBox box;
            box.setWindowTitle("Choose firmware type");
            box.setStandardButtons(QMessageBox::Yes|QMessageBox::No);
            QAbstractButton *pB1 = box.button(QMessageBox::Yes);
            pB1->setText("WPC/Sys11/DE");
            QAbstractButton *pB2 = box.button(QMessageBox::No);
            pB2->setText("SAM/Whitestar");
            box.exec();
            whitestar = (box.clickedButton() == pB2);
        }
        else
        {
            whitestar = (mCfg.version == 2); // is this a whitestar board?
        }

        // ask again
        QMessageBox::StandardButton reply;
        QString updStr = "Do you want to update from v";
        updStr += QString::number(mAGVersion);
        updStr += " to v";
        updStr += QString::number(v);
        if (whitestar)
        {
            updStr += " (WHITESTAR)";
        }
        updStr += "?";
        reply = QMessageBox::question(this, "Confirm", updStr,
                                      QMessageBox::Yes|QMessageBox::No);
        if (reply == QMessageBox::Yes)
        {
            // if we're not connected yet then this is an attempt to initialise the device
            bool init = !mConnected;

            // disconnect from the device
            if (mConnected)
            {
                connectAG();
            }

            // act busy
            ticker("FW update in progress...", QColor("orange"), QFont::Normal);

            // start the update process
#ifdef Q_OS_LINUX
            QString portDeviceName = "/dev/" + ui->serialPortSelection->currentText();
#elif defined Q_OS_WIN
            QString portDeviceName = ui->serialPortSelection->currentText();
#elif defined Q_OS_MACOS
            QString portDeviceName = "/dev/" + ui->serialPortSelection->currentText();
#endif
            bool success = fwUpdater.update(portDeviceName, whitestar);
            if (success)
            {
                ticker("FW update done.", QColor("green"), QFont::Normal);
            }
            else
            {
                QString errStr = "Update failed: ";
                errStr += fwUpdater.errorStr();
                ticker(errStr, QColor("red"), QFont::Normal);
            }

            // reconnect
            if (!init || success)
            {
                connectAG();
            }
        }
    }
    else
    {
        // error contacting the server
        QString errStr = "Could not retrieve the latest version from github: ";
        errStr += fwUpdater.errorStr();
        ticker(errStr, QColor("red"), QFont::Normal);
    }
}

void MainWindow::ticker(const QString &text, const QColor &c, int weight)
{
    ui->tickerText->setFontWeight(weight);
    ui->tickerText->setTextColor(c);
    ui->tickerText->append(text);
    ui->tickerText->repaint();
}
