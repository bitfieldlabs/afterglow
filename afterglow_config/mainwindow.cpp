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
#include <QFile>
#include <QJsonObject>
#include <QSerialPortInfo>

// default glow duration [ms]
#define DEFAULT_GLOWDUR 180

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->statusBar->showMessage("Not connected");

    // deactivate load and save buttons by default
    ui->saveButton->setEnabled(false);
    ui->loadButton->setEnabled(false);

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

    // enumerate the serial ports
    enumSerialPorts();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::readGames(void)
{
    QFile file("games.json");
    if (!file.exists())
    {
        ui->statusBar->showMessage("Games list not present! Put games.json into root folder.");
        return;
    }
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QByteArray bla = file.readAll();
    file.close();
    QJsonParseError err;
    mGamesDoc = QJsonDocument::fromJson(bla, &err);
    mGamesList = mGamesDoc.array();
}

void MainWindow::gameChanged(int ix)
{
    updateGameDesc(ix);
}

void MainWindow::createGameList()
{
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
                ui->lampMatrix->setRowHeight(r,(r%2)?30:48);
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
                ui->lampMatrix->setItem(r, c, new QTableWidgetItem(QString::number(DEFAULT_GLOWDUR, 10)));
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
}

void MainWindow::enumSerialPorts()
{
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos)
    {
        QString s = info.portName();
        ui->serialPortSelection->addItem(s);
    }

}
