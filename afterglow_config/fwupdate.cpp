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

#include "fwupdate.h"
#include "filedownload.h"
#include <QRegularExpression>
#include <QRegularExpressionMatch>


// github arduino sketch
#define GITHUB_ARDUINO_SKETCH_URL "https://raw.githubusercontent.com/smyp/afterglow/master/afterglow_arduino/afterglow_arduino.ino"

// local arduino sketch file name
#define GITHUB_ARDUINO_SKETCH_FILE "afterglow_arduino.ino"

// github firmware binary
#define GITHUB_ARDUINO_FW_URL "https://raw.githubusercontent.com/smyp/afterglow/master/afterglow_arduino/bin/afterglow_arduino.ino.eightanaloginputs.hex"

// local firmware binary file name
#define GITHUB_ARDUINO_FW_FILE "afterglow_arduino.eightanaloginputs.hex"



FWUpdater::FWUpdater()
{
    mpProcess = NULL;
}

FWUpdater::~FWUpdater()
{
    delete mpProcess;
}

int FWUpdater::getRemoteVersion()
{
    int version = 0;

    // get the arduino sketch (just for reading the current version)
    FileDownloader fd;
    if (fd.download(QUrl(GITHUB_ARDUINO_SKETCH_URL), GITHUB_ARDUINO_SKETCH_FILE))
    {
        // parse the current version number from the file
        QFile file(GITHUB_ARDUINO_SKETCH_FILE);
        if (file.open(QIODevice::ReadOnly))
        {
            QTextStream in(&file);
            QRegularExpression re("#define AFTERGLOW_VERSION (\\d+)");
            while (!in.atEnd())
            {
                QString line = in.readLine();
                QRegularExpressionMatch match = re.match(line);
                if (match.hasMatch())
                {
                    version = match.captured(1).toInt();
                    break;
                }
            }
            file.close();
        }
    }
    else
    {
        mErrorStr = fd.errorStr();
    }

    // return 0 on failure
    return version;
}

bool FWUpdater::update(const QString &portName)
{
    // download the firmware binary from github
    FileDownloader fd;
    if (fd.download(QUrl(GITHUB_ARDUINO_FW_URL), GITHUB_ARDUINO_FW_FILE))
    {
        // locate avrdude
        QString bin = "avrdude";
        QString arg = "-v -V -patmega328p -carduino -P";
        arg += portName;
        arg += "-b57600 -D -Uflash:w:";
        arg += GITHUB_ARDUINO_FW_FILE;
        arg += ":i";
        QStringList args;
        args.append(arg);

        // delete old processes first
        if (mpProcess)
        {
            delete mpProcess;
            mpProcess = NULL;
        }

        // start a new process
        mpProcess = new QProcess();
        if (mpProcess)
        {
            // connect to stdout of the process
            connect(mpProcess, SIGNAL(readyReadStandardOutput()),this, SLOT(stdOut()) );
            mpProcess->start(bin, args);
            return true;
        }
    }
    else
    {
        mErrorStr = fd.errorStr();
    }
    return false;
}

void FWUpdater::stdOut()
{
    if (mpProcess)
    {
        mProcessData += mpProcess->readAllStandardOutput();
    }
}
