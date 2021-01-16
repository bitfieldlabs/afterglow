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

// avrdude binary
#ifdef Q_OS_LINUX
#define AVRDUDE_BINARY "avrdude"
#elif defined Q_OS_WIN
#define AVRDUDE_BINARY "avrdude\\avrdude.exe"
#elif defined Q_OS_MACOS
#define AVRDUDE_BINARY "avrdude\\avrdude"
#endif

// github arduino sketch
#define GITHUB_ARDUINO_SKETCH_URL "https://raw.githubusercontent.com/bitfieldlabs/afterglow/master/afterglow_arduino/afterglow_arduino.ino"

// local arduino sketch file name
#define GITHUB_ARDUINO_SKETCH_FILE "afterglow_arduino.ino"

// github firmware binary
#define GITHUB_ARDUINO_FW_URL "https://raw.githubusercontent.com/bitfieldlabs/afterglow/master/afterglow_arduino/bin/afterglow_arduino.ino.eightanaloginputs.hex"

// local firmware binary file name
#define GITHUB_ARDUINO_FW_FILE "afterglow_arduino.eightanaloginputs.hex"


FWUpdater::FWUpdater()
{
    mpProcess = nullptr;
    mpFWUpdDialog = nullptr;
    mError = false;
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
        QString bin = AVRDUDE_BINARY;

        // compose the argument list
        QStringList args;
        args.append("-v");
        args.append("-V");
        args.append("-patmega328p");
        args.append("-carduino");
        args.append("-b57600");
        QString portArg = "-P";
        portArg += portName;
        args.append(portArg);
        args.append("-D");
        QString flashArg = "-Uflash:w:";
        flashArg += GITHUB_ARDUINO_FW_FILE;
        flashArg += ":i";
        args.append(flashArg);

        // delete old processes first
        if (mpProcess)
        {
            delete mpProcess;
            mpProcess = nullptr;
        }

        // start a new process
        mpProcess = new QProcess();
        if (mpProcess)
        {
            // open the output dialog
            mpFWUpdDialog = new FWUpdateDialog();

            // connect to stdout of the process
            mAvrdudeOutput.clear();
            mpProcess->setProcessChannelMode(QProcess::MergedChannels);
            connect(mpProcess, SIGNAL(readyRead()), this, SLOT(stdOut()), Qt::DirectConnection);
            connect(mpProcess, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(procFinished(int,QProcess::ExitStatus)), Qt::DirectConnection);
            connect(mpProcess, SIGNAL(errorOccurred(QProcess::ProcessError)), this, SLOT(errorOccurred(QProcess::ProcessError)), Qt::DirectConnection);
            mpProcess->start(bin, args);
            mpFWUpdDialog->exec();

            // terminate the process if it's still running
            if (mpProcess->state() != QProcess::NotRunning)
            {
                mpProcess->terminate();
            }

            // read all output
            mResponseStr = mpProcess->readAll();

            // cleanup
            delete mpProcess;
            mpProcess = nullptr;
            delete mpFWUpdDialog;
            mpFWUpdDialog = nullptr;

            return !mError;
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
        QByteArray data = mpProcess->readAllStandardOutput();
        mAvrdudeOutput += data;
        if (mpFWUpdDialog)
        {
            mpFWUpdDialog->setOutput(mAvrdudeOutput);
        }
    }
}

void FWUpdater::procFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    if (mpFWUpdDialog)
    {
        if ((exitStatus != QProcess::NormalExit) || (exitCode != 0))
        {
            // paint the message window red
            mpFWUpdDialog->setMsgStyle("background-color: rgb(255, 179, 179);");
            mError = true;
        }
        else
        {
            // we like green
            mpFWUpdDialog->setMsgStyle("background-color: rgb(179, 255, 179);");
        }
    }
}

void FWUpdater::errorOccurred(QProcess::ProcessError error)
{
    if (mpFWUpdDialog)
    {
        // paint the message window red
        mpFWUpdDialog->setOutput("Failed to run avrdude! Check whether the binary is available on your system.");
        mpFWUpdDialog->setMsgStyle("background-color: rgb(255, 179, 179);");
        mError = true;
    }
    (void)error;
}
