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


// github games list URL
#define GITHUB_ARDUINO_SKETCH_URL "https://raw.githubusercontent.com/smyp/afterglow/master/afterglow_arduino/afterglow_arduino.ino"

// local games list file name
#define GITHUB_ARDUINO_SKETCH_FILE "afterglow_arduino.ino"



FWUpdater::FWUpdater()
{
    mpProcess = NULL;
}

FWUpdater::~FWUpdater()
{
    delete mpProcess;
}

int FWUpdater::getRemoteVersion(QString *pFn)
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
                }
            }
            file.close();
        }
    }

    // return 0 on failure
    return version;
}
