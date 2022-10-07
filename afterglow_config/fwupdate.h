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

#ifndef FWUPDATE_H
#define FWUPDATE_H

#include <QProcess>
#include <QString>
#include "fwupdatedialog.h"

class FWUpdater : public QObject
{
    Q_OBJECT

public:
    FWUpdater();
    ~FWUpdater();

    int getRemoteVersion();
    bool update(const QString &portName, bool whitestar);
    QString& errorStr() { return mErrorStr; }
    QString& responseStr() { return mResponseStr; }

private slots:
    void stdOut();
    void procFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void errorOccurred(QProcess::ProcessError error);

private:

    QString mResponseStr;
    QString mErrorStr;
    QProcess *mpProcess;
    FWUpdateDialog *mpFWUpdDialog;
    QString mAvrdudeOutput;
    bool mError;
};


#endif // FWUPDATE_H
