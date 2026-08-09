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
