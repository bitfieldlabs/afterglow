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

#ifndef FILEDOWNLOAD_H
#define FILEDOWNLOAD_H

#include <QObject>
#include <QFile>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QUrl>

class FileDownloader : public QObject
{
    Q_OBJECT

public:
    FileDownloader();
    ~FileDownloader();

    bool download(const QUrl &url, const QString &fileName);
    QString& errorStr() { return mErrorStr; }

private slots:
    void httpReadyRead();
    void httpDownloadFinished();

private:
    bool mSuccess;
    QString mFileName;
    QString mErrorStr;
    QNetworkAccessManager *mpNm;
    QNetworkReply *mpReply;
    QFile *mpFile;
};

#endif // FILEDOWNLOAD_H
