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

#ifndef FILEDOWNLOAD_H
#define FILEDOWNLOAD_H

#include <QObject>
#include <QFile>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QUrl>
#include <QByteArray>

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
    QByteArray mData;
};

#endif // FILEDOWNLOAD_H
