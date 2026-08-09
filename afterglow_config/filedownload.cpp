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

#include "filedownload.h"
#include <QNetworkRequest>
#include <QEventLoop>

FileDownloader::FileDownloader()
{
    mpNm = nullptr;
    mpReply = nullptr;
    mpFile = nullptr;
    mSuccess = false;
}

FileDownloader::~FileDownloader()
{
    delete mpFile;
    delete mpNm;
}

bool FileDownloader::download(const QUrl &url, const QString &fileName)
{
    // start the network manager
    if (!mpNm)
    {
        mpNm = new QNetworkAccessManager();
    }
    if (mpNm)
    {
        // place the request
        mpReply = mpNm->get(QNetworkRequest(url));
        mFileName = fileName;
        mSuccess = false;
        if (mpReply)
        {
            // handle the signals
            connect(mpReply, SIGNAL(readyRead()), this, SLOT(httpReadyRead()));
            connect(mpReply, SIGNAL(finished()), this, SLOT(httpDownloadFinished()));
        }
        // clear previous data
        mData.clear();
    }

    // wait for the request to be handled
    QEventLoop loop;
    connect(mpReply,  SIGNAL(finished()), &loop, SLOT(quit()));
    loop.exec();

    // done
    return mSuccess;
}

void FileDownloader::httpReadyRead()
{
    // add the received data to the buffer
    mData.append(mpReply->readAll());
}

void FileDownloader::httpDownloadFinished()
{
    // open the destination file
    if (QFile::exists(mFileName))
    {
        // remove the current file
        QFile::remove(mFileName);
    }
    mpFile = new QFile(mFileName);
    if (!mpFile->open(QIODevice::WriteOnly))
    {
        mSuccess = false;
        mErrorStr = "Unable to save file: ";
        mErrorStr += mpFile->errorString();
        delete mpFile;
        mpFile = NULL;
    }

    // write the received data to the file
    if (mpFile)
    {
        mpFile->write(mData);

        // close the file
        mpFile->flush();
        mpFile->close();
    }
    if (mpReply->error())
    {
        // store error information
        mErrorStr = mpReply->errorString();
        mSuccess = false;
    }
    else
    {
        mSuccess = true;
    }
    mpReply->deleteLater();
}
