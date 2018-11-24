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

#include "filedownload.h"
#include <QNetworkRequest>
#include <QEventLoop>

FileDownloader::FileDownloader()
{
    mpNm = NULL;
    mpReply = NULL;
    mpFile = NULL;
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
