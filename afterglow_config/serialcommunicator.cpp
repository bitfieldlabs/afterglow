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

#include "serialcommunicator.h"
#include <QRegularExpression>
#include <QtEndian>

// timeout for serial communication [ms]
#define AG_SERIAL_TIMEOUT 4000

// write buffer size [bytes]
#define AG_CMD_WRITE_BUF 32

// command terminator character
#define AG_CMD_TERMINATOR ':'

// version poll command string
#define AG_CMD_VERSION_POLL "AGV"

// configuration poll command string
#define AG_CMD_CFG_POLL "AGCP"

// configuration save command string
#define AG_CMD_CFG_SAVE "AGCS"

// configuration reset to default command string
#define AG_CMD_CFG_DEFAULT "AGCD"

// data ready string
#define AG_CMD_CFG_DATA_READY "AGDR"

// acknowledge string
#define AG_CMD_ACK "AGCACK"

// NOT acknowledge string
#define AG_CMD_NACK "AGCNACK"

// recording size poll command string
#define AG_CMD_REC_SIZE "AGRS"

// recording download command string
#define AG_CMD_REC_DOWNLOAD "AGRD"


SerialCommunicator::SerialCommunicator()
{

}

bool SerialCommunicator::openPort(const QString &portName)
{
    bool res = true;

    // close previous connections
    disconnect();

    // port setup
    mSerialPort.setPortName(portName);
    mSerialPort.setBaudRate(115200);
    mSerialPort.setDataBits(QSerialPort::Data8);
    mSerialPort.setStopBits(QSerialPort::OneStop);
    mSerialPort.setParity(QSerialPort::NoParity);
    mSerialPort.setFlowControl(QSerialPort::NoFlowControl);

    // open the port
    if (mSerialPort.open(QIODevice::ReadWrite) == false)
    {
        res = false;
    }
    else
    {
        mSerialPort.setDataTerminalReady(true); // needed for Windows connection to the rp2040
    }
    return res;
}

void SerialCommunicator::disconnect()
{
    mSerialPort.close();
}

int SerialCommunicator::pollVersion(int *pCfgVersion)
{
    int version = 0;
    *pCfgVersion = 0;

    // clear the port
    mSerialPort.clear();

    // send the request
    QString cmd(AG_CMD_VERSION_POLL);
    cmd += AG_CMD_TERMINATOR;
    mSerialPort.write(cmd.toUtf8());

    // check the response
    if (mSerialPort.waitForBytesWritten(AG_SERIAL_TIMEOUT))
    {
        // read response
        if (mSerialPort.waitForReadyRead(AG_SERIAL_TIMEOUT))
        {
            QByteArray responseData = mSerialPort.readAll();
            while (mSerialPort.waitForReadyRead(100))
            {
                responseData += mSerialPort.readAll();
            }

            // parse the version
            const QString response = QString::fromUtf8(responseData);
            QRegularExpression re("AGV\\s(\\d+)\\s(\\d+)");
            QRegularExpressionMatch match = re.match(response);
            if (match.hasMatch())
            {
                version = match.captured(1).toInt();
                *pCfgVersion = match.captured(2).toInt();
            }
        }
    }
    return version;
}

bool SerialCommunicator::loadCfg(AFTERGLOW_CFG_t *pCfg)
{
    bool res = false;
    QByteArray responseData;

    // clear the port
    if (!mSerialPort.clear())
    {
        return false;
    }

    // send the request
    QString cmd(AG_CMD_CFG_POLL);
    cmd += AG_CMD_TERMINATOR;
    mSerialPort.write(cmd.toUtf8());

    // check the response
    if (mSerialPort.waitForBytesWritten(AG_SERIAL_TIMEOUT))
    {
        // read response
        if (mSerialPort.waitForReadyRead(AG_SERIAL_TIMEOUT))
        {
            responseData = mSerialPort.readAll();
            while (mSerialPort.waitForReadyRead(100))
            {
                responseData += mSerialPort.readAll();
            }

            // check for v1 configuration
            uint32_t rlen = responseData.length();
            uint32_t v1len = sizeof(AFTERGLOW_CFG_V1_t);
            uint32_t v2len = sizeof(AFTERGLOW_CFG_V2_t);
            uint32_t v3len = sizeof(AFTERGLOW_CFG_t);
            if (rlen == v1len)
            {
                int cfgSize = sizeof(AFTERGLOW_CFG_V1_t);
                const AFTERGLOW_CFG_V1_t *pkCfgV1 = (const AFTERGLOW_CFG_V1_t*)responseData.constData();
                if (pkCfgV1->version == 1)
                {
                    // check the crc
                    uint32_t crc = calculateCRC32((const uint8_t*)responseData.constData(), cfgSize-4);
                    uint32_t crcInCfg = qFromLittleEndian(pkCfgV1->crc);
                    if (crc == crcInCfg)
                    {
                        // copy the data, converting to current version
                        pCfg->version = pkCfgV1->version;
                        pCfg->crc = pkCfgV1->crc;
                        for (int c=0; c<NUM_COL; c++)
                        {
                            memcpy(pCfg->lampBrightness[c], pkCfgV1->lampBrightness[c], sizeof(pkCfgV1->lampBrightness[c]));
                            // only one glow duration, use for both on and off time
                            memcpy(pCfg->lampGlowDurOn[c], pkCfgV1->lampGlowDur[c], sizeof(pkCfgV1->lampGlowDur[c]));
                            memcpy(pCfg->lampGlowDurOff[c], pkCfgV1->lampGlowDur[c], sizeof(pkCfgV1->lampGlowDur[c]));
                        }
                        res = true;
                    }
                }
            }
            // check for v2 configuration
            else if (rlen == v2len)
            {
                int cfgSize = sizeof(AFTERGLOW_CFG_V2_t);
                const AFTERGLOW_CFG_V2_t *pkCfgV2 = (const AFTERGLOW_CFG_V2_t*)responseData.constData();
                if (pkCfgV2->version == 2)
                {
                    // check the crc
                    uint32_t crc = calculateCRC32((const uint8_t*)responseData.constData(), cfgSize-4);
                    uint32_t crcInCfg = qFromLittleEndian(pkCfgV2->crc);
                    if (crc == crcInCfg)
                    {
                        // copy the data, converting to current version
                        pCfg->version = pkCfgV2->version;
                        pCfg->crc = pkCfgV2->crc;
                        for (int c=0; c<NUM_COL; c++)
                        {
                            memcpy(pCfg->lampBrightness[c], pkCfgV2->lampBrightness[c], sizeof(pkCfgV2->lampBrightness[c]));
                            // only one glow duration, use for both on and off time
                            memcpy(pCfg->lampGlowDurOn[c], pkCfgV2->lampGlowDur[c], sizeof(pkCfgV2->lampGlowDur[c]));
                            memcpy(pCfg->lampGlowDurOff[c], pkCfgV2->lampGlowDur[c], sizeof(pkCfgV2->lampGlowDur[c]));
                        }
                        res = true;
                    }
                }
            }
            // check for v3 configuration
            else if (rlen == v3len)
            {
                int cfgSize = sizeof(AFTERGLOW_CFG_t);

                // copy the data
                memcpy(pCfg, responseData.constData(), cfgSize);

                // check the crc
                uint32_t crc = calculateCRC32((const uint8_t*)responseData.constData(), cfgSize-sizeof(pCfg->crc));
                uint32_t crcInCfg = qFromLittleEndian(pCfg->crc);
                if (crc == crcInCfg)
                {
                    res = true;
                }
            }
        }
    }

    return res;
}

uint32_t SerialCommunicator::serialPortError()
{
    return mSerialPort.error();
}

bool SerialCommunicator::defaultCfg()
{
    bool res = false;

    // clear the port
    if (!mSerialPort.clear())
    {
        return false;
    }

    // send the request
    QString cmd(AG_CMD_CFG_DEFAULT);
    cmd += AG_CMD_TERMINATOR;
    mSerialPort.write(cmd.toUtf8());

    // check the response
    if (mSerialPort.waitForBytesWritten(AG_SERIAL_TIMEOUT))
    {
        // read response
        if (mSerialPort.waitForReadyRead(AG_SERIAL_TIMEOUT))
        {
            QByteArray responseData = mSerialPort.readAll();
            while (mSerialPort.waitForReadyRead(100))
            {
                responseData += mSerialPort.readAll();
            }
            // check for the ACK
            const QString response = QString::fromUtf8(responseData);
            if (response.contains(AG_CMD_ACK))
            {
                res = true;
            }
        }
    }

    return res;
}

bool SerialCommunicator::saveCfg(AFTERGLOW_CFG_t *pCfg)
{
    AFTERGLOW_CFG_V1_t cfgV1;
    AFTERGLOW_CFG_V2_t cfgV2;
    const char *pkData = (const char*)pCfg;
    int cfgSize = sizeof(AFTERGLOW_CFG_t);
    bool res = false;

    // convert to config v1 if necessary
    if (pCfg->version == 1)
    {
        cfgV1.version = pCfg->version;
        for (int c=0; c<NUM_COL; c++)
        {
            memcpy(cfgV1.lampBrightness[c], pCfg->lampBrightness[c], sizeof(cfgV1.lampBrightness[c]));
            memcpy(cfgV1.lampGlowDur[c], pCfg->lampGlowDurOn[c], sizeof(cfgV1.lampGlowDur[c]));
        }
        pkData = (const char*)&cfgV1;
        cfgSize = sizeof(cfgV1);

        // update the crc
        uint32_t crc = calculateCRC32((const uint8_t*)&cfgV1, cfgSize-sizeof(cfgV1.crc));
        cfgV1.crc = qToLittleEndian(crc);
    }
    // convert to config v2 if necessary
    else if (pCfg->version == 2)
    {
        cfgV2.version = pCfg->version;
        for (int c=0; c<NUM_COL; c++)
        {
            memcpy(cfgV2.lampBrightness[c], pCfg->lampBrightness[c], sizeof(cfgV2.lampBrightness[c]));
            memcpy(cfgV2.lampGlowDur[c], pCfg->lampGlowDurOn[c], sizeof(cfgV2.lampGlowDur[c]));
        }
        pkData = (const char*)&cfgV2;
        cfgSize = sizeof(cfgV2);

        // update the crc
        uint32_t crc = calculateCRC32((const uint8_t*)&cfgV2, cfgSize-sizeof(cfgV2.crc));
        cfgV2.crc = qToLittleEndian(crc);
    }
    // current config version
    else
    {
        // update the crc
        uint32_t crc = calculateCRC32((const uint8_t*)pCfg, cfgSize-sizeof(pCfg->crc));
        pCfg->crc = qToLittleEndian(crc);
    }

    // clear the port
    mSerialPort.clear();

    int size = 0;

    // send the request
    QString cmd(AG_CMD_CFG_SAVE);
    cmd += AG_CMD_TERMINATOR;
    mSerialPort.write(cmd.toUtf8());

    // send the configuration in small chunks
    bool failed = false;
    while ((size < cfgSize) && !failed)
    {
        // wait for the data to be written
        if (mSerialPort.waitForBytesWritten(AG_SERIAL_TIMEOUT))
        {
            // wait for the data ready signal
            if (mSerialPort.waitForReadyRead(AG_SERIAL_TIMEOUT))
            {
                QByteArray responseData = mSerialPort.readAll();
                while (mSerialPort.waitForReadyRead(10))
                {
                    responseData += mSerialPort.readAll();
                }

                // check for the data ready signal
                const QString response = QString::fromUtf8(responseData);
                if (response.contains(AG_CMD_CFG_DATA_READY))
                {
                    // send data
                    uint32_t wb = ((cfgSize-size) < AG_CMD_WRITE_BUF) ? (cfgSize-size) : AG_CMD_WRITE_BUF;
                    mSerialPort.write(pkData, wb);
                    pkData += wb;
                    size += wb;
                }
            }
        }
        else
        {
            failed = true;
        }
    }

    // wait for the data to be written
    if (mSerialPort.waitForBytesWritten(AG_SERIAL_TIMEOUT))
    {
        // wait for the final acknowledge
        if (mSerialPort.waitForReadyRead(AG_SERIAL_TIMEOUT))
        {
            QByteArray responseData = mSerialPort.readAll();
            while (mSerialPort.waitForReadyRead(10))
            {
                responseData += mSerialPort.readAll();
            }

            // check for the ACK
            const QString response = QString::fromUtf8(responseData);
            if (response.contains(AG_CMD_ACK))
            {
                res = true;
            }
        }
    }

    return res;
}

uint32_t SerialCommunicator::calculateCRC32(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xffffffff;
    while (length--)
    {
        uint8_t c = *data++;
        for (uint32_t i = 0x80; i > 0; i >>= 1)
        {
            bool bit = crc & 0x80000000;
            if (c & i)
            {
                bit = !bit;
            }
            crc <<= 1;
            if (bit)
            {
                crc ^= 0x04c11db7;
            }
        }
    }
    return crc;
}

uint32_t SerialCommunicator::pollRecSize()
{
    int recSize = 0;

    // clear the port
    mSerialPort.clear();

    // send the request
    QString cmd(AG_CMD_REC_SIZE);
    cmd += AG_CMD_TERMINATOR;
    mSerialPort.write(cmd.toUtf8());

    // check the response
    if (mSerialPort.waitForBytesWritten(AG_SERIAL_TIMEOUT))
    {
        // read response
        if (mSerialPort.waitForReadyRead(AG_SERIAL_TIMEOUT))
        {
            QByteArray responseData = mSerialPort.readAll();
            while (mSerialPort.waitForReadyRead(100))
            {
                responseData += mSerialPort.readAll();
            }

            // parse the version
            const QString response = QString::fromUtf8(responseData);
            QString regExpStr = QString(AG_CMD_REC_SIZE)+"\\s(\\d+)";
            QRegularExpression re(regExpStr);
            QRegularExpressionMatch match = re.match(response);
            if (match.hasMatch())
            {
                recSize = match.captured(1).toInt();
            }
        }
    }
    return recSize;
}

uint32_t SerialCommunicator::recDownloadChunk(char *pBuf, uint32_t bufSize, bool firstChunk)
{
    uint32_t bytesRead = 0;

    if (firstChunk)
    {
        // clear the port
        mSerialPort.clear();

        // send the request
        QString cmd(AG_CMD_REC_DOWNLOAD);
        cmd += AG_CMD_TERMINATOR;
        mSerialPort.write(cmd.toUtf8());
    }

    if (mSerialPort.waitForReadyRead(100))
    {
        bytesRead += mSerialPort.read(pBuf, bufSize);
    }

    return bytesRead;
}
