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
#define AG_SERIAL_TIMEOUT 2000

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

    // open the port
    if (mSerialPort.open(QIODevice::ReadWrite) == false)
    {
        res = false;
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

    // clear the port
    mSerialPort.clear();

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
            QByteArray responseData = mSerialPort.readAll();
            while (mSerialPort.waitForReadyRead(100))
            {
                responseData += mSerialPort.readAll();
            }

            // check the size
            int cfgSize = sizeof(AFTERGLOW_CFG_t);
            if (responseData.length() == cfgSize)
            {
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

bool SerialCommunicator::defaultCfg()
{
    bool res = false;

    // clear the port
    mSerialPort.clear();

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
    bool res = false;

    // update the crc
    int cfgSize = sizeof(AFTERGLOW_CFG_t);
    uint32_t crc = calculateCRC32((const uint8_t*)pCfg, cfgSize-sizeof(pCfg->crc));
    pCfg->crc = qToLittleEndian(crc);

    // clear the port
    mSerialPort.clear();

    // send the request
    QString cmd(AG_CMD_CFG_SAVE);
    cmd += AG_CMD_TERMINATOR;
    mSerialPort.write(cmd.toUtf8());

    // send the configuration in small chunks
    int size = 0;
    const char *pkData = (const char*)pCfg;
    while (size < cfgSize)
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
