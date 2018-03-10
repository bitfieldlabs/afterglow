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

// version poll command string
#define AG_CMD_VERSION_POLL "AGV:"

// configuration poll command string
#define AG_CMD_CFG_POLL "AGCP:"


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
    mSerialPort.write(AG_CMD_VERSION_POLL);

    // check the response
    if (mSerialPort.waitForBytesWritten(AG_SERIAL_TIMEOUT))
    {
        // read response
        if (mSerialPort.waitForReadyRead(AG_SERIAL_TIMEOUT))
        {
            QByteArray responseData = mSerialPort.readAll();
            while (mSerialPort.waitForReadyRead(10))
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
    mSerialPort.write(AG_CMD_CFG_POLL);

    // check the response
    if (mSerialPort.waitForBytesWritten(AG_SERIAL_TIMEOUT))
    {
        // read response
        if (mSerialPort.waitForReadyRead(AG_SERIAL_TIMEOUT))
        {
            QByteArray responseData = mSerialPort.readAll();
            while (mSerialPort.waitForReadyRead(10))
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
