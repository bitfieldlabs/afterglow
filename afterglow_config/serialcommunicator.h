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

#ifndef SERIALCOMMUNICATOR_H
#define SERIALCOMMUNICATOR_H

#include <QSerialPort>
#include "agconfig.h"

class SerialCommunicator
{
public:
    SerialCommunicator();

    bool openPort(const QString &portName);
    void disconnect();
    int pollVersion(int *pCfgVersion);
    bool loadCfg(AFTERGLOW_CFG_t *pCfg);
    bool defaultCfg();
    bool saveCfg(AFTERGLOW_CFG_t *pCfg);
    uint32_t pollRecSize();
    uint32_t recDownloadChunk(char *buf, uint32_t bufSize, bool firstChunk);
    uint32_t serialPortError();

private:
    uint32_t calculateCRC32(const uint8_t *data, uint16_t length);

    QSerialPort mSerialPort;
};

#endif // SERIALCOMMUNICATOR_H
