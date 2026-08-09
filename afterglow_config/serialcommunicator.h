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
