/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

////////////////////////////////////////////////////////////////////////////////
/// @file The file for Dynamixel Fast Bulk Read
/// @author Honghyun Kim
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPFASTBULKREAD_H
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPFASTBULKREAD_H


#include "group_bulk_read.h"

namespace dynamixel
{

class WINDECLSPEC GroupFastBulkRead final : public GroupBulkRead
{
public:
    GroupFastBulkRead(std::shared_ptr<PortHandler> port, PacketHandler *ph);
    ~GroupFastBulkRead() final { clearParam(); }

    int txPacket() final;
    int rxPacket() final;
    int txRxPacket() final;

private:
    void makeParam() final;
};

}


#endif // DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPFASTBULKREAD_H
