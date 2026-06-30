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

/* Author: zerom, Ryu Woon Jung (Leon) */

#include <stdio.h>
#include <algorithm>

#if defined(__linux__)
#include "group_bulk_read.h"
#elif defined(__APPLE__)
#include "group_bulk_read.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "group_bulk_read.h"
#elif defined(ARDUINO) || defined(__OPENCR__) || defined(__OPENCM904__) || defined(ARDUINO_OpenRB)
#include "../../include/dynamixel_sdk/group_bulk_read.h"
#endif

using namespace dynamixel;

GroupBulkRead::GroupBulkRead(std::shared_ptr<PortHandler> port, PacketHandler *ph)
  : GroupHandler(std::move(port), ph),
    last_result_(false)
{

} 

void GroupBulkRead::makeParam()
{
  if (id_list_.size() == 0)
    return;

  delete[] param_;
  param_ = nullptr;

  if (ph_->getProtocolVersion() == 1.0)
  {
    param_ = new uint8_t[id_list_.size() * 3];  // ID(1) + ADDR(1) + LENGTH(1)
  }
  else    // 2.0
  {
    param_ = new uint8_t[id_list_.size() * 5];  // ID(1) + ADDR(2) + LENGTH(2)
  }

  int idx = 0;
  for (unsigned int i = 0; i < id_list_.size(); i++)
  {
    uint8_t id = id_list_[i];
    auto [address, length] = read_param_list_[id];
    if (ph_->getProtocolVersion() == 1.0)
    {
      param_[idx++] = static_cast<uint8_t>(length);    // LEN
      param_[idx++] = id;                           // ID
      param_[idx++] = static_cast<uint8_t>(address);   // ADDR
    }
    else    // 2.0
    {
      param_[idx++] = id;                               // ID
      param_[idx++] = DXL_LOBYTE(address);    // ADDR_L
      param_[idx++] = DXL_HIBYTE(address);    // ADDR_H
      param_[idx++] = DXL_LOBYTE(length);     // LEN_L
      param_[idx++] = DXL_HIBYTE(length);     // LEN_H
    }
  }
}

bool GroupBulkRead::addParam(uint8_t id, uint16_t start_address, uint16_t data_length)
{
  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
    return false;

  id_list_.push_back(id);
  read_param_list_[id] = {.start_address=start_address, .data_length=data_length};
  // TODO(jack): Better solution here... see: feature-cpp-raii?
  // Personally i'd rather avoid allocatiosn to keep memory fragmentaiton low, especially for "fast" versions
  data_list_[id]      = new uint8_t[data_length]; 
  error_list_[id]     = {};

  is_param_changed_   = true;
  return true;
}

void GroupBulkRead::removeParam(uint8_t id)
{
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return;

  id_list_.erase(it);
  read_param_list_.erase(id);
  delete[] data_list_[id];
  data_list_.erase(id);
  error_list_.erase(id);

  is_param_changed_   = true;
}

int GroupBulkRead::txPacket()
{
  if (id_list_.size() == 0)
    return COMM_NOT_AVAILABLE;

  if (is_param_changed_ == true || param_ == nullptr)
    makeParam();

  if (ph_->getProtocolVersion() == 1.0)
  {
    return ph_->bulkReadTx(port_, param_, id_list_.size() * 3);
  }
  else    // 2.0
  {
    return ph_->bulkReadTx(port_, param_, id_list_.size() * 5);
  }
}

int GroupBulkRead::rxPacket()
{
  int cnt            = id_list_.size();
  int result          = COMM_RX_FAIL;

  last_result_ = false;

  if (cnt == 0)
    return COMM_NOT_AVAILABLE;

  for (int i = 0; i < cnt; i++)
  {
    uint8_t id = id_list_[i];
    uint16_t length = read_param_list_[id].data_length;
    result = ph_->readRx(port_, id, length, data_list_[id], &error_list_[id]);
    if (result != COMM_SUCCESS)
      return result;
  }

  if (result == COMM_SUCCESS)
    last_result_ = true;

  return result;
}

int GroupBulkRead::txRxPacket()
{
  int result         = COMM_TX_FAIL;

  result = txPacket();
  if (result != COMM_SUCCESS)
    return result;

  return rxPacket();
}

bool GroupBulkRead::isAvailable(uint8_t id, uint16_t address, uint16_t data_length)
{
  if (last_result_ == false || data_list_.find(id) == data_list_.end())
    return false;

  auto [start_addr, length] = read_param_list_[id];

  if (address < start_addr || start_addr + length - data_length < address)
    return false;

  return true;
}

uint32_t GroupBulkRead::getData(uint8_t id, uint16_t address, uint16_t data_length)
{
  if (isAvailable(id, address, data_length) == false)
    return 0;

  uint16_t start_addr = read_param_list_[id].start_address;
  uint8_t *data = data_list_[id];
  switch(data_length)
  {
    case 1:
      return data[address - start_addr];

    case 2:
      return DXL_MAKEWORD(data[address - start_addr], data[address - start_addr + 1]);

    case 4:
      return DXL_MAKEDWORD(DXL_MAKEWORD(data[address - start_addr + 0], data[address - start_addr + 1]),
                           DXL_MAKEWORD(data[address - start_addr + 2], data[address - start_addr + 3]));

    default:
      return 0;
  }
}

bool GroupBulkRead::getError(uint8_t id, uint8_t* error)
{
  if (ph_->getProtocolVersion() == 1.0 || !last_result_ || error_list_.find(id) == error_list_.end())
    return false;

  *error = error_list_[id];
  return true;
}
