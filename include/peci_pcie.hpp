/*
// Copyright (c) 2019 intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#pragma once
#include <peci.h>

#include <cstdint>
#include <vector>

namespace peci_pcie
{
static constexpr char const* peciPCIeObject = "xyz.openbmc_project.PCIe";
static constexpr char const* peciPCIePath =
    "/xyz/openbmc_project/inventory/pcie";
static constexpr char const* peciPCIeDeviceInterface =
    "xyz.openbmc_project.Inventory.Item.PCIeDevice";
static constexpr char const* peciPCIeAssetInterface =
    "xyz.openbmc_project.Inventory.Decorator.Asset";

static constexpr const int maxPCIBuses = 256;
static constexpr const int maxPCIDevices = 32;
static constexpr const int maxPCIFunctions = 8;

static constexpr const int peciCheckInterval = 10;
static constexpr const int osStandbyDelaySeconds = 10;

static constexpr const int pointToCapStruct = 0x34;
static constexpr const int maskOfCLS = 0x0F;

static constexpr const int capPointerOffset = 1;
static constexpr const int linkStatusOffset = 18;

// PCIe version
// GEN1 : 0001b : transfer rate 2.5GB
// GEN2 : 0010b : transfer rate 5GB
// GEN3 : 0011b : transfer rate 8GB
// GEN4 : 0100b : transfer rate 16GB
// GEN5 : 0101b : transfer rate 32GB
enum GenerationInUse : int
{
    pcieGen1 = 1,
    pcieGen2,
    pcieGen3,
    pcieGen4,
    pcieGen5
};
} // namespace peci_pcie
