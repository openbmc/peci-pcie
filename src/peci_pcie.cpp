/*
// Copyright (c) 2019 Intel Corporation
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

#include "peci_pcie.hpp"

#include "pciDeviceClass.hpp"
#include "pciVendors.hpp"

#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>

namespace peci_pcie
{
static boost::container::flat_map<
    int, boost::container::flat_map<
             int, boost::container::flat_map<
                      int, std::shared_ptr<sdbusplus::asio::dbus_interface>>>>
    pcieDeviceDBusMap;

static bool abortScan;

namespace function
{
static constexpr char const* functionTypeName = "FunctionType";
static constexpr char const* deviceClassName = "DeviceClass";
static constexpr char const* vendorIdName = "VendorId";
static constexpr char const* deviceIdName = "DeviceId";
static constexpr char const* classCodeName = "ClassCode";
static constexpr char const* revisionIdName = "RevisionId";
static constexpr char const* subsystemIdName = "SubsystemId";
static constexpr char const* subsystemVendorIdName = "SubsystemVendorId";
} // namespace function

static constexpr const std::array pciConfigInfo{
    std::tuple<const char*, int, int>{function::functionTypeName, -1, -1},
    std::tuple<const char*, int, int>{function::deviceClassName, -1, -1},
    std::tuple<const char*, int, int>{function::vendorIdName, 0, 2},
    std::tuple<const char*, int, int>{function::deviceIdName, 2, 2},
    std::tuple<const char*, int, int>{function::classCodeName, 9, 3},
    std::tuple<const char*, int, int>{function::revisionIdName, 8, 1},
    std::tuple<const char*, int, int>{function::subsystemIdName, 0x2e, 2},
    std::tuple<const char*, int, int>{function::subsystemVendorIdName, 0x2c,
                                      2}};
} // namespace peci_pcie

enum class resCode
{
    resOk,
    resSkip,
    resErr
};

struct CPUInfo
{
    size_t addr;
    bool skipCpuBuses;
    boost::container::flat_set<size_t> cpuBusNums;
};

// PECI Client Address Map
static resCode getCPUBusMap(std::vector<CPUInfo>& cpuInfo)
{
    cpuInfo.clear();
    for (size_t addr = MIN_CLIENT_ADDR; addr <= MAX_CLIENT_ADDR; addr++)
    {
        if (peci_Ping(addr) != PECI_CC_SUCCESS)
        {
            continue;
        }

        auto& cpu = cpuInfo.emplace_back(CPUInfo{addr, false, {}});
        uint8_t cc = 0;
        CPUModel model{};
        uint8_t stepping = 0;
        if (peci_GetCPUID(addr, &model, &stepping, &cc) != PECI_CC_SUCCESS)
        {
            std::cerr << "Cannot get CPUID!\n";
            return resCode::resErr;
        }

        switch (model)
        {
            case skx:
            {
                // Get the assigned CPU bus numbers from CPUBUSNO and CPUBUSNO1
                // (B(0) D8 F2 offsets CCh and D0h)
                uint32_t cpuBusNum = 0;
                if (peci_RdPCIConfigLocal(addr, 0, 8, 2, 0xCC, 4,
                                          (uint8_t*)&cpuBusNum,
                                          &cc) != PECI_CC_SUCCESS)
                {
                    return resCode::resErr;
                }
                uint32_t cpuBusNum1 = 0;
                if (peci_RdPCIConfigLocal(addr, 0, 8, 2, 0xD0, 4,
                                          (uint8_t*)&cpuBusNum1,
                                          &cc) != PECI_CC_SUCCESS)
                {
                    return resCode::resErr;
                }

                // Add the CPU bus numbers to the set for this CPU
                while (cpuBusNum)
                {
                    // Get the LSB
                    size_t busNum = cpuBusNum & 0xFF;
                    cpu.cpuBusNums.insert(busNum);
                    // Shift right by one byte
                    cpuBusNum >>= 8;
                }
                while (cpuBusNum1)
                {
                    // Get the LSB
                    size_t busNum = cpuBusNum1 & 0xFF;
                    cpu.cpuBusNums.insert(busNum);
                    // Shift right by one byte
                    cpuBusNum1 >>= 8;
                }
                cpu.skipCpuBuses = true;
            }
        }
    }
    return cpuInfo.empty() ? resCode::resErr : resCode::resOk;
}

static bool isPECIAvailable(void)
{
    for (size_t i = MIN_CLIENT_ADDR; i <= MAX_CLIENT_ADDR; i++)
    {
        if (peci_Ping(i) == PECI_CC_SUCCESS)
        {
            return true;
        }
    }
    return false;
}

static resCode getDataFromPCIeConfig(const int& clientAddr, const int& bus,
                                     const int& dev, const int& func,
                                     const int& offset, const int& size,
                                     uint32_t& pciData)
{
    // PECI RdPCIConfig() currently only supports 4 byte reads, so adjust
    // the offset and size to get the right data
    static constexpr const int pciReadSize = 4;
    int mod = offset % pciReadSize;
    int pciOffset = offset - mod;
    if (mod + size > pciReadSize)
    {
        return resCode::resErr;
    }

    std::array<uint8_t, pciReadSize> data;
    uint8_t cc;
    int ret = PECI_CC_TIMEOUT;
    for (int index = 0; (index < 5) && (ret == PECI_CC_TIMEOUT); index++)
    {
#ifdef USE_RDENDPOINTCFG
        ret = peci_RdEndPointConfigPci(clientAddr,  // CPU Address
                                       0,           // PCI Seg (use 0 for now)
                                       bus,         // PCI Bus
                                       dev,         // PCI Device
                                       func,        // PCI Function
                                       pciOffset,   // PCI Offset
                                       pciReadSize, // PCI Read Size
                                       data.data(), // PCI Read Data
                                       &cc);        // PECI Completion Code
#else
        ret = peci_RdPCIConfig(clientAddr,  // CPU Address
                               bus,         // PCI Bus
                               dev,         // PCI Device
                               func,        // PCI Function
                               pciOffset,   // PCI Offset
                               data.data(), // PCI Read Data
                               &cc);        // PECI Completion Code
#endif
    }
    if (ret != PECI_CC_SUCCESS || cc != PECI_DEV_CC_SUCCESS)
    {
        return resCode::resErr;
    }

    // Now build the requested data into a single number
    pciData = 0;
    for (int i = mod; i < mod + size; i++)
    {
        pciData |= static_cast<uint32_t>(data[i]) << 8 * (i - mod);
    }

    return resCode::resOk;
}

static resCode getStringFromData(const int& size, const uint32_t& data,
                                 std::string& res)
{
    // And convert it to a string
    std::stringstream dataStream;
    dataStream << "0x" << std::hex << std::setfill('0') << std::setw(size * 2)
               << data;
    res = dataStream.str();
    return resCode::resOk;
}

static resCode getVendorName(const int& clientAddr, const int& bus,
                             const int& dev, std::string& res)
{
    static constexpr const int vendorIDOffset = 0x00;
    static constexpr const int vendorIDSize = 2;

    // Get the header type register from function 0
    uint32_t vendorID = 0;
    if (getDataFromPCIeConfig(clientAddr, bus, dev, 0, vendorIDOffset,
                              vendorIDSize, vendorID) != resCode::resOk)
    {
        return resCode::resErr;
    }
    // Get the vendor name or use Other if it doesn't exist
    res = pciVendors.try_emplace(vendorID, otherVendor).first->second;
    return resCode::resOk;
}

static resCode getDeviceClass(const int& clientAddr, const int& bus,
                              const int& dev, const int& func, std::string& res)
{
    static constexpr const int baseClassOffset = 0x0b;
    static constexpr const int baseClassSize = 1;

    // Get the Device Base Class
    uint32_t baseClass = 0;
    if (getDataFromPCIeConfig(clientAddr, bus, dev, func, baseClassOffset,
                              baseClassSize, baseClass) != resCode::resOk)
    {
        return resCode::resErr;
    }
    // Get the base class name or use Other if it doesn't exist
    res = pciDeviceClasses.try_emplace(baseClass, otherClass).first->second;
    return resCode::resOk;
}

static resCode getCapReading(const int& clientAddr, const int& bus,
                             const int& dev, uint32_t& capReading,
                             const int compareCapID, const int offsetAddress,
                             const int offsetLength)
{
    resCode error;
    uint32_t capAddress = 0;
    uint32_t capabilityID;
    uint32_t nextCapPointer = peci_pcie::pointToCapStruct;

    do
    {
        // Get capability address
        error = getDataFromPCIeConfig(clientAddr, bus, dev, 0, nextCapPointer,
                                      1, capAddress);
        if (error != resCode::resOk)
        {
            return error;
        }
        // Capability struct address is a pointer which point to next capability
        // struct, so if capability struct address is 0 means it doesn't have
        // next capability struct.
        if (capAddress == 0)
        {
            return resCode::resSkip;
        }
        // Get capability ID
        error = getDataFromPCIeConfig(clientAddr, bus, dev, 0, capAddress, 1,
                                      capabilityID);
        if (error != resCode::resOk)
        {
            return error;
        }
        nextCapPointer = capAddress + peci_pcie::capPointerOffset;

    } while (capabilityID != compareCapID);
    // Get capability reading.
    error = getDataFromPCIeConfig(clientAddr, bus, dev, 0,
                                  capAddress + offsetAddress, offsetLength,
                                  capReading);
    if (error != resCode::resOk)
    {
        return error;
    }
    return resCode::resOk;
}

static resCode getGenerationInUse(const int& clientAddr, const int& bus,
                                  const int& dev, std::string& generationInUse)
{
    resCode error;
    std::string res;
    uint32_t capReading = 0;

    // Capability ID 0x10(16) is PCI Express
    constexpr int pcieCapID = 16;
    constexpr int capLength = 2;
    uint32_t linkStatus;

    error = getCapReading(clientAddr, bus, dev, linkStatus, pcieCapID,
                          peci_pcie::linkStatusOffset, capLength);
    if (error != resCode::resOk)
    {
        return error;
    }

    uint32_t linkSpeed = linkStatus & peci_pcie::maskOfCLS;

    switch (linkSpeed)
    {
        case peci_pcie::pcieGen1:
            generationInUse = "xyz.openbmc_project.Inventory.Item."
                              "PCIeSlot.Generations.Gen1";
            error = resCode::resOk;
            break;
        case peci_pcie::pcieGen2:
            generationInUse = "xyz.openbmc_project.Inventory.Item."
                              "PCIeSlot.Generations.Gen2";
            error = resCode::resOk;
            break;
        case peci_pcie::pcieGen3:
            generationInUse = "xyz.openbmc_project.Inventory.Item."
                              "PCIeSlot.Generations.Gen3";
            error = resCode::resOk;
            break;
        case peci_pcie::pcieGen4:
            generationInUse = "xyz.openbmc_project.Inventory.Item."
                              "PCIeSlot.Generations.Gen4";
            error = resCode::resOk;
            break;
        case peci_pcie::pcieGen5:
            generationInUse = "xyz.openbmc_project.Inventory.Item."
                              "PCIeSlot.Generations.Gen5";
            error = resCode::resOk;
            break;
        default:
            std::cerr << "Link speed : " << linkSpeed
                      << " can not mapping to PCIe type list.\n";
            error = resCode::resSkip;
    }
    return error;
}

static resCode isMultiFunction(const int& clientAddr, const int& bus,
                               const int& dev, bool& res)
{
    static constexpr const int headerTypeOffset = 0x0e;
    static constexpr const int headerTypeSize = 1;
    static constexpr const int multiFuncBit = 1 << 7;

    res = false;
    // Get the header type register from function 0
    uint32_t headerType = 0;
    if (getDataFromPCIeConfig(clientAddr, bus, dev, 0, headerTypeOffset,
                              headerTypeSize, headerType) != resCode::resOk)
    {
        return resCode::resErr;
    }
    // Check if it's a multifunction device
    if (headerType & multiFuncBit)
    {
        res = true;
    }
    return resCode::resOk;
}

static resCode pcieFunctionExists(const int& clientAddr, const int& bus,
                                  const int& dev, const int& func, bool& res)
{
    constexpr const int pciIDOffset = 0;
    constexpr const int pciIDSize = 4;
    uint32_t pciID = 0;
    res = false;
    if (getDataFromPCIeConfig(clientAddr, bus, dev, func, pciIDOffset,
                              pciIDSize, pciID) != resCode::resOk)
    {
        return resCode::resOk;
    }

    // if VID and DID are all 0s or 1s, then the device doesn't exist
    if (pciID != 0x00000000 && pciID != 0xFFFFFFFF)
    {
        res = true;
    }
    return resCode::resOk;
}

static resCode pcieDeviceExists(const int& clientAddr, const int& bus,
                                const int& dev, bool& res)
{
    // Check if this device exists by checking function 0
    return pcieFunctionExists(clientAddr, bus, dev, 0, res);
}

static resCode setPCIeProperty(const int& clientAddr, const int& bus,
                               const int& dev, const std::string& propertyName,
                               const std::string& propertyValue)
{
    std::shared_ptr<sdbusplus::asio::dbus_interface> iface =
        peci_pcie::pcieDeviceDBusMap[clientAddr][bus][dev];

    if (iface->is_initialized())
    {
        if (!iface->set_property(propertyName, propertyValue))
            return resCode::resErr;
    }
    else
    {
        if (!iface->register_property(propertyName, propertyValue))
            return resCode::resErr;
    }
    return resCode::resOk;
}

static void setDefaultPCIeFunctionProperties(const int& clientAddr,
                                             const int& bus, const int& dev,
                                             const int& func)
{
    // Set the function-specific properties
    for (const auto& [name, offset, size] : peci_pcie::pciConfigInfo)
    {
        setPCIeProperty(clientAddr, bus, dev,
                        "Function" + std::to_string(func) + std::string(name),
                        std::string());
    }
}

static resCode setPCIeFunctionProperties(const int& clientAddr, const int& bus,
                                         const int& dev, const int& func)
{
    uint32_t data = 0;
    std::string res;
    resCode error;

    for (const auto& [name, offset, size] : peci_pcie::pciConfigInfo)
    {
        if (offset < 0)
        {
            continue;
        }

        error = getDataFromPCIeConfig(clientAddr, bus, dev, func, offset, size,
                                      data);
        if (error != resCode::resOk)
        {
            return error;
        }
        getStringFromData(size, data, res);
        setPCIeProperty(clientAddr, bus, dev,
                        "Function" + std::to_string(func) + std::string(name),
                        res);
    }

    // Set the function type always to physical for now
    setPCIeProperty(clientAddr, bus, dev,
                    "Function" + std::to_string(func) +
                        std::string(peci_pcie::function::functionTypeName),
                    "Physical");

    // Set the function Device Class
    error = getDeviceClass(clientAddr, bus, dev, func, res);
    if (error != resCode::resOk)
    {
        return error;
    }
    setPCIeProperty(clientAddr, bus, dev,
                    "Function" + std::to_string(func) +
                        std::string(peci_pcie::function::deviceClassName),
                    res);
    return resCode::resOk;
}

static resCode setPCIeDeviceProperties(const int& clientAddr, const int& bus,
                                       const int& dev)
{
    // Set the device manufacturer
    std::string manuf;
    resCode error = getVendorName(clientAddr, bus, dev, manuf);
    if (error != resCode::resOk)
    {
        return error;
    }
    setPCIeProperty(clientAddr, bus, dev, "Manufacturer", manuf);

    // Set the device type
    constexpr char const* deviceTypeName = "DeviceType";
    bool multiFunc;
    error = isMultiFunction(clientAddr, bus, dev, multiFunc);
    if (error != resCode::resOk)
    {
        return error;
    }
    if (multiFunc)
    {
        setPCIeProperty(clientAddr, bus, dev, deviceTypeName, "MultiFunction");
    }
    else
    {
        setPCIeProperty(clientAddr, bus, dev, deviceTypeName, "SingleFunction");
    }

    // Set PCIe Generation
    constexpr char const* generationInUseName = "GenerationInUse";
    std::string generationInUse;
    error = getGenerationInUse(clientAddr, bus, dev, generationInUse);
    if (error == resCode::resErr)
    {
        return error;
    }
    // "resSkip" status means it can't get the capability reading, such like
    // this device is not PCI Express.
    if (error == resCode::resSkip)
    {
        setPCIeProperty(clientAddr, bus, dev, generationInUseName, "");
        return resCode::resOk;
    }
    setPCIeProperty(clientAddr, bus, dev, generationInUseName, generationInUse);

    return resCode::resOk;
}

static resCode updatePCIeDevice(const int& clientAddr, const int& bus,
                                const int& dev)
{
    if (setPCIeDeviceProperties(clientAddr, bus, dev) != resCode::resOk)
    {
        return resCode::resErr;
    }

    // Walk through and populate the functions for this device
    for (int func = 0; func < peci_pcie::maxPCIFunctions; func++)
    {
        bool res;
        resCode error = pcieFunctionExists(clientAddr, bus, dev, func, res);
        if (error != resCode::resOk)
        {
            return error;
        }
        if (res)
        {
            // Set the properties for this function
            if (setPCIeFunctionProperties(clientAddr, bus, dev, func) !=
                resCode::resOk)
            {
                return resCode::resErr;
            }
        }
        else
        {
            // Set default properties for unused functions
            setDefaultPCIeFunctionProperties(clientAddr, bus, dev, func);
        }
    }
    return resCode::resOk;
}

static void removePCIeDevice(sdbusplus::asio::object_server& objServer,
                             const int& clientAddr, const int& bus,
                             const int& dev)
{
    std::shared_ptr<sdbusplus::asio::dbus_interface> iface =
        peci_pcie::pcieDeviceDBusMap[clientAddr][bus][dev];

    objServer.remove_interface(iface);

    peci_pcie::pcieDeviceDBusMap[clientAddr][bus].erase(dev);
    if (peci_pcie::pcieDeviceDBusMap[clientAddr][bus].empty())
    {
        peci_pcie::pcieDeviceDBusMap[clientAddr].erase(bus);
    }
    if (peci_pcie::pcieDeviceDBusMap[clientAddr].empty())
    {
        peci_pcie::pcieDeviceDBusMap.erase(clientAddr);
    }
}

static resCode addPCIeDevice(sdbusplus::asio::object_server& objServer,
                             const int& clientAddr, const int& cpu,
                             const int& bus, const int& dev)
{
    std::string pathName = std::string(peci_pcie::peciPCIePath) + "/S" +
                           std::to_string(cpu) + "B" + std::to_string(bus) +
                           "D" + std::to_string(dev);
    std::shared_ptr<sdbusplus::asio::dbus_interface> iface =
        objServer.add_interface(pathName, peci_pcie::peciPCIeDeviceInterface);
    peci_pcie::pcieDeviceDBusMap[clientAddr][bus][dev] = iface;

    // Update the properties for the new device
    if (updatePCIeDevice(clientAddr, bus, dev) != resCode::resOk)
    {
        removePCIeDevice(objServer, clientAddr, bus, dev);
        return resCode::resErr;
    }

    iface->initialize();
    return resCode::resOk;
}

static bool pcieDeviceInDBusMap(const int& clientAddr, const int& bus,
                                const int& dev)
{
    if (auto clientAddrIt = peci_pcie::pcieDeviceDBusMap.find(clientAddr);
        clientAddrIt != peci_pcie::pcieDeviceDBusMap.end())
    {
        if (auto busIt = clientAddrIt->second.find(bus);
            busIt != clientAddrIt->second.end())
        {
            if (auto devIt = busIt->second.find(dev);
                devIt != busIt->second.end())
            {
                if (devIt->second)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

static resCode probePCIeDevice(boost::asio::io_service& io,
                               sdbusplus::asio::object_server& objServer,
                               size_t addr, int cpu, int bus, int dev)
{
    bool res;
    resCode error = pcieDeviceExists(addr, bus, dev, res);
    if (error != resCode::resOk)
    {
        return error;
    }
    if (res)
    {
        if (pcieDeviceInDBusMap(addr, bus, dev))
        {
            // This device is already in D-Bus, so update it
            if (updatePCIeDevice(addr, bus, dev) != resCode::resOk)
            {
                return resCode::resErr;
            }
        }
        else
        {
            // This device is not in D-Bus, so add it
            if (addPCIeDevice(objServer, addr, cpu, bus, dev) != resCode::resOk)
            {
                return resCode::resErr;
            }
        }
    }
    else
    {
        // If PECI is not available, then stop scanning
        if (!isPECIAvailable())
        {
            return resCode::resOk;
        }

        if (pcieDeviceInDBusMap(addr, bus, dev))
        {
            // This device is in D-Bus, so remove it
            removePCIeDevice(objServer, addr, bus, dev);
        }
    }
    return resCode::resOk;
}

static void scanNextPCIeDevice(boost::asio::io_service& io,
                               sdbusplus::asio::object_server& objServer,
                               std::vector<CPUInfo>& cpuInfo, int cpu, int bus,
                               int dev);
static void scanPCIeDevice(boost::asio::io_service& io,
                           sdbusplus::asio::object_server& objServer,
                           std::vector<CPUInfo>& cpuInfo, int cpu, int bus,
                           int dev)
{
    if (cpu >= cpuInfo.size())
    {
        std::cerr << "Request to scan CPU" << cpu
                  << " while CPU array has size " << cpuInfo.size() << "\n";
        return;
    }
    auto& info = cpuInfo[cpu];
    // Check if this is a CPU bus that we should skip
    if (info.skipCpuBuses && info.cpuBusNums.count(bus))
    {
        std::cout << "Skipping CPU " << cpu << " Bus Number " << bus << "\n";
        // Skip all the devices on this bus
        dev = peci_pcie::maxPCIDevices;
    }
    else
    {
        if (probePCIeDevice(io, objServer, info.addr, cpu, bus, dev) !=
            resCode::resOk)
        {
            std::cerr << "Failed to probe CPU " << cpu << " Bus " << bus
                      << " Device " << dev << "\n";
        }
    }

    scanNextPCIeDevice(io, objServer, cpuInfo, cpu, bus, dev);
    return;
}

static void scanNextPCIeDevice(boost::asio::io_service& io,
                               sdbusplus::asio::object_server& objServer,
                               std::vector<CPUInfo>& cpuInfo, int cpu, int bus,
                               int dev)
{
    if (peci_pcie::abortScan)
    {
        std::cerr << "PCIe scan aborted\n";
        return;
    }

    // PCIe Device scan completed, so move to the next device
    if (++dev >= peci_pcie::maxPCIDevices)
    {
        // All devices scanned, so move to the next bus
        dev = 0;
        if (++bus >= peci_pcie::maxPCIBuses)
        {
            // All buses scanned, so move to the next CPU
            bus = 0;
            if (++cpu >= cpuInfo.size())
            {
                // All CPUs scanned, so we're done
                std::cerr << "PCIe scan completed\n";
                return;
            }
        }
    }
    boost::asio::post(io, [&io, &objServer, &cpuInfo, cpu, bus, dev]() mutable {
        scanPCIeDevice(io, objServer, cpuInfo, cpu, bus, dev);
    });
}

static void peciAvailableCheck(boost::asio::steady_timer& peciWaitTimer,
                               boost::asio::io_service& io,
                               sdbusplus::asio::object_server& objServer,
                               std::vector<CPUInfo>& cpuInfo)
{
    static bool lastPECIState = false;
    bool peciAvailable = isPECIAvailable();
    if (peciAvailable && !lastPECIState)
    {
        lastPECIState = true;
        static boost::asio::steady_timer pcieTimeout(io);
        constexpr const int pcieWaitTime = 60;
        pcieTimeout.expires_after(std::chrono::seconds(pcieWaitTime));
        pcieTimeout.async_wait(
            [&io, &objServer, &cpuInfo](const boost::system::error_code& ec) {
                if (ec)
                {
                    // operation_aborted is expected if timer is canceled
                    // before completion.
                    if (ec != boost::asio::error::operation_aborted)
                    {
                        std::cerr << "PECI PCIe async_wait failed " << ec;
                    }
                    lastPECIState = false;
                    return;
                }
                // get the PECI client address list
                if (getCPUBusMap(cpuInfo) != resCode::resOk)
                {
                    lastPECIState = false;
                    return;
                }
                // scan PCIe starting from CPU 0, Bus 0, Device 0
                std::cerr << "PCIe scan started\n";
                scanPCIeDevice(io, objServer, cpuInfo, 0, 0, 0);
            });
    }
    else if (!peciAvailable && lastPECIState)
    {
        lastPECIState = false;
    }

    peciWaitTimer.expires_after(
        std::chrono::seconds(peci_pcie::peciCheckInterval));
    peciWaitTimer.async_wait([&peciWaitTimer, &io, &objServer,
                              &cpuInfo](const boost::system::error_code& ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled
            // before completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::cerr << "PECI Available Check async_wait failed " << ec;
            }
            return;
        }
        peciAvailableCheck(peciWaitTimer, io, objServer, cpuInfo);
    });
}

static void waitForOSStandbyDelay(boost::asio::io_service& io,
                                  sdbusplus::asio::object_server& objServer,
                                  boost::asio::steady_timer& osStandbyTimer,
                                  std::vector<CPUInfo>& cpuInfo)
{
    osStandbyTimer.expires_after(
        std::chrono::seconds(peci_pcie::osStandbyDelaySeconds));

    osStandbyTimer.async_wait(
        [&io, &objServer, &cpuInfo](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return; // we're being canceled
            }
            else if (ec)
            {
                std::cerr << "OS Standby async_wait failed: " << ec.value()
                          << ": " << ec.message() << "\n";
                return;
            }
            // get the PECI client address list
            if (getCPUBusMap(cpuInfo) != resCode::resOk)
            {
                return;
            }
            // scan PCIe starting from CPU 0, Bus 0, Device 0
            std::cerr << "PCIe scan started\n";
            scanPCIeDevice(io, objServer, cpuInfo, 0, 0, 0);
        });
}

static void monitorOSStandby(boost::asio::io_service& io,
                             std::shared_ptr<sdbusplus::asio::connection> conn,
                             sdbusplus::asio::object_server& objServer,
                             boost::asio::steady_timer& osStandbyTimer,
                             std::vector<CPUInfo>& cpuInfo)
{
    std::cerr << "Start OperatingSystemState Monitor\n";

    static sdbusplus::bus::match::match osStateMatch(
        *conn,
        "type='signal',interface='org.freedesktop.DBus.Properties',member='"
        "PropertiesChanged',arg0='xyz.openbmc_project.State.OperatingSystem."
        "Status'",
        [&io, &objServer, &osStandbyTimer,
         &cpuInfo](sdbusplus::message::message& msg) {
            // Get the OS State from the message
            std::string osStateInterface;
            boost::container::flat_map<std::string, std::variant<std::string>>
                propertiesChanged;
            msg.read(osStateInterface, propertiesChanged);

            for (const auto& [name, value] : propertiesChanged)
            {
                if (name == "OperatingSystemState")
                {
                    const std::string* state = std::get_if<std::string>(&value);
                    if (state == nullptr)
                    {
                        std::cerr << "Unable to read OS state value\n";
                        return;
                    }

                    if ((*state == "Standby") ||
                        (*state == "xyz.openbmc_project.State.OperatingSystem."
                                   "Status.OSStatus.Standby"))
                    {
                        peci_pcie::abortScan = false;
                        waitForOSStandbyDelay(io, objServer, osStandbyTimer,
                                              cpuInfo);
                    }
                    else if ((*state == "Inactive") ||
                             (*state ==
                              "xyz.openbmc_project.State.OperatingSystem."
                              "Status.OSStatus.Inactive"))
                    {
                        peci_pcie::abortScan = true;
                        osStandbyTimer.cancel();
                    }
                }
            }
        });

    // Check if the OS state is already available
    conn->async_method_call(
        [&io, &objServer, &osStandbyTimer,
         &cpuInfo](boost::system::error_code ec,
                   const std::variant<std::string>& property) {
            if (ec)
            {
                std::cerr << "error with OS state async_method_call\n";
                return;
            }

            const std::string* state = std::get_if<std::string>(&property);
            if (state == nullptr)
            {
                std::cerr << "Unable to read OS state value\n";
                return;
            }

            // If the OS state is in Standby, then BIOS is done and we can
            // continue.  Otherwise, we just wait for the match
            if ((*state == "Standby") ||
                (*state == "xyz.openbmc_project.State.OperatingSystem.Status."
                           "OSStatus.Standby"))
            {
                waitForOSStandbyDelay(io, objServer, osStandbyTimer, cpuInfo);
            }
        },
        "xyz.openbmc_project.State.OperatingSystem",
        "/xyz/openbmc_project/state/os", "org.freedesktop.DBus.Properties",
        "Get", "xyz.openbmc_project.State.OperatingSystem.Status",
        "OperatingSystemState");
}

int main(int argc, char* argv[])
{
    // setup connection to dbus
    boost::asio::io_service io;
    std::shared_ptr<sdbusplus::asio::connection> conn =
        std::make_shared<sdbusplus::asio::connection>(io);

    // PECI PCIe Object
    conn->request_name(peci_pcie::peciPCIeObject);
    sdbusplus::asio::object_server server =
        sdbusplus::asio::object_server(conn);

    // CPU map
    std::vector<CPUInfo> cpuInfo;

#ifdef WAIT_FOR_OS_STANDBY
    boost::asio::steady_timer osStandbyTimer(io);
    monitorOSStandby(io, conn, server, osStandbyTimer, cpuInfo);
#else
    // Start the PECI check loop
    boost::asio::steady_timer peciWaitTimer(
        io, std::chrono::seconds(peci_pcie::peciCheckInterval));
    peciWaitTimer.async_wait([&peciWaitTimer, &io, &server,
                              &cpuInfo](const boost::system::error_code& ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled
            // before completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::cerr << "PECI Available Check async_wait failed " << ec;
            }
            return;
        }
        peciAvailableCheck(peciWaitTimer, io, server, cpuInfo);
    });
#endif

    io.run();

    return 0;
}
