/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021 YADRO.
 */

#include "pci_ids.hpp"

#include <cctype>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>

namespace pci_ids
{
static std::map<uint64_t, pciidInfo> pciidsMap;
static std::map<uint32_t, std::string> pciidsClassMap;
} // namespace pci_ids

static constexpr char const* pciidsPath = "/usr/share/misc/pci.ids";

bool pciidDeviceLookup(const uint32_t vid, const uint32_t did,
                       const uint32_t subvid, const uint32_t subdid,
                       pciidInfo& pciDevInfo)
{
    pciDevInfo.vendorName = std::string();
    pciDevInfo.deviceName = std::string();
    pciDevInfo.subsystem = std::string();
    std::ifstream idfile(pciidsPath);
    if (!idfile.is_open())
    {
        std::cerr << "Failed to open pci.ids database \n";
        return false;
    }

    static const std::regex idRegex("[a-f0-9]{4}",
                                    std::regex::icase | std::regex::optimize);
    static const std::regex subsystemRegex(
        "[a-f0-9]{4} [a-f0-9]{4}", std::regex::icase | std::regex::optimize);
    std::string curVIDstr;
    std::string curDIDstr;
    std::string vidStr;
    std::string didStr;
    std::string subsystemStr;
    {
        char tmpStr[16];
        snprintf(tmpStr, sizeof(tmpStr), "%04x", vid);
        vidStr = tmpStr;
        snprintf(tmpStr, sizeof(tmpStr), "%04x", did);
        didStr = tmpStr;
        snprintf(tmpStr, sizeof(tmpStr), "%04x %04x", subvid, subdid);
        subsystemStr = tmpStr;
    }
    bool vendorFound = false;
    bool deviceFound = false;
    static const size_t wordLen = 4;
    static const size_t sepLen = 2;

    std::string line;
    while (std::getline(idfile, line))
    {
        if (line.empty() || (line[0] == '#'))
        {
            // Comment line, just ignore
        }
        else if ((line[0] == 'C') && (line[1] == ' '))
        {
            // Class line, main part of the file ended
            return true;
        }
        else if (isxdigit(line[0]))
        {
            // VendorID line
            if (deviceFound)
            {
                // Device found in last entry of previous section
                return true;
            }
            if (vendorFound)
            {
                std::cout << "Device " << didStr << " not found in section for "
                          << pciDevInfo.vendorName << ". \n";
                return true;
            }

            static const size_t prefixLen = 0;
            curVIDstr = line.substr(prefixLen, wordLen);
            if (!std::regex_match(curVIDstr, idRegex))
            {
                std::cerr << "Wrong vendor line format: " << line << " \n";
                continue;
            }

            int res = curVIDstr.compare(vidStr);
            if (res == 0)
            {
                pciDevInfo.vendorName =
                    line.substr(prefixLen + wordLen + sepLen);
                vendorFound = true;
                if (line.substr(prefixLen + wordLen, sepLen) != "  ")
                {
                    std::cerr << "Wrong vendor line format: " << line << " \n";
                }
            }
            else if (res > 0)
            {
                std::cout << "Vendor " << vidStr
                          << " not found. Break search on " << curVIDstr
                          << " \n";
                return true;
            }
        }
        else if ((line[0] == '\t') && isxdigit(line[1]))
        {
            // DeviceID line
            if (!vendorFound)
            {
                continue;
            }
            if (deviceFound)
            {
                // DeviceID found, but there is no subclass defined
                return true;
            }

            static const size_t prefixLen = 1;
            curDIDstr = line.substr(prefixLen, wordLen);
            if (!std::regex_match(curDIDstr, idRegex))
            {
                std::cerr << "Wrong device line format: " << line << " \n";
                continue;
            }

            int res = curDIDstr.compare(didStr);
            if (res == 0)
            {
                pciDevInfo.deviceName =
                    line.substr(prefixLen + wordLen + sepLen);
                deviceFound = true;
                if (line.substr(prefixLen + wordLen, sepLen) != "  ")
                {
                    std::cerr << "Wrong device line format: " << line << " \n";
                }
                continue;
            }
            else if (res > 0)
            {
                std::cout << "Device " << didStr << " not found in section for "
                          << pciDevInfo.vendorName << ". Break search on "
                          << curDIDstr << ". \n";
                return true;
            }
        }
        else if ((line[0] == '\t') && (line[1] == '\t') && isxdigit(line[2]))
        {
            // Subsystem line
            if (!deviceFound)
            {
                continue;
            }

            static const size_t prefixLen = 2;
            auto curSubsystemStr = line.substr(prefixLen, wordLen * 2 + 1);
            if (!std::regex_match(curSubsystemStr, subsystemRegex))
            {
                std::cerr << "Wrong subsystem line format: " << line << " \n";
                continue;
            }

            int res = curSubsystemStr.compare(subsystemStr);
            if (res == 0)
            {
                pciDevInfo.subsystem =
                    line.substr(prefixLen + wordLen * 2 + 1 + sepLen);
                return true;
            }
            else if (res > 0)
            {
                // if no subsystem found, DID is enough
                return true;
            }
        }
        else
        {
            std::cerr << "Unexpected line format: " << line << " \n";
        }
    }
    return true;
}

bool pciidClassLookup(const uint32_t devClass, const uint32_t devSubClass,
                      std::string& className)
{
    className = std::string();
    std::ifstream idfile(pciidsPath);
    if (!idfile.is_open())
    {
        std::cerr << "Failed to open pci.ids database \n";
        return false;
    }

    static const std::regex idRegex("[a-f0-9]{2}",
                                    std::regex::icase | std::regex::optimize);
    std::string curCIDstr;
    std::string curSCIDstr;
    std::string cidStr;
    std::string scidStr;
    {
        char tmpStr[16];
        snprintf(tmpStr, sizeof(tmpStr), "%02x", devClass);
        cidStr = tmpStr;
        snprintf(tmpStr, sizeof(tmpStr), "%02x", devSubClass);
        scidStr = tmpStr;
    }
    bool classFound = false;
    static const size_t wordLen = 2;
    static const size_t sepLen = 2;

    std::string line;
    while (std::getline(idfile, line))
    {
        if (line.empty() || (line[0] == '#'))
        {
            // Comment line, just ignore
        }
        else if ((line[0] == 'C') && (line[1] == ' '))
        {
            // Class line
            if (classFound)
            {
                std::cout << "Subclass " << scidStr
                          << " not found in section for " << className
                          << ". \n";
                return true;
            }

            static const size_t prefixLen = 2;
            curCIDstr = line.substr(prefixLen, wordLen);
            if (!std::regex_match(curCIDstr, idRegex))
            {
                std::cerr << "Wrong class line format: " << line << " \n";
                continue;
            }

            int res = curCIDstr.compare(cidStr);
            if (res == 0)
            {
                className = line.substr(prefixLen + wordLen + sepLen);
                classFound = true;
                if (line.substr(prefixLen + wordLen, sepLen) != "  ")
                {
                    std::cerr << "Wrong class line format: " << line << " \n";
                }
            }
            else if (res > 0)
            {
                std::cout << "Class " << cidStr
                          << " not found. Break search on " << curCIDstr
                          << " \n";
                return true;
            }
        }
        else if ((line[0] == '\t') && isxdigit(line[1]))
        {
            // SubClassID line
            if (!classFound)
            {
                continue;
            }

            static const size_t prefixLen = 1;
            curSCIDstr = line.substr(prefixLen, wordLen);
            if (!std::regex_match(curSCIDstr, idRegex))
            {
                std::cerr << "Wrong subclass line format: " << line << " \n";
                continue;
            }

            int res = curSCIDstr.compare(scidStr);
            if (res == 0)
            {
                className = line.substr(prefixLen + wordLen + sepLen);
                if (line.substr(prefixLen + wordLen, sepLen) != "  ")
                {
                    std::cerr << "Wrong subclass line format: " << line
                              << " \n";
                }
                return true;
            }
            else if (res > 0)
            {
                std::cout << "Subclass " << scidStr
                          << " not found in section for " << className
                          << ". Break search on " << curCIDstr << ". \n";
                return true;
            }
        }
        else if (isxdigit(line[0]))
        {
            // VendorID line, ignore here
        }
        else if ((line[0] == '\t') && (line[1] == '\t') && isxdigit(line[2]))
        {
            // Subsystem or prog-if line, ignore here
        }
        else
        {
            std::cerr << "Unexpected line format: " << line << " \n";
        }
    }
    return true;
}

static uint64_t cacheKey(const uint32_t vid, const uint32_t did,
                         const uint32_t subvid, const uint32_t subdid)
{
    uint64_t key = 0;
    key |= (vid & 0xFFFFLL) << 48;
    key |= (did & 0xFFFFLL) << 32;
    key |= (subvid & 0xFFFFLL) << 16;
    key |= (subdid & 0xFFFFLL);
    return key;
}

bool pciidGetDevice(const uint32_t vid, const uint32_t did,
                    const uint32_t subvid, const uint32_t subdid,
                    pciidInfo& pciDevInfo)
{
    // if no pci.ids file present, don't try to use it in future
    static bool pciidsFileFound = true;
    if (!pciidsFileFound)
    {
        return false;
    }
    uint64_t key = cacheKey(vid, did, subvid, subdid);
    auto it = pci_ids::pciidsMap.find(key);
    if (it != pci_ids::pciidsMap.end())
    {
        pciDevInfo = it->second;
    }
    else
    {
        if (pciidDeviceLookup(vid, did, subvid, subdid, pciDevInfo))
        {
            pci_ids::pciidsMap[key] = pciDevInfo;
        }
        else
        {
            pciidsFileFound = false;
        }
    }
    return pciidsFileFound;
}

bool pciidGetClass(const uint32_t classid, std::string& className)
{
    // if no pci.ids file present, don't try to use it in future
    static bool pciidsFileFound = true;
    if (!pciidsFileFound)
    {
        return false;
    }
    uint32_t key = classid;
    uint32_t cid = (classid >> 16) & 0xFF;
    uint32_t scid = (classid >> 8) & 0xFF;
    auto it = pci_ids::pciidsClassMap.find(key);
    if (it != pci_ids::pciidsClassMap.end())
    {
        className = it->second;
    }
    else
    {
        if (pciidClassLookup(cid, scid, className))
        {
            pci_ids::pciidsClassMap[key] = className;
        }
        else
        {
            pciidsFileFound = false;
        }
    }
    return pciidsFileFound;
}
