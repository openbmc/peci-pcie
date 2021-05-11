/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021 YADRO.
 */

#pragma once

#include <string>

struct pciidInfo
{
    std::string vendorName;
    std::string deviceName;
    std::string subsystem;
};

bool pciidGetDevice(const uint32_t vid, const uint32_t did,
                    const uint32_t subvid, const uint32_t subdid,
                    pciidInfo& pciDevInfo);
bool pciidGetClass(const uint32_t classid, std::string& className);
