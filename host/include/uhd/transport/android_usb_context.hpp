//
// Copyright 2026 pixdr contributors
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <cstdint>
#include <string>

namespace uhd { namespace transport {

// Optional Android USB host context supplied by an embedding application.
//
// Android applications cannot enumerate /dev/bus/usb directly under normal
// SELinux policy. The app owns an already-authorized UsbDeviceConnection and
// injects its file descriptor here. Desktop builds simply do not provide a
// context and UHD continues to use normal libusb enumeration.
class android_usb_context
{
public:
    virtual ~android_usb_context() = default;

    virtual intptr_t fd() const = 0;
    virtual std::string usbfs_path() const = 0;
    virtual uint16_t vid() const = 0;
    virtual uint16_t pid() const = 0;
    virtual bool firmware_loaded() const = 0;
};

// Returns nullptr when no Android USB context is available or fd() < 0.
const android_usb_context* get_android_usb_context();

}} // namespace uhd::transport
