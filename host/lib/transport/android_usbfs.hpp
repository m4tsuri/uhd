//
// Copyright 2026 pixdr contributors
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/transport/usb_control.hpp>
#include <uhd/transport/usb_device_handle.hpp>
#include <uhd/transport/usb_zero_copy.hpp>
#include <uhd/types/device_addr.hpp>
#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <string>

namespace uhd { namespace transport {

class android_usbfs_device_handle : public usb_device_handle
{
public:
    typedef std::shared_ptr<android_usbfs_device_handle> sptr;

    android_usbfs_device_handle(
        int fd, uint16_t vid, uint16_t pid, std::string usbfs_path, bool firmware_loaded);
    ~android_usbfs_device_handle() override;

    std::string get_serial() const override;
    std::string get_manufacturer() const override;
    std::string get_product() const override;
    uint16_t get_vendor_id() const override;
    uint16_t get_product_id() const override;
    bool firmware_loaded() override;

    int fd() const;
    const std::string& usbfs_path() const;

    void claim_interface(int interface);
    void release_claimed_interfaces();
    void clear_endpoint(unsigned char endpoint);
    void clear_endpoints(unsigned char recv_endpoint, unsigned char send_endpoint);
    void reset_device();
    int control_transfer(uint8_t request_type,
        uint8_t request,
        uint16_t value,
        uint16_t index,
        unsigned char* buff,
        uint16_t length,
        uint32_t timeout_ms);
    static sptr from_usb_device_handle(const usb_device_handle::sptr& handle);

private:
    int _fd;
    uint16_t _vid;
    uint16_t _pid;
    std::string _usbfs_path;
    bool _firmware_loaded;
    mutable std::mutex _mutex;
    std::set<int> _claimed_interfaces;
};

usb_control::sptr make_android_usbfs_control(
    const android_usbfs_device_handle::sptr& handle, int interface);
usb_zero_copy::sptr make_android_usbfs_zero_copy(android_usbfs_device_handle::sptr handle,
    int recv_interface,
    unsigned char recv_endpoint,
    int send_interface,
    unsigned char send_endpoint,
    const device_addr_t& hints);

}} // namespace uhd::transport
