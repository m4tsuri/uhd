//
// Copyright 2026 pixdr contributors
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "android_usbfs.hpp"
#include <uhd/exception.hpp>
#include <uhd/transport/android_usb_context.hpp>
#include <uhd/transport/buffer_pool.hpp>
#include <uhd/utils/log.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/format.hpp>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <linux/usbdevice_fs.h>
#include <map>
#include <mutex>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

using namespace uhd;
using namespace uhd::transport;

namespace {
static const size_t DEFAULT_NUM_XFERS = 16;
static const size_t DEFAULT_XFER_SIZE = 32 * 512;

static std::string errno_string(const char* op)
{
    return str(boost::format("%s failed: errno=%d (%s)") % op % errno % std::strerror(errno));
}
} // namespace

android_usbfs_device_handle::android_usbfs_device_handle(
    int fd, uint16_t vid, uint16_t pid, std::string usbfs_path, bool firmware_loaded)
    : _fd(fd)
    , _vid(vid)
    , _pid(pid)
    , _usbfs_path(std::move(usbfs_path))
    , _firmware_loaded(firmware_loaded)
{
}

android_usbfs_device_handle::~android_usbfs_device_handle()
{
    try {
        release_claimed_interfaces();
    } catch (...) {
        // NOP: destructors must not throw.
    }
}

std::string android_usbfs_device_handle::get_serial() const
{
    return "";
}

std::string android_usbfs_device_handle::get_manufacturer() const
{
    return _firmware_loaded ? "Ettus Research LLC" : "Cypress";
}

std::string android_usbfs_device_handle::get_product() const
{
    return _firmware_loaded ? "USRP B200/B210" : "WestBridge";
}

uint16_t android_usbfs_device_handle::get_vendor_id() const
{
    return _vid;
}

uint16_t android_usbfs_device_handle::get_product_id() const
{
    return _pid;
}

bool android_usbfs_device_handle::firmware_loaded()
{
    return _firmware_loaded;
}

int android_usbfs_device_handle::fd() const
{
    return _fd;
}

const std::string& android_usbfs_device_handle::usbfs_path() const
{
    return _usbfs_path;
}

void android_usbfs_device_handle::claim_interface(int interface)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_claimed_interfaces.count(interface)) {
        return;
    }

    unsigned int ifno = static_cast<unsigned int>(interface);
    if (ioctl(_fd, USBDEVFS_CLAIMINTERFACE, &ifno) < 0) {
        if (errno != EBUSY) {
            throw uhd::os_error(errno_string("USBDEVFS_CLAIMINTERFACE"));
        }
        UHD_LOGGER_WARNING("USB") << "Android usbfs interface " << interface
                                  << " already claimed (continuing)";
    }
    _claimed_interfaces.insert(interface);
}

void android_usbfs_device_handle::release_claimed_interfaces()
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (int interface : _claimed_interfaces) {
        unsigned int ifno = static_cast<unsigned int>(interface);
        ioctl(_fd, USBDEVFS_RELEASEINTERFACE, &ifno);
    }
    _claimed_interfaces.clear();
}

void android_usbfs_device_handle::clear_endpoint(unsigned char endpoint)
{
    unsigned int ep = endpoint;
    if (ioctl(_fd, USBDEVFS_CLEAR_HALT, &ep) < 0) {
        UHD_LOGGER_TRACE("USB") << "Android USBDEVFS_CLEAR_HALT ep=0x" << std::hex
                                << int(endpoint) << std::dec << " failed: errno=" << errno;
    }
}

void android_usbfs_device_handle::clear_endpoints(
    unsigned char recv_endpoint, unsigned char send_endpoint)
{
    clear_endpoint(recv_endpoint | 0x80);
    clear_endpoint(send_endpoint | 0x00);
}

void android_usbfs_device_handle::reset_device()
{
    if (ioctl(_fd, USBDEVFS_RESET, 0) < 0) {
        UHD_LOGGER_TRACE("USB") << "Android USBDEVFS_RESET failed: errno=" << errno;
    }
}

int android_usbfs_device_handle::control_transfer(uint8_t request_type,
    uint8_t request,
    uint16_t value,
    uint16_t index,
    unsigned char* buff,
    uint16_t length,
    uint32_t timeout_ms)
{
    usbdevfs_ctrltransfer ctrl;
    std::memset(&ctrl, 0, sizeof(ctrl));
    ctrl.bRequestType = request_type;
    ctrl.bRequest     = request;
    ctrl.wValue       = value;
    ctrl.wIndex       = index;
    ctrl.wLength      = length;
    ctrl.timeout      = timeout_ms;
    ctrl.data         = buff;

    int ret = ioctl(_fd, USBDEVFS_CONTROL, &ctrl);
    if (ret < 0) {
        return -errno;
    }
    return ret;
}

android_usbfs_device_handle::sptr android_usbfs_device_handle::from_usb_device_handle(
    const usb_device_handle::sptr& handle)
{
    return std::dynamic_pointer_cast<android_usbfs_device_handle>(handle);
}

class android_usbfs_control_impl : public usb_control
{
public:
    android_usbfs_control_impl(android_usbfs_device_handle::sptr handle, int interface)
        : _handle(std::move(handle))
    {
        _handle->claim_interface(interface);
    }

    int submit(uint8_t request_type,
        uint8_t request,
        uint16_t value,
        uint16_t index,
        unsigned char* buff,
        uint16_t length,
        uint32_t timeout = 0) override
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _handle->control_transfer(
            request_type, request, value, index, buff, length, timeout);
    }

private:
    android_usbfs_device_handle::sptr _handle;
    std::mutex _mutex;
};

usb_control::sptr uhd::transport::make_android_usbfs_control(
    const android_usbfs_device_handle::sptr& handle, int interface)
{
    return usb_control::sptr(new android_usbfs_control_impl(handle, interface));
}

class android_usbfs_zero_copy_impl;

struct android_usbfs_urb_slot
{
    android_usbfs_urb_slot()
        : urb(static_cast<usbdevfs_urb*>(std::calloc(1, sizeof(usbdevfs_urb))))
    {
        UHD_ASSERT_THROW(urb != nullptr);
    }

    ~android_usbfs_urb_slot()
    {
        std::free(urb);
    }

    usbdevfs_urb* urb                  = nullptr;
    android_usbfs_zero_copy_impl* owner = nullptr;
    bool is_recv                       = false;
    size_t index                       = 0;
    uint64_t sequence                  = 0;
    std::atomic<bool> in_flight{false};
};

class android_usbfs_recv_mb : public managed_recv_buffer
{
public:
    android_usbfs_recv_mb(void* buffer, size_t capacity, android_usbfs_zero_copy_impl* owner, size_t index)
        : _capacity(capacity)
        , _owner(owner)
        , _index(index)
    {
        _buffer = buffer;
        _length = capacity;
    }

    void release() override;

    managed_recv_buffer::sptr make_with_length(size_t length)
    {
        return make<managed_recv_buffer>(this, _buffer, length);
    }

    size_t capacity() const
    {
        return _capacity;
    }

private:
    size_t _capacity;
    android_usbfs_zero_copy_impl* _owner;
    size_t _index;
};

class android_usbfs_send_mb : public managed_send_buffer
{
public:
    android_usbfs_send_mb(void* buffer, size_t capacity, android_usbfs_zero_copy_impl* owner, size_t index)
        : _capacity(capacity)
        , _owner(owner)
        , _index(index)
    {
        _buffer = buffer;
        _length = capacity;
    }

    void release() override;

    managed_send_buffer::sptr make_empty()
    {
        return make<managed_send_buffer>(this, _buffer, _capacity);
    }

private:
    size_t _capacity;
    android_usbfs_zero_copy_impl* _owner;
    size_t _index;
};

class android_usbfs_zero_copy_impl : public usb_zero_copy
{
public:
    android_usbfs_zero_copy_impl(android_usbfs_device_handle::sptr handle,
        int recv_interface,
        unsigned char recv_endpoint,
        int send_interface,
        unsigned char send_endpoint,
        const device_addr_t& hints)
        : _handle(std::move(handle))
        , _recv_endpoint((recv_endpoint & 0x7f) | 0x80)
        , _send_endpoint((send_endpoint & 0x7f) | 0x00)
        , _num_recv_frames(size_t(hints.cast<double>("num_recv_frames", DEFAULT_NUM_XFERS)))
        , _num_send_frames(size_t(hints.cast<double>("num_send_frames", DEFAULT_NUM_XFERS)))
        , _recv_frame_size(size_t(hints.cast<double>("recv_frame_size", DEFAULT_XFER_SIZE)))
        , _send_frame_size(size_t(hints.cast<double>("send_frame_size", DEFAULT_XFER_SIZE)))
        , _recv_pool(buffer_pool::make(_num_recv_frames, _recv_frame_size))
        , _send_pool(buffer_pool::make(_num_send_frames, _send_frame_size))
    {
        _handle->claim_interface(recv_interface);
        _handle->claim_interface(send_interface);
        _handle->clear_endpoints(_recv_endpoint, _send_endpoint);

        _recv_slots.reserve(_num_recv_frames);
        _send_slots.reserve(_num_send_frames);
        for (size_t i = 0; i < _num_recv_frames; i++) {
            _recv_slots.push_back(std::make_shared<android_usbfs_urb_slot>());
            _recv_slots.back()->owner   = this;
            _recv_slots.back()->is_recv = true;
            _recv_slots.back()->index   = i;
            _recv_mbs.push_back(std::make_shared<android_usbfs_recv_mb>(
                _recv_pool->at(i), _recv_frame_size, this, i));
        }
        for (size_t i = 0; i < _num_send_frames; i++) {
            _send_slots.push_back(std::make_shared<android_usbfs_urb_slot>());
            _send_slots.back()->owner   = this;
            _send_slots.back()->is_recv = false;
            _send_slots.back()->index   = i;
            _send_mbs.push_back(std::make_shared<android_usbfs_send_mb>(
                _send_pool->at(i), _send_frame_size, this, i));
            _send_free.push_back(i);
        }

        _reaper_thread = std::thread([this]() { this->reap_loop(); });
        for (size_t i = 0; i < _num_recv_frames; i++) {
            submit_recv(i);
        }
    }

    ~android_usbfs_zero_copy_impl() override
    {
        _stopping.store(true);
        for (const auto& slot : _recv_slots) {
            if (slot->in_flight.load()) {
                ioctl(_handle->fd(), USBDEVFS_DISCARDURB, slot->urb);
            }
        }
        for (const auto& slot : _send_slots) {
            if (slot->in_flight.load()) {
                ioctl(_handle->fd(), USBDEVFS_DISCARDURB, slot->urb);
            }
        }
        if (_reaper_thread.joinable()) {
            _reaper_thread.join();
        }
    }

    managed_recv_buffer::sptr get_recv_buff(double timeout) override
    {
        std::unique_lock<std::mutex> lock(_recv_mutex);
        auto has_next = [this]() {
            return _recv_completed.count(_next_recv_deliver_seq) > 0 || _stopping.load();
        };
        if (!has_next()) {
            if (timeout < 0.0) {
                _recv_cv.wait(lock, has_next);
            } else {
                _recv_cv.wait_for(lock,
                    std::chrono::microseconds(static_cast<int64_t>(timeout * 1e6)),
                    has_next);
            }
        }
        auto it = _recv_completed.find(_next_recv_deliver_seq);
        if (it == _recv_completed.end()) {
            return managed_recv_buffer::sptr();
        }
        const size_t index = it->second;
        _recv_completed.erase(it);
        _next_recv_deliver_seq++;
        const int actual_length = _recv_slots[index]->urb->actual_length;
        return _recv_mbs[index]->make_with_length(size_t(actual_length > 0 ? actual_length : 0));
    }

    managed_send_buffer::sptr get_send_buff(double timeout) override
    {
        std::unique_lock<std::mutex> lock(_send_mutex);
        if (_send_free.empty()) {
            if (timeout < 0.0) {
                _send_cv.wait(lock, [this]() { return !_send_free.empty() || _stopping.load(); });
            } else {
                _send_cv.wait_for(lock,
                    std::chrono::microseconds(static_cast<int64_t>(timeout * 1e6)),
                    [this]() { return !_send_free.empty() || _stopping.load(); });
            }
        }
        if (_send_free.empty()) {
            return managed_send_buffer::sptr();
        }
        const size_t index = _send_free.front();
        _send_free.pop_front();
        return _send_mbs[index]->make_empty();
    }

    void release_recv(size_t index)
    {
        if (!_stopping.load()) {
            submit_recv(index);
        }
    }

    void release_send(size_t index, size_t nbytes)
    {
        if (_stopping.load() || nbytes == 0) {
            mark_send_free(index);
            return;
        }
        submit_send(index, nbytes);
    }

    size_t get_num_recv_frames(void) const override
    {
        return _num_recv_frames;
    }

    size_t get_recv_frame_size(void) const override
    {
        return _recv_frame_size;
    }

    size_t get_num_send_frames(void) const override
    {
        return _num_send_frames;
    }

    size_t get_send_frame_size(void) const override
    {
        return _send_frame_size;
    }

private:
    android_usbfs_device_handle::sptr _handle;
    unsigned char _recv_endpoint;
    unsigned char _send_endpoint;
    size_t _num_recv_frames;
    size_t _num_send_frames;
    size_t _recv_frame_size;
    size_t _send_frame_size;
    buffer_pool::sptr _recv_pool;
    buffer_pool::sptr _send_pool;
    std::vector<std::shared_ptr<android_usbfs_urb_slot>> _recv_slots;
    std::vector<std::shared_ptr<android_usbfs_urb_slot>> _send_slots;
    std::vector<std::shared_ptr<android_usbfs_recv_mb>> _recv_mbs;
    std::vector<std::shared_ptr<android_usbfs_send_mb>> _send_mbs;
    std::map<uint64_t, size_t> _recv_completed;
    std::deque<size_t> _send_free;
    std::mutex _recv_mutex;
    std::mutex _send_mutex;
    std::condition_variable _recv_cv;
    std::condition_variable _send_cv;
    std::thread _reaper_thread;
    std::atomic<bool> _stopping{false};
    uint64_t _next_recv_submit_seq = 0;
    uint64_t _next_recv_deliver_seq = 0;

    void init_urb(android_usbfs_urb_slot& slot, unsigned char endpoint, void* buffer, size_t length)
    {
        std::memset(slot.urb, 0, sizeof(*slot.urb));
        slot.urb->type          = USBDEVFS_URB_TYPE_BULK;
        slot.urb->endpoint      = endpoint;
        slot.urb->buffer        = buffer;
        slot.urb->buffer_length = int(length);
        slot.urb->usercontext   = &slot;
    }

    void submit_recv(size_t index)
    {
        android_usbfs_urb_slot& slot = *_recv_slots[index];
        init_urb(slot, _recv_endpoint, _recv_pool->at(index), _recv_frame_size);
        {
            std::lock_guard<std::mutex> lock(_recv_mutex);
            slot.sequence = _next_recv_submit_seq++;
        }
        if (!submit_slot(slot, "IN")) {
            mark_recv_completed(slot);
        }
    }

    void submit_send(size_t index, size_t nbytes)
    {
        android_usbfs_urb_slot& slot = *_send_slots[index];
        init_urb(slot, _send_endpoint, _send_pool->at(index), nbytes);
        if (!submit_slot(slot, "OUT")) {
            mark_send_free(index);
        }
    }

    bool submit_slot(android_usbfs_urb_slot& slot, const char* direction)
    {
        slot.in_flight.store(true);
        if (ioctl(_handle->fd(), USBDEVFS_SUBMITURB, slot.urb) < 0) {
            slot.in_flight.store(false);
            UHD_LOGGER_WARNING("USB") << "Android USBDEVFS_SUBMITURB " << direction
                                      << " ep=0x" << std::hex << int(slot.urb->endpoint)
                                      << std::dec << " failed: errno=" << errno;
            return false;
        }
        return true;
    }

    void reap_loop()
    {
        while (!_stopping.load()) {
            usbdevfs_urb* completed = nullptr;
            int ret = ioctl(_handle->fd(), USBDEVFS_REAPURBNDELAY, &completed);
            if (ret < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    std::this_thread::sleep_for(std::chrono::microseconds(250));
                    continue;
                }
                UHD_LOGGER_WARNING("USB") << "Android USBDEVFS_REAPURBNDELAY failed: errno="
                                          << errno;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            if (!completed || !completed->usercontext) {
                continue;
            }
            auto* slot = static_cast<android_usbfs_urb_slot*>(completed->usercontext);
            if (slot && slot->owner) {
                slot->owner->complete_slot(*slot);
            }
        }
    }

    void complete_slot(android_usbfs_urb_slot& slot)
    {
        slot.in_flight.store(false);
        if (_stopping.load()) {
            return;
        }

        if (slot.is_recv) {
            if (slot.urb->status == 0 && slot.urb->actual_length > 0) {
                mark_recv_completed(slot);
            } else {
                if (slot.urb->status != -ENOENT && slot.urb->status != -ECONNRESET) {
                    UHD_LOGGER_TRACE("USB") << "Android usbfs IN URB status=" << slot.urb->status;
                }
                slot.urb->actual_length = 0;
                mark_recv_completed(slot);
            }
        } else {
            if (slot.urb->status != 0 && slot.urb->status != -ENOENT && slot.urb->status != -ECONNRESET) {
                UHD_LOGGER_WARNING("USB") << "Android usbfs OUT URB status=" << slot.urb->status;
            }
            mark_send_free(slot.index);
        }
    }

    void mark_recv_completed(android_usbfs_urb_slot& slot)
    {
        {
            std::lock_guard<std::mutex> lock(_recv_mutex);
            _recv_completed[slot.sequence] = slot.index;
        }
        _recv_cv.notify_one();
    }

    void mark_send_free(size_t index)
    {
        {
            std::lock_guard<std::mutex> lock(_send_mutex);
            _send_free.push_back(index);
        }
        _send_cv.notify_one();
    }
};

void android_usbfs_recv_mb::release()
{
    _owner->release_recv(_index);
}

void android_usbfs_send_mb::release()
{
    const size_t nbytes = size();
    _owner->release_send(_index, nbytes);
    _length = _capacity;
}

usb_zero_copy::sptr uhd::transport::make_android_usbfs_zero_copy(
    android_usbfs_device_handle::sptr handle,
    int recv_interface,
    unsigned char recv_endpoint,
    int send_interface,
    unsigned char send_endpoint,
    const device_addr_t& hints)
{
    return usb_zero_copy::sptr(new android_usbfs_zero_copy_impl(std::move(handle),
        recv_interface,
        recv_endpoint,
        send_interface,
        send_endpoint,
        hints));
}

std::vector<usb_device_handle::sptr> usb_device_handle::get_device_list(
    uint16_t vid, uint16_t pid, int fd, const std::string& usbfs_path)
{
    std::vector<usb_device_handle::sptr> handles;
    bool firmware_loaded = false;
    if (const android_usb_context* ctx = get_android_usb_context()) {
        firmware_loaded = ctx->firmware_loaded();
    }
    handles.push_back(std::make_shared<android_usbfs_device_handle>(
        fd, vid, pid, usbfs_path, firmware_loaded));
    return handles;
}
