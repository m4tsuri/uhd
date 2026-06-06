//
// Copyright 2026 pixdr contributors
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/transport/android_usb_context.hpp>
#include <dlfcn.h>

using uhd::transport::android_usb_context;

extern "C" __attribute__((weak)) const android_usb_context* pixdr_uhd_android_usb_context();

namespace uhd { namespace transport {

const android_usb_context* get_android_usb_context()
{
    using provider_t = const android_usb_context* (*)();

    provider_t provider = nullptr;
    if (pixdr_uhd_android_usb_context) {
        provider = pixdr_uhd_android_usb_context;
    } else {
        provider = reinterpret_cast<provider_t>(
            dlsym(RTLD_DEFAULT, "pixdr_uhd_android_usb_context"));
    }

    if (!provider) {
        return nullptr;
    }

    const android_usb_context* ctx = provider();
    if (!ctx || ctx->fd() < 0) {
        return nullptr;
    }
    return ctx;
}

}} // namespace uhd::transport
