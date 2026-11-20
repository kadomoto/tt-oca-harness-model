// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Tenstorrent USA, Inc.
// ===========================================================================
// vp/platform/smu/smu_platform.hpp
//
// SMU on-die composition of the SMC and SEP virtual platforms plus the
// SMU AXI interconnect (RTL: hw/smu/rtl/smu.sv). Implementation lives in
// src/smu_platform.cpp — same split as smc_platform / sep_platform.
//
// This type is not an sc_module. The two subsystem platforms stay
// top-level (`dut`, `och_sep_ss1`) so standalone smc-vp / sep-vp CCI
// inis apply unchanged.
// ===========================================================================

#pragma once

#include "smc_platform.hpp"
#include "sep_platform.hpp"

#include "inc/smu_axi_xbar.h"
#include "inc/axi_window_remap.h"
#include "inc/user_port.h"

#include <systemc>
#include <memory>
#include <string>
#include <tlm_utils/simple_initiator_socket.h>

namespace smu {

class idle_initiator : public sc_core::sc_module
{
public:
    tlm_utils::simple_initiator_socket<idle_initiator, 64> sock{"sock"};
    explicit idle_initiator(sc_core::sc_module_name name)
        : sc_core::sc_module(name) {}
};

class smu_platform
{
public:
    // RTL SEP_SMC_REGION_BASE / _SIZE: the SEP's dedicated view of the SMC.
    static constexpr uint64_t SEP_SMC_REGION_BASE = 0x4000'0000ULL;
    static constexpr uint64_t SEP_SMC_REGION_SIZE = 0x4000'0000ULL;  // 1 GiB

    smc::smc_platform         dut;
    och_sep_ss                sep;
    smu_axi_xbar              xbar;
    axi_window_remap<64, 64>  sep2smc_remap;
    idle_initiator            idle_jtag;

    // `suffix` is appended to the SMU-owned module names (smu_xbar, ...) so
    // that several dies can live in one process (mdie-vp). smu-vp uses "".
    // Optional window for the chiplet's own logic (mdie-vp): with
    // user_size > 0, ext_out traffic in [user_base, +user_size) goes to
    // user_out() instead of the AoU. smu-vp leaves it off.
    std::unique_ptr<user_window_router> user_router;

    explicit smu_platform(const char* smc_name = "dut",
                          const char* sep_name = "och_sep_ss1",
                          const std::string& suffix = "",
                          uint64_t user_base = 0, uint64_t user_size = 0);

    // Bind the chiplet's own logic here (only when user_size > 0).
    tlm_utils::simple_initiator_socket<user_window_router, 64>& user_out() { return user_router->to_user; }
};

}  // namespace smu
