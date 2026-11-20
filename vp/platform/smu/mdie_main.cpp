// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Tenstorrent USA, Inc.
// ===========================================================================
// vp/platform/smu/mdie_main.cpp
//
// sc_main for `mdie-vp`: two OCA chiplets (dies) in one SystemC process,
// each a full smu_platform (SMC + SEP + SMU crossbar), with their AXI-over-UCIe
// bridges cross-connected so that die 0's AoU forwards to die 1's AoU and back.
// This replaces each die's remote-die stand-in (smc_platform::aou_peer_ ->
// stub_sysmem) with the other real die.
//
//   mdie-vp <smc-cci-ini> <sep-cci-ini> <sim_time_ms>
//           <die0-smc-elf> <die0-sep-elf> <die1-smc-elf> <die1-sep-elf>
//
// Naming: die 0 keeps the smu-vp names (`dut`, `och_sep_ss1`, `smu_xbar`) so
// existing inis apply unchanged; die 1 is `dut1`, `och_sep_ss2`, `smu_xbar1`.
// Every preset loaded for die 0 is cloned to die 1, then the per-die values
// below are applied.
//
// Global address map: die k's SMC global window is 0x4000_0000 + k * 64 MiB
// (each SMC's own outbound remap windows occupy base + 16..32 MiB).
// The SMU crossbar aperture (smu_xbar*.smc_global_base) is preset here; the
// SMC fabric's own GLOBAL_BASE CSR (0xC001_0000) is programmed by each die's
// firmware at boot, as on silicon. Traffic one die sends to the other die's
// window leaves through its crossbar's ext_out catch-all, crosses the AoU pair,
// and enters the other die through ext_in, which routes it to that die's SMC.
//
// Topology limit: each die has one AoU, so this top connects exactly one pair.
// ===========================================================================

#include "smu_platform.hpp"
#include "reg_logger.h"
#include "reg_param.h"
#include "tlm_quantum_policy.h"

#include <systemc.h>
#include <cci_configuration>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

uint64_t read_elf64_entry(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        std::cerr << "ERROR: cannot open ELF '" << path << "'\n";
        return UINT64_MAX;
    }
    unsigned char ident[16];
    f.read(reinterpret_cast<char*>(ident), 16);
    if (!f || ident[0] != 0x7f || ident[1] != 'E' || ident[2] != 'L' || ident[3] != 'F') {
        std::cerr << "ERROR: '" << path << "' is not an ELF file\n";
        return UINT64_MAX;
    }
    if (ident[4] != 2) {  // ELFCLASS64
        std::cerr << "ERROR: '" << path << "' is not ELF64 (mdie-vp runs RV64 SMC)\n";
        return UINT64_MAX;
    }
    f.seekg(24, std::ios::beg);
    uint64_t entry = 0;
    f.read(reinterpret_cast<char*>(&entry), 8);
    if (!f) {
        std::cerr << "ERROR: short read on ELF header of '" << path << "'\n";
        return UINT64_MAX;
    }
    return entry;
}

void parse_ini(const std::string& path);

void apply_ini_value(const std::string& key, const std::string& section,
                     const std::string& raw)
{
    cci::cci_originator o("smu_vp_cfg");
    auto broker = cci::cci_get_global_broker(o);

    auto trim = [](std::string s) {
        auto a = s.find_first_not_of(" \t");
        auto b = s.find_last_not_of(" \t\r\n");
        if (a == std::string::npos) return std::string{};
        return s.substr(a, b - a + 1);
    };
    std::string keyname = trim(key);
    std::string v = trim(raw);

    cci::cci_value val;
    if (section == "bool") {
        val = cci::cci_value((v == "true" || v == "1"));
    } else if (section == "int") {
        val = cci::cci_value(static_cast<int64_t>(std::strtoll(v.c_str(), nullptr, 0)));
    } else if (section == "uint") {
        val = cci::cci_value(static_cast<uint64_t>(std::strtoull(v.c_str(), nullptr, 0)));
    } else if (section == "double" || section == "float") {
        val = cci::cci_value(std::strtod(v.c_str(), nullptr));
    } else if (section == "string") {
        if (v.size() >= 2 && v.front() == '"' && v.back() == '"') v = v.substr(1, v.size() - 2);
        val = cci::cci_value(v);
    } else {
        return;
    }
    broker.set_preset_cci_value(keyname, val);
}

void parse_ini(const std::string& path)
{
    std::ifstream f(path);
    if (!f) {
        std::cerr << "WARNING: cannot open ini '" << path << "' (skipping)\n";
        return;
    }
    std::string dir;
    {
        auto slash = path.find_last_of('/');
        dir = (slash == std::string::npos) ? "." : path.substr(0, slash);
    }
    std::string line, section;
    while (std::getline(f, line)) {
        auto t0 = line.find_first_not_of(" \t");
        if (t0 == std::string::npos) continue;
        std::string t = line.substr(t0);
        if (t.empty() || t[0] == '#') continue;
        if (t[0] == '@') {
            std::istringstream ss(t.substr(1));
            std::string inc; ss >> inc;
            std::string ip = inc;
            if (!ip.empty() && ip[0] != '/') ip = dir + "/" + ip;
            parse_ini(ip);
            continue;
        }
        if (t.front() == '[') {
            auto e = t.find(']');
            section = (e == std::string::npos) ? t.substr(1) : t.substr(1, e - 1);
            continue;
        }
        auto colon = t.find(':');
        if (colon == std::string::npos) continue;
        apply_ini_value(t.substr(0, colon), section, t.substr(colon + 1));
    }
}

void apply_smc_default_presets(const std::string& top, uint64_t elf_entry)
{
    cci::cci_originator o("smu_vp_cfg");
    auto broker = cci::cci_get_global_broker(o);
    auto set = [&](const std::string& leaf, cci::cci_value v) {
        broker.set_preset_cci_value(top + "." + leaf, std::move(v));
    };

    set("bootrom.size_bytes",        cci::cci_value(uint64_t(0x20000)));
    set("scratchpad_ram.size_bytes", cci::cci_value(uint64_t(0x100000)));
    set("plic.num_sources",          cci::cci_value(336u));
    set("plic.num_contexts",         cci::cci_value(8u));
    set("clint.num_harts",           cci::cci_value(4u));
    set("clint.tick_period_ns",      cci::cci_value(0.0));
    set("reset_unit.num_subsystems", cci::cci_value(32u));

    set("cluster.num_harts",    cci::cci_value(4u));
    set("cluster.reset_pc",     cci::cci_value(elf_entry));
    set("cluster.fast_mem_lo",  cci::cci_value(uint64_t(0x80000000)));
    set("cluster.fast_mem_hi",  cci::cci_value(uint64_t(0x90000000)));
    set("cluster.mmio_lo",      cci::cci_value(uint64_t(0xC000'0000ULL)));
    set("cluster.mmio_hi",      cci::cci_value(uint64_t(0xC100'0000ULL)));
    set("cluster.source_id",    cci::cci_value(uint16_t(0x10)));
}

} // namespace

namespace {

constexpr unsigned NUM_DIES = 2;
constexpr uint64_t SMC_GLOBAL_BASE0  = 0x4000'0000ULL;
constexpr uint64_t SMC_GLOBAL_STRIDE = 0x0400'0000ULL;   // 64 MiB per die: clears the M-mode and Xvisor remap windows at base + 16..32 MiB

struct die_names {
    std::string smc, sep, suffix;
    std::string xbar() const { return "smu_xbar" + suffix; }
};

die_names names_of(unsigned k)
{
    if (k == 0) return {"dut", "och_sep_ss1", ""};
    return {"dut" + std::to_string(k), "och_sep_ss" + std::to_string(k + 1), std::to_string(k)};
}

// Copy every not-yet-consumed preset under `from.` to `to.`, except the keys
// a die must not inherit (`skip` leaves, e.g. its own reset PC or ELF).
void clone_presets(const std::string& from, const std::string& to,
                   const std::vector<std::string>& skip)
{
    cci::cci_originator o("mdie_vp_cfg");
    auto broker = cci::cci_get_global_broker(o);
    std::vector<std::pair<std::string, cci::cci_value>> todo;
    for (const auto& p : broker.get_unconsumed_preset_values()) {
        const std::string& key = p.first;
        if (key.rfind(from + ".", 0) != 0) continue;
        const std::string leaf = key.substr(from.size() + 1);
        bool skipped = false;
        for (const auto& s : skip) skipped |= (leaf == s);
        if (!skipped) todo.emplace_back(to + "." + leaf, p.second);
    }
    for (auto& [k, v] : todo) broker.set_preset_cci_value(k, v);
}

} // namespace

int sc_main(int argc, char** argv)
{
    if (argc != 8) {
        std::cerr << "Usage: " << argv[0]
                  << " <smc-cci-ini> <sep-cci-ini> <sim_time_ms>"
                     " <die0-smc-elf> <die0-sep-elf> <die1-smc-elf> <die1-sep-elf>\n";
        return 1;
    }
    const auto absn = [](const char* p) {
        return std::filesystem::absolute(p).lexically_normal().string();
    };
    const std::string smc_ini = absn(argv[1]);
    const std::string sep_ini = absn(argv[2]);
    const double sim_time_ms  = std::stod(argv[3]);
    std::array<std::string, NUM_DIES> smc_elf{absn(argv[4]), absn(argv[6])};
    std::array<std::string, NUM_DIES> sep_elf{absn(argv[5]), absn(argv[7])};

    {
        std::filesystem::path cfgPath(sep_ini);
        (void) ::setenv("SEP_VP_INI_DIR", cfgPath.parent_path().string().c_str(), 1);
        std::filesystem::current_path(cfgPath.parent_path());
        regmodel::load_config_file(cfgPath.string().c_str());
    }

    cci::cci_originator o("mdie_vp_cfg");
    auto broker = cci::cci_get_global_broker(o);

    std::array<uint64_t, NUM_DIES> entry{};
    for (unsigned k = 0; k < NUM_DIES; ++k) {
        entry[k] = read_elf64_entry(smc_elf[k]);
        if (entry[k] == UINT64_MAX) return 1;
        const die_names n = names_of(k);
        apply_smc_default_presets(n.smc, entry[k]);
        broker.set_preset_cci_value(n.xbar() + ".sep_region_size", cci::cci_value(uint64_t(0x0100'0000ULL)));
        broker.set_preset_cci_value(n.xbar() + ".smc_region_size", cci::cci_value(uint64_t(0x0100'0000ULL)));
        std::cout << "mdie-vp: die" << k << " SMC ELF '" << smc_elf[k] << "' entry=0x"
                  << std::hex << entry[k] << std::dec << "\n";
    }
    parse_ini(smc_ini);   // die-0 keys (dut.*, smu_xbar.*)

    for (unsigned k = 1; k < NUM_DIES; ++k) {
        const die_names n0 = names_of(0), n = names_of(k);
        clone_presets(n0.smc, n.smc, {"cluster.reset_pc"});
        clone_presets(n0.xbar(), n.xbar(), {});
        clone_presets(n0.sep, n.sep, {"targets"});
    }

    for (unsigned k = 0; k < NUM_DIES; ++k) {
        const die_names n = names_of(k);
        const uint64_t smc_global = SMC_GLOBAL_BASE0 + k * SMC_GLOBAL_STRIDE;
        broker.set_preset_cci_value(n.sep + ".smc_global.forward_en", cci::cci_value(true));
        broker.set_preset_cci_value(n.sep + ".sep_global_base", cci::cci_value(uint64_t(0x5000'0000ULL)));
        const cci::cci_value xbar_sep_size = broker.get_preset_cci_value(n.xbar() + ".sep_region_size");
        if (xbar_sep_size.is_uint64() || xbar_sep_size.is_number())
            broker.set_preset_cci_value(n.sep + ".sep_region_size", xbar_sep_size);
        broker.set_preset_cci_value(n.xbar() + ".smc_global_base", cci::cci_value(smc_global));
        broker.set_preset_cci_value(n.sep + ".smc_global_base",
                                    cci::cci_value(smu::smu_platform::SEP_SMC_REGION_BASE));
        broker.set_preset_cci_value(n.sep + ".smc_region_size",
                                    cci::cci_value(smu::smu_platform::SEP_SMC_REGION_SIZE));
        broker.set_preset_cci_value(n.sep + ".targets",
                                    cci::cci_value::from_json("\"" + sep_elf[k] + "\""));
        std::cout << "mdie-vp: die" << k << " = " << n.smc << " + " << n.sep
                  << ", SMC global window 0x" << std::hex << smc_global << std::dec << "\n";
    }

    RegLogger::setGlobalLogFile("och_sep_ss.log");

    {
        auto read_u64 = [&](const std::string& key, uint64_t& out) -> bool {
            const cci::cci_value v = broker.get_preset_cci_value(key);
            if (v.is_uint64()) { out = v.get_uint64(); return true; }
            return false;
        };
        uint64_t qns = simtlm::DEFAULT_GLOBAL_QUANTUM_NS;
        if (!read_u64("global_quantum_ns", qns) &&
            !read_u64("och_sep_ss1.globalQuantumNs", qns) &&
            !read_u64("dut.cluster.quantum_ns", qns)) {
            qns = simtlm::DEFAULT_GLOBAL_QUANTUM_NS;
        }
        simtlm::set_global_quantum_ns(qns);
        std::cout << "mdie-vp: global TLM quantum " << qns << " ns\n";
    }

    std::vector<std::unique_ptr<smu::smu_platform>> die;
    for (unsigned k = 0; k < NUM_DIES; ++k) {
        const die_names n = names_of(k);
        die.emplace_back(std::make_unique<smu::smu_platform>(n.smc.c_str(), n.sep.c_str(), n.suffix));
        if (!die.back()->dut.cluster.load_elf({smc_elf[k]})) {
            std::cerr << "ERROR: die" << k << " cluster.load_elf failed for '" << smc_elf[k] << "'\n";
            return 1;
        }
    }

    // Die-to-die link: each AoU's peer is the other die's AoU instead of the
    // in-platform stand-in. Activation on either side auto-acks the other.
    die[0]->dut.aou_.connect_peer(&die[1]->dut.aou_);
    die[1]->dut.aou_.connect_peer(&die[0]->dut.aou_);
    std::cout << "mdie-vp: die0.aou <-> die1.aou connected\n";

    // MDIE_VP_VERBOSITY=<n> overrides the SystemC report verbosity after
    // elaboration (each platform sets it from its own log_verbosity).
    if (const char* v = std::getenv("MDIE_VP_VERBOSITY"))
        sc_core::sc_report_handler::set_verbosity_level(std::atoi(v));

    std::cout << "mdie-vp: running " << sim_time_ms << " ms of simulation...\n";
    sc_start(sc_time(sim_time_ms, SC_MS));
    std::cout << "mdie-vp: simulation ended at " << sc_time_stamp() << "\n";

    for (unsigned k = 0; k < NUM_DIES; ++k) {
        std::cout << "---- die" << k << " SMC UART0 output ----\n";
        uint8_t ch = 0;
        while (die[k]->dut.uart_[0].dbg_tx_pop(ch)) std::cout.put((char)ch);
        std::cout << "\n---- end die" << k << " SMC UART0 ----\n";
    }
    return 0;
}
