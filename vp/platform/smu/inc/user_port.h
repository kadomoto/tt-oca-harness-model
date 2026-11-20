// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 Tenstorrent USA, Inc.
// ===========================================================================
// vp/platform/smu/inc/user_port.h
//
// A port for the chiplet's own logic (the part of a chiplet that is not the
// Harness), for early exploration in mdie-vp.
//
//   user_window_router  sits between the SMU crossbar's ext_out and the local
//                       AoU: transactions in [base, base + size) go to the
//                       chiplet's own logic, everything else to the AoU as before.
//   py_bridge           a TLM target that forwards each access to a model
//                       process (e.g. a Python register model) over pipes.
//   error_target        answers TLM_ADDRESS_ERROR_RESPONSE; binds an unused port.
//
// py_bridge protocol, one line per access, all numbers in hex except time:
//   -> R <addr> <len> <time_ns>            <- OK <value> <delay_ns> | ERR <reason>
//   -> W <addr> <len> <value> <time_ns>    <- OK 0 <delay_ns>       | ERR <reason>
//   -> Q                                    (at end of simulation; no reply)
// `addr` is the offset inside the window; accesses are 1..8 bytes, little endian.
// The model sees simulation time with every access, so time-dependent behavior
// (a transfer that completes after N ns) is evaluated lazily, without callbacks.
// ===========================================================================

#pragma once

#include <systemc>
#include <tlm.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace smu {

class user_window_router : public sc_core::sc_module
{
public:
    tlm_utils::simple_target_socket<user_window_router, 64>    in{"in"};
    tlm_utils::simple_initiator_socket<user_window_router, 64> to_aou{"to_aou"};
    tlm_utils::simple_initiator_socket<user_window_router, 64> to_user{"to_user"};

    user_window_router(sc_core::sc_module_name name, uint64_t base, uint64_t size)
        : sc_core::sc_module(name), base_(base), size_(size)
    {
        in.register_b_transport(this, &user_window_router::b_transport);
        in.register_transport_dbg(this, &user_window_router::transport_dbg);
    }

    uint64_t base() const { return base_; }
    uint64_t size() const { return size_; }

private:
    bool in_window(uint64_t a) const { return a >= base_ && a - base_ < size_; }

    void b_transport(tlm::tlm_generic_payload& t, sc_core::sc_time& d)
    {
        const uint64_t a = t.get_address();
        if (in_window(a)) {
            t.set_address(a - base_);
            to_user->b_transport(t, d);
            t.set_address(a);
        } else {
            to_aou->b_transport(t, d);
        }
    }

    unsigned transport_dbg(tlm::tlm_generic_payload& t)
    {
        const uint64_t a = t.get_address();
        if (in_window(a)) {
            t.set_address(a - base_);
            const unsigned n = to_user->transport_dbg(t);
            t.set_address(a);
            return n;
        }
        return to_aou->transport_dbg(t);
    }

    uint64_t base_, size_;
};

class error_target : public sc_core::sc_module
{
public:
    tlm_utils::simple_target_socket<error_target, 64> tgt{"tgt"};
    explicit error_target(sc_core::sc_module_name name) : sc_core::sc_module(name)
    {
        tgt.register_b_transport(this, &error_target::b_transport);
    }
private:
    void b_transport(tlm::tlm_generic_payload& t, sc_core::sc_time&)
    {
        t.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    }
};

class py_bridge : public sc_core::sc_module
{
public:
    tlm_utils::simple_target_socket<py_bridge, 64> tgt{"tgt"};

    // `command` is run with /bin/sh -c; the child's stderr is inherited, so
    // the model can log straight into the simulation log.
    py_bridge(sc_core::sc_module_name name, const std::string& command)
        : sc_core::sc_module(name), command_(command)
    {
        tgt.register_b_transport(this, &py_bridge::b_transport);
        start();
    }

    ~py_bridge() override
    {
        if (to_ != nullptr) {
            std::fputs("Q\n", to_);
            std::fflush(to_);
            std::fclose(to_);
        }
        if (from_ != nullptr) std::fclose(from_);
        if (pid_ > 0) {
            int st = 0;
            ::waitpid(pid_, &st, 0);
        }
    }

private:
    void start()
    {
        int down[2], up[2];
        if (::pipe(down) != 0 || ::pipe(up) != 0) {
            SC_REPORT_FATAL(name(), "py_bridge: pipe() failed");
            return;
        }
        pid_ = ::fork();
        if (pid_ == 0) {
            ::dup2(down[0], STDIN_FILENO);
            ::dup2(up[1], STDOUT_FILENO);
            ::close(down[0]); ::close(down[1]); ::close(up[0]); ::close(up[1]);
            ::execl("/bin/sh", "sh", "-c", command_.c_str(), static_cast<char*>(nullptr));
            ::_exit(127);
        }
        ::close(down[0]);
        ::close(up[1]);
        to_ = ::fdopen(down[1], "w");
        from_ = ::fdopen(up[0], "r");
        std::cout << name() << ": model process started: " << command_ << "\n";
    }

    void b_transport(tlm::tlm_generic_payload& t, sc_core::sc_time& d)
    {
        const unsigned len = t.get_data_length();
        if (len == 0 || len > 8 || to_ == nullptr) {
            t.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
            return;
        }
        const double now_ns = (sc_core::sc_time_stamp() + d).to_seconds() * 1e9;
        uint64_t v = 0;
        if (t.is_write()) {
            std::memcpy(&v, t.get_data_ptr(), len);
            std::fprintf(to_, "W %llx %x %llx %.3f\n", static_cast<unsigned long long>(t.get_address()), len,
                         static_cast<unsigned long long>(v), now_ns);
        } else {
            std::fprintf(to_, "R %llx %x %.3f\n", static_cast<unsigned long long>(t.get_address()), len, now_ns);
        }
        std::fflush(to_);

        char line[256] = {0};
        if (std::fgets(line, sizeof line, from_) == nullptr) {
            SC_REPORT_ERROR(name(), "py_bridge: model process closed its output");
            t.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
            return;
        }
        unsigned long long value = 0;
        double delay_ns = 0.0;
        if (std::sscanf(line, "OK %llx %lf", &value, &delay_ns) >= 1) {
            if (t.is_read()) {
                v = value;
                std::memcpy(t.get_data_ptr(), &v, len);
            }
            d += sc_core::sc_time(delay_ns, sc_core::SC_NS);
            t.set_response_status(tlm::TLM_OK_RESPONSE);
        } else {
            t.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
        }
    }

    std::string command_;
    pid_t pid_ = -1;
    FILE* to_ = nullptr;
    FILE* from_ = nullptr;
};

}  // namespace smu
