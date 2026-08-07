# Copyright (c) 2021, Xilinx
# Copyright (c) 2022, Advanced Micro Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of FINN nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import pkg_resources as pk

from pyverilator import PyVerilator

axilite_expected_signals = [
    "AWVALID",
    "AWREADY",
    "AWADDR",
    "WVALID",
    "WREADY",
    "WDATA",
    "WSTRB",
    "ARVALID",
    "ARREADY",
    "ARADDR",
    "RVALID",
    "RREADY",
    "RDATA",
    "RRESP",
    "BVALID",
    "BREADY",
    "BRESP",
]

aximm_expected_signals = [
    "AWVALID",
    "AWREADY",
    "AWADDR",
    "AWID",
    "AWLEN",
    "AWSIZE",
    "AWBURST",
    "AWLOCK",
    "AWCACHE",
    "AWPROT",
    "AWQOS",
    "AWREGION",
    "AWUSER",
    "WVALID",
    "WREADY",
    "WDATA",
    "WSTRB",
    "WLAST",
    "WID",
    "WUSER",
    "ARVALID",
    "ARREADY",
    "ARADDR",
    "ARID",
    "ARLEN",
    "ARSIZE",
    "ARBURST",
    "ARLOCK",
    "ARCACHE",
    "ARPROT",
    "ARQOS",
    "ARREGION",
    "ARUSER",
    "RVALID",
    "RREADY",
    "RDATA",
    "RLAST",
    "RID",
    "RUSER",
    "RRESP",
    "BVALID",
    "BREADY",
    "BRESP",
    "BID",
    "BUSER",
]


def rtlsim_multi_io(
    sim,
    io_dict,
    num_out_values,
    trace_file="",
    sname="_V_V_",
    liveness_threshold=10000,
    hook_preclk=None,
    hook_postclk=None,
):
    """Runs the pyverilator simulation by passing the input values to the simulation,
    toggle the clock and observing the execution time. Function contains also an
    observation loop that can abort the simulation if no output value is produced
    after a set number of cycles. Can handle multiple i/o streams. See function
    implementation for details on how the top-level signals should be named.

    Arguments:

    * sim: the PyVerilator object for simulation
    * io_dict: a dict of dicts in the following format:
      {"inputs" : {"in0" : <input_data>, "in1" : <input_data>},
      "outputs" : {"out0" : [], "out1" : []} }
      <input_data> is a list of Python arbitrary-precision ints indicating
      what data to push into the simulation, and the output lists are
      similarly filled when the simulation is complete
    * num_out_values: number of total values to be read from the simulation to
      finish the simulation and return.
    * trace_file: vcd dump filename, empty string (no vcd dump) by default
    * sname: signal naming for streams, "_V_V_" by default, vitis_hls uses "_V_"
    * liveness_threshold: if no new output is detected after this many cycles,
      terminate simulation
    * hook_preclk: hook function to call prior to clock tick
    * hook_postclk: hook function to call after clock tick

    Returns: number of clock cycles elapsed for completion

    """

    if trace_file != "":
        sim.start_vcd_trace(trace_file)

    for outp in io_dict["outputs"]:
        _write_signal(sim, outp + sname + "TREADY", 1)

    # observe if output is completely calculated
    # total_cycle_count will contain the number of cycles the calculation ran
    output_done = False
    total_cycle_count = 0
    output_count = 0
    old_output_count = 0

    # avoid infinite looping of simulation by aborting when there is no change in
    # output values after 100 cycles
    no_change_count = 0

    inp_idx_dict = {}
    for inp in io_dict["inputs"]:
        inp_idx_dict[inp] = 0

    while not (output_done):
        for inp in io_dict["inputs"]:
            inputs = io_dict["inputs"][inp]
            pending = inp_idx_dict[inp] < len(inputs)
            _write_signal(sim, inp + sname + "TVALID", 1 if pending else 0)
            _write_signal(sim, inp + sname + "TDATA", inputs[inp_idx_dict[inp]] if pending else 0)
            if _read_signal(sim, inp + sname + "TREADY") == 1 and _read_signal(sim, inp + sname + "TVALID") == 1:
                inp_idx_dict[inp] += 1

        for outp in io_dict["outputs"]:
            if _read_signal(sim, outp + sname + "TREADY") == 1 and _read_signal(sim, outp + sname + "TVALID") == 1:
                io_dict["outputs"][outp].append(_read_signal(sim, outp + sname + "TDATA"))
                output_count += 1

        if hook_preclk:
            hook_preclk(sim)
        toggle_clk(sim)
        if hook_postclk:
            hook_postclk(sim)

        total_cycle_count = total_cycle_count + 1

        if output_count == old_output_count:
            no_change_count = no_change_count + 1
        else:
            no_change_count = 0
            old_output_count = output_count

        # check if all expected output words received
        if output_count == num_out_values:
            output_done = True

        # end sim on timeout
        if no_change_count == liveness_threshold:
            if trace_file != "":
                sim.flush_vcd_trace()
                sim.stop_vcd_trace()
            raise Exception(
                "Error in simulation! Takes too long to produce output. "
                "Consider setting the LIVENESS_THRESHOLD env.var. to a "
                "larger value."
            )

    if trace_file != "":
        sim.flush_vcd_trace()
        sim.stop_vcd_trace()

    return total_cycle_count


def _find_signal(sim, signal_name):
    # handle both mixed caps and lowercase signal names
    if signal_name in sim.io:
        return signal_name
    elif signal_name.lower() in sim.io:
        return signal_name.lower()
    else:
        raise Exception("Signal not found: " + signal_name)


def _read_signal(sim, signal_name):
    signal_name = _find_signal(sim, signal_name)
    return sim.io[signal_name]


def _write_signal(sim, signal_name, signal_value):
    signal_name = _find_signal(sim, signal_name)
    sim.io[signal_name] = signal_value


def reset_rtlsim(sim, rst_name="ap_rst_n", active_low=True, clk_name="ap_clk"):
    """Sets reset input in pyverilator to zero, toggles the clock and set it
    back to one"""
    _write_signal(sim, rst_name, 0 if active_low else 1)
    toggle_clk(sim, clk_name=clk_name)
    toggle_clk(sim, clk_name=clk_name)
    _write_signal(sim, rst_name, 1 if active_low else 0)
    toggle_clk(sim, clk_name=clk_name)
    toggle_clk(sim, clk_name=clk_name)


def toggle_clk(sim, clk_name="ap_clk"):
    """Toggles the clock input in pyverilator once."""
    _write_signal(sim, clk_name, 0)
    sim.eval()
    _write_signal(sim, clk_name, 1)
    sim.eval()


def wait_for_handshake(sim, ifname, basename="s_axi_control_", dataname="DATA"):
    """Wait for handshake (READY and VALID high at the same time) on given
    interface on PyVerilator sim object.

    Arguments:
    - sim : PyVerilator sim object
    - ifname : name for decoupled interface to wait for handshake on
    - basename : prefix for decoupled interface name
    - dataname : interface data sig name, will be return value if it exists

    Returns: value of interface data signal during handshake (if given by dataname),
    None otherwise (e.g. if there is no data signal associated with interface)
    """
    ret = None
    while 1:
        hs = _read_signal(sim, basename + ifname + "READY") == 1 and _read_signal(sim, basename + ifname + "VALID") == 1
        try:
            ret = _read_signal(sim, basename + ifname + dataname)
        except Exception:
            ret = None
        toggle_clk(sim)
        if hs:
            break
    return ret


def multi_handshake(sim, ifnames, basename="s_axi_control_"):
    """Perform a handshake on list of interfaces given by ifnames. Will assert
    VALID and de-assert after READY observed, in as few cycles as possible."""

    done = []
    for ifname in ifnames:
        _write_signal(sim, basename + ifname + "VALID", 1)
    while len(ifnames) > 0:
        for ifname in ifnames:
            if _read_signal(sim, basename + ifname + "READY") == 1 and _read_signal(sim, basename + ifname + "VALID") == 1:
                done.append(ifname)
        toggle_clk(sim)
        for ifname in done:
            if ifname in ifnames:
                ifnames.remove(ifname)
            _write_signal(sim, basename + ifname + "VALID", 0)


def axilite_write(sim, addr, val, basename="s_axi_control_", wstrb=0xF, sim_addr_and_data=True):
    """Write val to addr on AXI lite interface given by basename.

    Arguments:
    - sim : PyVerilator sim object
    - addr : address for write
    - val : value to be written at addr
    - basename : prefix for AXI lite interface name
    - wstrb : write strobe value to do partial writes, see AXI protocol reference
    - sim_addr_and_data : handshake AW and W channels simultaneously
    """
    _write_signal(sim, basename + "WSTRB", wstrb)
    _write_signal(sim, basename + "WDATA", val)
    _write_signal(sim, basename + "AWADDR", addr)
    if sim_addr_and_data:
        multi_handshake(sim, ["AW", "W"], basename=basename)
    else:
        _write_signal(sim, basename + "AWVALID", 1)
        wait_for_handshake(sim, "AW", basename=basename)
        # write request done
        _write_signal(sim, basename + "AWVALID", 0)
        # write data
        _write_signal(sim, basename + "WVALID", 1)
        wait_for_handshake(sim, "W", basename=basename)
        # write data OK
        _write_signal(sim, basename + "WVALID", 0)
    # wait for write response
    _write_signal(sim, basename + "BREADY", 1)
    wait_for_handshake(sim, "B", basename=basename)
    # write response OK
    _write_signal(sim, basename + "BREADY", 0)


def axilite_read(sim, addr, basename="s_axi_control_"):
    """Read val from addr on AXI lite interface given by basename.

    Arguments:
    - sim : PyVerilator sim object
    - addr : address for read
    - basename : prefix for AXI lite interface name

    Returns: read value from AXI lite interface at given addr
    """
    _write_signal(sim, basename + "ARADDR", addr)
    _write_signal(sim, basename + "ARVALID", 1)
    wait_for_handshake(sim, "AR", basename=basename)
    # read request OK
    _write_signal(sim, basename + "ARVALID", 0)
    # wait for read response
    _write_signal(sim, basename + "RREADY", 1)
    ret_data = wait_for_handshake(sim, "R", basename=basename)
    _write_signal(sim, basename + "RREADY", 0)
    return ret_data


def create_axi_mem_hook(ref_sim, aximm_ifname, mem_depth, mem_init_file="", trace_file=""):
    """Create and return a pair of (pre_hook, post_hook) functions to serve
    as an AXI slave memory on the AXI MM master interface with given name."""
    # find the AXI-MM master interface with given name and extract interface widths
    data_width = ref_sim.io[aximm_ifname + "RDATA"].signal.width
    id_width = ref_sim.io[aximm_ifname + "RID"].signal.width
    addr_width = ref_sim.io[aximm_ifname + "ARADDR"].signal.width
    strb_width = int(data_width / 8)
    # create pyverilator sim object for AXI memory
    example_root = pk.resource_filename("pyverilator.data", "verilog/verilog-axi")
    aximem_sim = PyVerilator.build(
        "axi_ram.v",
        verilog_path=[example_root],
        top_module_name="axi_ram",
        extra_args=[
            "-GDATA_WIDTH=%d" % data_width,
            "-GADDR_WIDTH=%d" % addr_width,
            "-GID_WIDTH=%d" % id_width,
            "-GSTRB_WIDTH=%d" % strb_width,
            "-GMEM_DEPTH=%d" % mem_depth,
            '-GMEM_INIT_FILE="%s"' % mem_init_file,
            "-GDO_MEM_INIT=%d" % (1 if mem_init_file != "" else 0),
        ],
    )
    master_to_slave = []
    slave_to_master = []
    aximem_ifname = "s_axi_"
    # build signal lists that will be passed back and forth between the two sims
    for signal_name in aximm_expected_signals:
        if ref_sim.io[aximm_ifname + signal_name].signal.__class__.__name__ == "Output":
            if (aximem_ifname + signal_name).lower() in aximem_sim.io:
                master_to_slave.append(signal_name)
        elif ref_sim.io[aximm_ifname + signal_name].signal.__class__.__name__ == "Input":
            if (aximem_ifname + signal_name).lower() in aximem_sim.io:
                slave_to_master.append(signal_name)
        else:
            raise Exception("Don't know how to handle AXI MM signal " + signal_name)
    # apply reset
    reset_rtlsim(aximem_sim, "rst", False, clk_name="clk")
    if trace_file != "":
        aximem_sim.start_vcd_trace(trace_file)
    # define hook functions

    def sim_hook_axi_mem_preclk(sim):
        # copy AXI master outputs to AXI slave inputs
        for signal_name in master_to_slave:
            _write_signal(
                aximem_sim,
                aximem_ifname + signal_name,
                sim.io[aximm_ifname + signal_name],
            )
        # sync clock level for aximem_sim
        _write_signal(aximem_sim, "clk", _read_signal(sim, "ap_clk"))
        aximem_sim.eval()

    def sim_hook_axi_mem_postclk(sim):
        toggle_clk(aximem_sim, clk_name="clk")
        # copy all AXI slave outputs to AXI master inputs
        for signal_name in slave_to_master:
            sim.io[aximm_ifname + signal_name] = _read_signal(aximem_sim, aximem_ifname + signal_name)
        if trace_file != "":
            aximem_sim.flush_vcd_trace()

    return (sim_hook_axi_mem_preclk, sim_hook_axi_mem_postclk)
