#!/bin/sh

# cleanup
rm -rf obj_dir
rm -f counter.vcd

# run Verilator to translate Verilog into C++, including C++ Testbench
verilator -Wall --cc --trace counter.sv --exe counter_tb.cpp

# build CPP project via make automatically generated
make -j -C obj_dir/ -f Vcounter.mk Vcounter

# run exe sim
obj_dir/Vcounter