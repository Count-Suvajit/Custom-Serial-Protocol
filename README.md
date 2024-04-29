RTL detects a packet and performs LED on/off based on command bytes in packet.
It has a serial TX/RX bus to communicate. It drives RX with TX bytes after link_stable is achieved(Align Markers detection).
Files uses Synopsys VCS to compile and simulate.

  Byte Encodings: -
  //AM = 0x55
  //Send 5 successive AMs to assert link_stable
  //SOP = 0xFB
  //EOP = 0xFD
  //IDLE = 0x07
  
  //Packet Format: -
  //<IDLE>
  //<SOP>
  //<ID>
  //<LENGTH>
  //<COMMAND>
  //<EOP>
 
  //COMMAND Ecoding: -
  //LED0 ON = <00>
  //LED0 OFF = <01>
  //LED1 ON = <10>
  //LED1 OFF = <11>

  //<ID> is a configurable parameter in RTL.
  //RTL will not send anything on tx_serial till link_stable
  //2 RTLs are connected in daisy chain.
  //Minimum length of packet should be 1 otherwise it will lead to error.

COMPILE:
vcs -lca -timescale=1ns/100ps -sverilog +verilog2001ext+.v -ntb_opts uvm-1.2 +define+WAVES_FSDB -kdb -debug_access+all -l custom_serial_protocol_compile.log -f custom_serial_protocol_tb_files.f -top custom_serial_protocol_top

SIMULATE:
./simv +vcs+lic+wait -l custom_serial_protocol_sim.log +UVM_MAX_QUIT_COUNT=10000 +UVM_LOG_RECORD +UVM_TR_RECORD +ntb_random_seed=6546 +UVM_VERBOSITY=UVM_MEDIUM
