![RTL-LINT](https://github.com/sjo99-kr/DDR_based_Memory_Controller/actions/workflows/ci.yml/badge.svg)
# DDR Multi-Channel Memory Controller (SystemVerilog)
A timing-aware, **multi-channel DDR Memory Controller** designed and verified in SystemVerilog.

This project implements a hierarchical DDR controller architecture with per-channel, per-rank, and per-bank scheduling, along with a custom DDR4 Bus Functional Model (BFM) and a verification infrastructure including driver, monitor, and scoreboard.


**Note**: The conditions of Refresh is needed to be modified, I'm working on this now (2026/06/13)

---


## 🏗 Architecture Overview
<img width="1213" height="672" alt="image" src="https://github.com/user-attachments/assets/f93a4d25-b2bc-4200-8718-d41b12eb4995" />



### Key Architectural Features

- Dual-channel DDR backend
- Per-rank FR-FCFS request scheduler and execution unit
- Bank-level FSM with timing enforcement
- AXI-based frontend interface
- Separate CMD/ADDR and DQ bus arbitration
- Open-page policy with timing-aware command scheduling
- Timing-aware rank and bank tracking

---

## 🧠 Design Philosophy

The controller is structured hierarchically:

- **Frontend**
  - Cache Request Arbiter
  - Cache Response Arbiter
  - Cache Response Scheduler
  - Address Translation Unit
  - MC Request Arbiter

- **Backend**
  - Channel Controller
  - Rank Scheduler
  - Rank Execution Unit
  - Bank FSMs
  - DDR CMD/ADDR Bus Arbiter
  - DDR DQ Bus Arbiter
  - PHY Read/Write mode separation
  - Read/Write Buffer Controller

The design enforces DRAM timing constraints at coommand-level and models channel/rank/bank-level parallelism explicitly.

---

### Verification Overview (UVM-like + RTL + BFM)
<img width="1191" height="667" alt="image" src="https://github.com/user-attachments/assets/9b5a96e1-11db-4499-86ee-abc5416957fb" />

The verification environment follows a UVM-like layered architecture:

- AXI-based Driver (random traffic via LFSR)
- Monitor (AXI + DDR command-level event tracking)
- Scoreboard with timing validation
- Custom DDR4 Bus Functional Model (BFM)

The scoreboard validates:

- Read Request/Response ID & User matching
- Write Request/ACK ID & User matching
- Deadlock detection
- DRAM timing constraint enforcement:
  - tCL
  - tCWL
  - tCCD (bank-group aware)
  - tRCD (per-rank, per-bank tracking)
- Data burst timing validation
- Command-to-data consistency

---  

## 🛠 Tool Flow

This project supports automated linting, synthesis, and simulation.

| Stage       | Tool                    | Description                                   |
|------------|--------------------------|-----------------------------------------------|
| Lint       | Verilator 5.045          | SystemVerilog lint & static analysis          |
| Synthesis  | Yosys 0.62+0             | RTL synthesis (Nangate45 technology mapping)  |
| Simulation | Vivado 2024.2 (XSIM)     | Functional simulation & waveform analysis     |

### Run Examples

```bash
# Lint (RTL & BFM)
./scripts/lint_rtl.sh
./scripts/lint_bfm.sh

# Synthesis
./scripts/syn_nangate45.sh

# Simulation (UVM-like testbench, XSIM)
./scripts/xsim_uvm.sh
```

## 📚 References

1. L. Gopalakrishnan, V. Thyagarajan, P. Kole, and G. R. Gangula,  
   *“Memory Controller with Reconfigurable Hardware,”* 2015.  
   – Architectural inspiration for hierarchical controller design.

2. ananthbhat94,  
   *“DDR4MemoryController”* (GitHub repository).  
   – Reference for DDR interface signal definitions.

3. H. Luo et al.,  
   *“Ramulator 2.0: A Modern, Modular, and Extensible DRAM Simulator,”*  
   IEEE Computer Architecture Letters, 2023.  
   – Reference for cycle-level DDR timing parameters.
   

