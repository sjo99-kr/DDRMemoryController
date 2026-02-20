# DDR4 Memory BFM

This directory contains a cycle-accurate DDR4 Bus Functional Model (BFM)
used to verify and validate the custom multi-channel DDR4 memory controller.

The BFM models channel, rank, and bank-level DRAM behavior with
command-level timing enforcement and burst-level data transfers.

---

## 📐 System Overview
<img width="640" height="320" alt="image" src="https://github.com/user-attachments/assets/3e0d4e60-2e5d-4277-a01c-cb80f499e431" />




The Memory Controller connects to multiple DDR4 channels.
Each channel contains multiple independent ranks, and each rank
is composed of multiple bank-level FSMs.

Hierarchical structure:

MemoryBFM  
 └── MemoryChannel (per channel)  
      └── MemoryRank (per rank)  
           └── MemoryBankFSM (per bank)  

---

## 🧱 Module Structure

### 1️⃣ MemoryBFM
Top-level structural wrapper.

- Instantiates multiple memory channels
- No timing logic
- Pure structural composition

---

### 2️⃣ MemoryChannel
Channel-level DDR4 model.

Responsibilities:
- Broadcast CA/ADDR signals to all ranks
- Generate channel-level DQS during read bursts
- Perform rank-level DQ arbitration
- Model tri-state bidirectional DQ bus behavior

---

### 3️⃣ MemoryRank
Rank-level behavioral model.

Responsibilities:
- Decode BG/BK fields
- Select target bank
- Aggregate per-bank read/write activity
- Expose rank-level DQ valid signals

Assumption:
At most one bank drives DQ at a time.

---

### 4️⃣ MemoryBankFSM
Bank-level DDR4 behavioral FSM.

Responsibilities:
- Decode ACT / READ / WRITE / PRE / REF commands
- Enforce timing constraints
- Model row state transitions
- Generate burst-level data (clk2x domain)
- Support auto-precharge behavior

State transitions:

rowClosed → Activate → rowOpened  
rowOpened → Read / Write / Precharge  
Read/Write → (AutoPrecharge) → Precharge  

---

## ⏱ Timing Constraints Implemented

| Parameter | Modeled |
|-----------|---------|
| tRCD | ✅ |
| tCL  | ✅ |
| tCWL | ✅ |
| tRP  | ✅ |
| tRFC | ✅ |

Timing is enforced via a dedicated `DRAMTimingCounter`
separated from FSM logic.

---

## 🔄 Multi-Clock Behavior

- `clk` : Command decoding and state transitions
- `clk2x` : Burst-level data transfer

This allows realistic DDR burst modeling.

---

## 🔌 Bidirectional Bus Modeling

DQ/DQS are modeled as:

Tri-state bidirectional buses.

- Read → Rank drives DQ
- Write → Controller drives DQ
- Idle → High-Z state (or Verilator-safe modeling)

---

## 🧪 Verification Notes

- Integrated with custom DDR4 Memory Controller RTL
- Multi-channel simulation validated
- Lint-clean under Verilator
- Tested with Vivado simulator

---

## 🚀 Future Extensions

- tCCD / tRRD / tFAW modeling
- Full row-address-aware memory array
- Assertion-based conflict checking
- Coverage instrumentation

---

This BFM is intended for architectural validation,
memory controller verification, and multi-channel DDR experimentation.
