# ===============================
# Packages / Interfaces
# ===============================
../rtl/common/MemoryController_Definitions.sv
../rtl/common/DDR4Interface.sv
../tb/uvm/svut_if.sv

# ===============================
# Common Modules
# ===============================
../rtl/common/DRAMTimingCounter.sv
../rtl/common/DualPortBuffer.sv
../rtl/common/priority_ptr_WriteBuf.sv
../rtl/common/PriorityEncoder_LSB.sv

# ===============================
# Backend
# ===============================
../rtl/backend/APTimingCounter.sv
../rtl/backend/CMDGrantScheduler.sv
../rtl/backend/CMDTurnaroundGrant.sv
../rtl/backend/DQRdWrCCDGrant.sv
../rtl/backend/DQTurnaroundGrant.sv
../rtl/backend/PHYController.sv
../rtl/backend/PHYReadMode.sv
../rtl/backend/PHYWriteMode.sv
../rtl/backend/RankExecutionUnit.sv
../rtl/backend/RankSched.sv
../rtl/backend/RankController.sv
../rtl/backend/ReadBufferController.sv
../rtl/backend/WriteBufferController.sv
../rtl/backend/ChannelController.sv
../rtl/backend/MemoryControllerBackend.sv

# ===============================
# Frontend
# ===============================
../rtl/frontend/AddressTranslationUnit.sv
../rtl/frontend/MemoryControllerFrontend.sv

# ===============================
# Top RTL
# ===============================
../rtl/MemoryController.sv

# ===============================
# BFM
# ===============================
../bfm/MemoryBankFSM.sv
../bfm/MemoryRank.sv
../bfm/MemoryChannel.sv
../bfm/MemoryBFM.sv

# ===============================
# Testbench
# ===============================
../tb/uvm/lfsr_driver.sv
../tb/uvm/driver.sv
../tb/uvm/monitor.sv
../tb/uvm/scoreboard.sv
../tb/uvm/Top_xsim.sv
