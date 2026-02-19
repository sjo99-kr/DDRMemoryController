# ===============================
# Common (packages / interfaces)
# ===============================
common/MemoryController_Definitions.sv
common/DDR4Interface.sv
common/DRAMTimingCounter.sv
common/DualPortBuffer.sv
common/priority_ptr_WriteBuf.sv
common/PriorityEncoder_LSB.sv

# ===============================
# Backend
# ===============================
backend/APTimingCounter.sv
backend/CMDGrantScheduler.sv
backend/CMDTurnaroundGrant.sv
backend/DQRdWrCCDGrant.sv
backend/DQTurnaroundGrant.sv
backend/PHYController.sv
backend/PHYReadMode.sv
backend/PHYWriteMode.sv
backend/RankFSM.sv
backend/RankSched.sv
backend/RankController.sv
backend/ReadBufferController.sv
backend/WriteBufferController.sv
backend/ChannelController.sv
backend/MemoryControllerBackend.sv

# ===============================
# Frontend
# ===============================
frontend/AddressTranslationUnit.sv
frontend/MemoryControllerFrontend.sv

# ===============================
# Top
# ===============================
MemoryController.sv
