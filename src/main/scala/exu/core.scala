//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISC-V Processor Core
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// BOOM has the following (conceptual) stages:
//   if0 - Instruction Fetch 0 (next-pc select)
//   if1 - Instruction Fetch 1 (I$ access)
//   if2 - Instruction Fetch 2 (instruction return)
//   if3 - Instruction Fetch 3 (enqueue to fetch buffer)
//   if4 - Instruction Fetch 4 (redirect from bpd)
//   dec - Decode
//   ren - Rename1
//   dis - Rename2/Dispatch
//   iss - Issue
//   rrd - Register Read
//   exe - Execute
//   mem - Memory
//   sxt - Sign-extend
//   wb  - Writeback
//   com - Commit

package boom.exu

import java.nio.file.Paths

import chisel3.{printf, _}
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.Instructions._
import freechips.rocketchip.rocket.{Causes, PRV}
import freechips.rocketchip.util.{CoreMonitorBundle, Str, UIntIsOneOf}
import freechips.rocketchip.devices.tilelink.{CLINTConsts, PLICConsts}
import testchipip.ExtendedTracedInstruction
import boom.common._
import boom.ifu.{GlobalHistory, HasBoomFrontendParameters}
import boom.exu.FUConstants._
import boom.util._

/**
 * Top level core object that connects the Frontend to the rest of the pipeline.
 */
class BoomCore(usingTrace: Boolean)(implicit p: Parameters) extends BoomModule
  with HasBoomFrontendParameters // TODO: Don't add this trait
{
  val io = new freechips.rocketchip.tile.CoreBundle
  {
    val hartid = Input(UInt(hartIdLen.W))
    val interrupts = Input(new freechips.rocketchip.tile.CoreInterrupts())
    val ifu = new boom.ifu.BoomFrontendIO
    val ptw = Flipped(new freechips.rocketchip.rocket.DatapathPTWIO())
    val rocc = Flipped(new freechips.rocketchip.tile.RoCCCoreIO())
    val lsu = Flipped(new boom.lsu.LSUCoreIO)
    val ptw_tlb = new freechips.rocketchip.rocket.TLBPTWIO()
    val trace = Output(Vec(coreParams.retireWidth, new ExtendedTracedInstruction))
    val fcsr_rm = UInt(freechips.rocketchip.tile.FPConstants.RM_SZ.W)

  }
  //**********************************
  // construct all of the modules  构造所有模块

  // Only holds integer-registerfile execution units.
  val exe_units = new boom.exu.ExecutionUnits(fpu=false)   //仅保存整数寄存器文件的执行单元,关闭FPU
  val jmp_unit_idx = exe_units.jmp_unit_idx
  val jmp_unit = exe_units(jmp_unit_idx)                   //执行单元中的jmp单元
 

  // Meanwhile, the FP pipeline holds the FP issue window, FP regfile, and FP arithmetic units.
  var fp_pipeline: FpPipeline = null                       //同时，FP管道包含FP issue窗口，FP 寄存器文件和FP算术单元。
  if (usingFPU) fp_pipeline = Module(new FpPipeline)       //SmallBoomConfig默认使用FPU

  // ********************************************************
  // Clear fp_pipeline before use
                                          // 初始化FPU流水线
  if (usingFPU) {
    fp_pipeline.io.ll_wports := DontCare                   // from memory unit
    fp_pipeline.io.wb_valids := DontCare
    fp_pipeline.io.wb_pdsts  := DontCare
  }

  val numIrfWritePorts        = exe_units.numIrfWritePorts + memWidth    //memWidth是内存issue单元的位宽
  val numLlIrfWritePorts      = exe_units.numLlIrfWritePorts             //ll是指什么？
  val numIrfReadPorts         = exe_units.numIrfReadPorts

  val numFastWakeupPorts      = exe_units.count(_.bypassable)            //可以绕过exe单元
  val numAlwaysBypassable     = exe_units.count(_.alwaysBypassable)      //总是绕过exe单元，即只有ALU单元

  val numIntIssueWakeupPorts  = numIrfWritePorts + numFastWakeupPorts - numAlwaysBypassable // + memWidth for ll_wb
  val numIntRenameWakeupPorts = numIntIssueWakeupPorts
  val numFpWakeupPorts        = if (usingFPU) fp_pipeline.io.wakeups.length else 0


  val decode_units     = for (w <- 0 until decodeWidth) yield { val d = Module(new DecodeUnit); d }
  val dec_brmask_logic = Module(new BranchMaskGenerationLogic(coreWidth))
  val rename_stage     = Module(new RenameStage(coreWidth, numIntPhysRegs, numIntRenameWakeupPorts, false))
  val fp_rename_stage  = if (usingFPU) Module(new RenameStage(coreWidth, numFpPhysRegs, numFpWakeupPorts, true)) else null
  val pred_rename_stage = Module(new PredRenameStage(coreWidth, ftqSz, 1))
  val rename_stages    = if (usingFPU) Seq(rename_stage, fp_rename_stage, pred_rename_stage) else Seq(rename_stage, pred_rename_stage)


  val mem_iss_unit     = Module(new IssueUnitCollapsing(memIssueParam, numIntIssueWakeupPorts))       //内存issue单元
  mem_iss_unit.suggestName("mem_issue_unit")
  val int_iss_unit     = Module(new IssueUnitCollapsing(intIssueParam, numIntIssueWakeupPorts))       //整数issue单元
  int_iss_unit.suggestName("int_issue_unit")

  val issue_units      = Seq(mem_iss_unit, int_iss_unit)                                              //内存与整数issue单元
  val dispatcher       = Module(new BasicDispatcher)                                                  //分派单元

  val iregfile         = Module(new RegisterFileSynthesizable(                                        //可综合的整数寄存器文件
                             numIntPhysRegs,
                             numIrfReadPorts,
                             numIrfWritePorts,
                             xLen,
                             Seq.fill(memWidth) {true} ++ exe_units.bypassable_write_port_mask)) // bypassable ll_wb
                 //bypassable_write_port_mask: 内存单元也会在Reg ReaD阶段绕过写操作。注意:这里不包括ll_wport
                 //从func单元的写端口 绕过到 regfile的读端口 列表             //可综合的pred寄存器文件，仅在启用SFB时使用
  val pregfile         = Module(new RegisterFileSynthesizable(
                            ftqSz,
                            exe_units.numIrfReaders,
                            1,
                            1,
                            Seq(true))) // The jmp unit is always bypassable
  pregfile.io := DontCare // Only use the IO if enableSFBOpt

  // wb arbiter for the 0th ll writeback
  // TODO: should this be a multi-arb?
  val ll_wbarb         = Module(new Arbiter(new ExeUnitResp(xLen), 1 +
                                                                   (if (usingFPU) 1 else 0) +
                                                                   (if (usingRoCC) 1 else 0)))
                                                  //寄存器读
                                                  //似乎jmp与pred有关
  val iregister_read   = Module(new RegisterRead(
                           issue_units.map(_.issueWidth).sum,
                           exe_units.withFilter(_.readsIrf).map(_.supportedFuncUnits),
                           numIrfReadPorts,
                           exe_units.withFilter(_.readsIrf).map(x => 2),
                           exe_units.numTotalBypassPorts,
                           jmp_unit.numBypassStages,
                           xLen))
                                                            //ROB
  val rob              = Module(new Rob(
                           numIrfWritePorts + numFpWakeupPorts, // +memWidth for ll writebacks
                           numFpWakeupPorts))
  // Used to wakeup registers in rename and issue. ROB needs to listen to something else.   //用于唤醒重命名和issue的寄存器。 ROB需要收听其他内容。
  val int_iss_wakeups  = Wire(Vec(numIntIssueWakeupPorts, Valid(new ExeUnitResp(xLen))))
  val int_ren_wakeups  = Wire(Vec(numIntRenameWakeupPorts, Valid(new ExeUnitResp(xLen))))
  val pred_wakeup  = Wire(Valid(new ExeUnitResp(1)))

  require (exe_units.length == issue_units.map(_.issueWidth).sum)

  //***********************************
  // Pipeline State Registers and Wires

  // Decode/Rename1 Stage
  val dec_valids = Wire(Vec(coreWidth, Bool()))  // are the decoded instruction valid? It may be held up though. // 解码有效性
  val dec_uops   = Wire(Vec(coreWidth, new MicroOp()))                                                           //解码微操作
  val dec_fire   = Wire(Vec(coreWidth, Bool()))  // can the instruction fire beyond decode?                      //指令可否在解码之外发出？
                                                    // (can still be stopped in ren or dis)
  val dec_ready  = Wire(Bool())                                          //解码准备
  val dec_xcpts  = Wire(Vec(coreWidth, Bool()))                          //解码—异常
  val ren_stalls = Wire(Vec(coreWidth, Bool()))                          //重命名暂停

  // Rename2/Dispatch stage
  val dis_valids = Wire(Vec(coreWidth, Bool()))                          //分派有效性
  val dis_uops   = Wire(Vec(coreWidth, new MicroOp))                     //分派微操作
  val dis_fire   = Wire(Vec(coreWidth, Bool()))                          //指令可否在分派之外解雇？
  val dis_ready  = Wire(Bool())                                          //分派准备

  // Issue Stage/Register Read
  val iss_valids = Wire(Vec(exe_units.numIrfReaders, Bool()))            //issue有效性
  val iss_uops   = Wire(Vec(exe_units.numIrfReaders, new MicroOp()))     //issue微操作
  val bypasses   = Wire(Vec(exe_units.numTotalBypassPorts, Valid(new ExeUnitResp(xLen)))) //可否绕过
  val pred_bypasses = Wire(Vec(jmp_unit.numBypassStages, Valid(new ExeUnitResp(1))))  //jmp可否绕过
  require(jmp_unit.bypassable)

  // --------------------------------------
  // Dealing with branch resolutions                                    //处理分支解析

  // The individual branch resolutions from each ALU
  val brinfos = Reg(Vec(coreWidth, new BrResolutionInfo()))                //来自ALU单元的分支解析信息

  // "Merged" branch update info from all ALUs                             //所有ALU的“合并”分支更新信息
  // brmask contains masks for rapidly clearing mispredicted instructions  //brmask包含用于快速清除错误指令的掩码
  // brindices contains indices to reset pointers for allocated structures //brindices包含用于重置已分配结构的指针的索引
  //           brindices is delayed a cycle                                //brindices延迟了一个周期
  val brupdate  = Wire(new BrUpdateInfo)
  val b1    = Wire(new BrUpdateMasks)                                      // b1:在第一个周期中，我们获得了杀死寄存器的掩码   //解析掩码、误预测掩码
  val b2    = Reg(new BrResolutionInfo)                                    // b2:在第二个周期，我们得到索引以重置指针    //来自分支单元的分支解析信息

  brupdate.b1 := b1
  brupdate.b2 := b2

  for ((b, a) <- brinfos zip exe_units.alu_units) {                        //来自ALU单元的分支解析信息  赋值给brinfos
    b := a.io.brinfo
    b.valid := a.io.brinfo.valid && !rob.io.flush.valid
  }
  b1.resolve_mask := brinfos.map(x => x.valid << x.uop.br_tag).reduce(_|_)            //reduce(_|_)是什么意思？
  b1.mispredict_mask := brinfos.map(x => (x.valid && x.mispredict) << x.uop.br_tag).reduce(_|_)

  // Find the oldest mispredict and use it to update indices             //寻找最老的误预测分支，并使用它去更新索引
  var mispredict_val = false.B
  var oldest_mispredict = brinfos(0)
  for (b <- brinfos) {
    val use_this_mispredict = !mispredict_val ||
    b.valid && b.mispredict && IsOlder(b.uop.rob_idx, oldest_mispredict.uop.rob_idx, rob.io.rob_head_idx)

    mispredict_val = mispredict_val || (b.valid && b.mispredict)
    oldest_mispredict = Mux(use_this_mispredict, b, oldest_mispredict)
  }

  b2.mispredict  := mispredict_val
  b2.cfi_type    := oldest_mispredict.cfi_type                         //cfi是什么意思？
  b2.taken       := oldest_mispredict.taken
  b2.pc_sel      := oldest_mispredict.pc_sel
  b2.uop         := UpdateBrMask(brupdate, oldest_mispredict.uop)
  b2.jalr_target := RegNext(jmp_unit.io.brinfo.jalr_target)
  b2.target_offset := oldest_mispredict.target_offset

  val oldest_mispredict_ftq_idx = oldest_mispredict.uop.ftq_idx        //索引FTQ以找出我们的fetch PC。

   //在回滚期间不准有误预测
  assert (!((brupdate.b1.mispredict_mask =/= 0.U || brupdate.b2.mispredict)
    && rob.io.commit.rollback), "Can't have a mispredict during rollback.")

  io.ifu.brupdate := brupdate                                                 //更新指令fetch单元的分支更新信息

                                                     //更新执行单元的分支更新信息
  for (eu <- exe_units) {
    eu.io.brupdate := brupdate
  }

                                                             //更新FPU流水线的分支更新信息
  if (usingFPU) {
    fp_pipeline.io.brupdate := brupdate
  }

  // Load/Store Unit & ExeUnits                                               //load、store单元
  val mem_units = exe_units.memory_units
  val mem_resps = mem_units.map(_.io.ll_iresp)                                //ll是啥意思？
  for (i <- 0 until memWidth) {
    mem_units(i).io.lsu_io <> io.lsu.exe(i)
  }
                                   //lsu的连接？具体含义

  //-------------------------------------------------------------
  // Uarch Hardware Performance Events (HPEs)                                 //微架构硬件性能事件（HPE）

  val perfEvents = new freechips.rocketchip.rocket.EventSets(Seq(
    new freechips.rocketchip.rocket.EventSet((mask, hits) => (mask & hits).orR, Seq(
      ("exception", () => rob.io.com_xcpt.valid),
      ("nop",       () => false.B),
      ("nop",       () => false.B),
      ("nop",       () => false.B))),

    new freechips.rocketchip.rocket.EventSet((mask, hits) => (mask & hits).orR, Seq(
//      ("I$ blocked",                        () => icache_blocked),
      ("nop",                               () => false.B),
      // ("branch misprediction",              () => br_unit.brinfo.mispredict),
      // ("control-flow target misprediction", () => br_unit.brinfo.mispredict &&
      //                                             br_unit.brinfo.cfi_type === CFI_JALR),
      ("flush",                             () => rob.io.flush.valid)
      //("branch resolved",                   () => br_unit.brinfo.valid)
    )),

    new freechips.rocketchip.rocket.EventSet((mask, hits) => (mask & hits).orR, Seq(
      ("I$ miss",     () => io.ifu.perf.acquire),
      ("D$ miss",     () => io.lsu.perf.acquire),
      ("D$ release",  () => io.lsu.perf.release),
      ("ITLB miss",   () => io.ifu.perf.tlbMiss),
      ("DTLB miss",   () => io.lsu.perf.tlbMiss),
      ("L2 TLB miss", () => io.ptw.perf.l2miss)))))
  val csr = Module(new freechips.rocketchip.rocket.CSRFile(perfEvents, boomParams.customCSRs.decls))    //csr与boomcsr的配置
  csr.io.inst foreach { c => c := DontCare }
  csr.io.rocc_interrupt := io.rocc.interrupt

  val custom_csrs = Wire(new BoomCustomCSRs)                     //初始化custom_csrs
  (custom_csrs.csrs zip csr.io.customCSRs).map { case (lhs, rhs) => lhs := rhs }

  //val icache_blocked = !(io.ifu.fetchpacket.valid || RegNext(io.ifu.fetchpacket.valid))
  val icache_blocked = false.B
  csr.io.counters foreach { c => c.inc := RegNext(perfEvents.evaluate(c.eventSel)) } //csr性能计数器

  //****************************************
  // Time Stamp Counter & Retired Instruction Counter                              //时间戳计数器和retired指令计数器
  // (only used for printf and vcd dumps - the actual counters are in the CSRFile) //仅用于printf和vcd转储-实际计数器在CSRFile中）
  val debug_tsc_reg = RegInit(0.U(xLen.W))
  val debug_irt_reg = RegInit(0.U(xLen.W))
  val debug_brs     = Reg(Vec(4, UInt(xLen.W)))
  val debug_jals    = Reg(Vec(4, UInt(xLen.W)))
  val debug_jalrs   = Reg(Vec(4, UInt(xLen.W)))

                                                         //PopCount的作用？
  for (j <- 0 until 4) {
    debug_brs(j) := debug_brs(j) + PopCount(VecInit((0 until coreWidth) map {i =>
      rob.io.commit.arch_valids(i) &&
      (rob.io.commit.uops(i).debug_fsrc === j.U) &&
      rob.io.commit.uops(i).is_br
    }))
    debug_jals(j) := debug_jals(j) + PopCount(VecInit((0 until coreWidth) map {i =>
      rob.io.commit.arch_valids(i) &&
      (rob.io.commit.uops(i).debug_fsrc === j.U) &&
      rob.io.commit.uops(i).is_jal
    }))
    debug_jalrs(j) := debug_jalrs(j) + PopCount(VecInit((0 until coreWidth) map {i =>
      rob.io.commit.arch_valids(i) &&
      (rob.io.commit.uops(i).debug_fsrc === j.U) &&
      rob.io.commit.uops(i).is_jalr
    }))
  }//debug_fsrc 哪种预测结构可根据此操作提供预测

  dontTouch(debug_brs)    //dontTouch防止被优化
  dontTouch(debug_jals)
  dontTouch(debug_jalrs)

  debug_tsc_reg := debug_tsc_reg + 1.U
  debug_irt_reg := debug_irt_reg + PopCount(rob.io.commit.arch_valids.asUInt)
  dontTouch(debug_tsc_reg)
  dontTouch(debug_irt_reg)

  //****************************************
  // Print-out information about the machine                               //打印机器信息

  val issStr =
    if (enableAgePriorityIssue) " (Age-based Priority)"
    else " (Unordered Priority)"

  // val btbStr =
  //   if (enableBTB) ("" + boomParams.btb.nSets * boomParams.btb.nWays + " entries (" + boomParams.btb.nSets + " x " + boomParams.btb.nWays + " ways)")
  //   else 0
  val btbStr = ""

  val fpPipelineStr =
    if (usingFPU) fp_pipeline.toString
    else ""

  override def toString: String =
    (BoomCoreStringPrefix("====Overall Core Params====") + "\n"
    + exe_units.toString + "\n"
    + fpPipelineStr + "\n"
    + rob.toString + "\n"
    + BoomCoreStringPrefix(
        "===Other Core Params===",
        "Fetch Width           : " + fetchWidth,
        "Decode Width          : " + coreWidth,
        "Issue Width           : " + issueParams.map(_.issueWidth).sum,
        "ROB Size              : " + numRobEntries,
        "Issue Window Size     : " + issueParams.map(_.numEntries) + issStr,
        "Load/Store Unit Size  : " + numLdqEntries + "/" + numStqEntries,
        "Num Int Phys Registers: " + numIntPhysRegs,
        "Num FP  Phys Registers: " + numFpPhysRegs,
        "Max Branch Count      : " + maxBrCount)
    + iregfile.toString + "\n"
    + BoomCoreStringPrefix(
        "Num Slow Wakeup Ports : " + numIrfWritePorts,
        "Num Fast Wakeup Ports : " + exe_units.count(_.bypassable),
        "Num Bypass Ports      : " + exe_units.numTotalBypassPorts) + "\n"
    + BoomCoreStringPrefix(
        "DCache Ways           : " + dcacheParams.nWays,
        "DCache Sets           : " + dcacheParams.nSets,
        "DCache nMSHRs         : " + dcacheParams.nMSHRs,
        "ICache Ways           : " + icacheParams.nWays,
        "ICache Sets           : " + icacheParams.nSets,
        "D-TLB Ways            : " + dcacheParams.nTLBWays,
        "I-TLB Ways            : " + icacheParams.nTLBWays,
        "Paddr Bits            : " + paddrBits,
        "Vaddr Bits            : " + vaddrBits) + "\n"
    + BoomCoreStringPrefix(
        "Using FPU Unit?       : " + usingFPU.toString,
        "Using FDivSqrt?       : " + usingFDivSqrt.toString,
        "Using VM?             : " + usingVM.toString) + "\n")

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Fetch Stage/Frontend ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------
  io.ifu.redirect_val         := false.B         //重定向前端？
  io.ifu.redirect_flush       := false.B         //刷新并挂起前端？

  // Breakpoint info                             //断点信息
  io.ifu.status  := csr.io.status
  io.ifu.bp      := csr.io.bp

  io.ifu.mcontext := csr.io.mcontext
  io.ifu.scontext := csr.io.scontext


  io.ifu.flush_icache := (0 until coreWidth).map { i =>
    (rob.io.commit.arch_valids(i) && rob.io.commit.uops(i).is_fencei) ||
    (RegNext(dec_valids(i) && dec_uops(i).is_jalr && csr.io.status.debug))
  }.reduce(_||_)

  // TODO FIX THIS HACK
  // The below code works because of two quirks with the flush mechanism
  //  1 ) All flush_on_commit instructions are also is_unique,
  //      In the future, this constraint will be relaxed.
  //  2 ) We send out flush signals one cycle after the commit signal. We need to
  //      mux between one/two cycle delay for the following cases:
  //       ERETs are reported to the CSR two cycles before we send the flush
  //       Exceptions are reported to the CSR on the cycle we send the flush
  // This discrepency should be resolved elsewhere.
  //修复此问题
  //以下代码之所以有效，是因为使用了刷新机制有两个怪癖
  // 1）所有flush_on_commit指令也是is_unique，将来，这一限制将得到放松。
  // 2）我们在提交信号之后的一个周期发出冲洗信号。 对于以下情况，我们需要在一个/两个周期延迟之间进行复用：
  //在发送冲洗之前的两个周期，将ERET报告给CSR
  //在发送刷新的周期内，异常会报告给CSR
  //这种差异应在其他地方解决。以下是刷新前端的各种情况。

                                            //1、ROB中的flush信号有效
  when (RegNext(rob.io.flush.valid)) {
    io.ifu.redirect_val   := true.B                  //重定向前端
    io.ifu.redirect_flush := true.B                  //刷新并挂起前端
    val flush_typ = RegNext(rob.io.flush.bits.flush_typ)   //刷新类型，ROB需要告知FTQ是否存在管道刷新（以及类型），因此FTQ可以使用正确的重定向PC驱动前端。
    // Clear the global history when we flush the ROB (exceptions, AMOs, unique instructions, etc.)  //刷新ROB时清除全局历史记录（异常，AMO，唯一指令等）
    val new_ghist = WireInit((0.U).asTypeOf(new GlobalHistory))     //新的全局历史表
    new_ghist.current_saw_branch_not_taken := true.B                //当前未看到分支
    new_ghist.ras_idx := io.ifu.get_pc(0).entry.ras_idx             //ras与ftq的关系?  get_pc:为FunctionalUnit提供端口以获取指令PC,对于JALR，则是下一条指令PC的端口。
    io.ifu.redirect_ghist := new_ghist
                      
    //type === xcpt.U || type === eret.U        //刷新类型是否是 eret 执行环境返回指令
                                                //otherwise：刷新类型是否是 xcpt 执行环境返回指令
    when (FlushTypes.useCsrEvec(flush_typ)) {
      io.ifu.redirect_pc  := Mux(flush_typ === FlushTypes.eret,
                                 RegNext(RegNext(csr.io.evec)),
                                 csr.io.evec)
    } .otherwise {
      val flush_pc = (AlignPCToBoundary(io.ifu.get_pc(0).pc, icBlockBytes)
                      + RegNext(rob.io.flush.bits.pc_lob)
                      - Mux(RegNext(rob.io.flush.bits.edge_inst), 2.U, 0.U))
      val flush_pc_next = flush_pc + Mux(RegNext(rob.io.flush.bits.is_rvc), 2.U, 4.U)
      io.ifu.redirect_pc := Mux(FlushTypes.useSamePC(flush_typ),
                                flush_pc, flush_pc_next)

    }
    io.ifu.redirect_ftq_idx := RegNext(rob.io.flush.bits.ftq_idx)    //设置重定向FTQ
  } .elsewhen (brupdate.b2.mispredict && !RegNext(rob.io.flush.valid)) {           //2、ROB未发出flush信号但是b2误预测信号有效
    val block_pc = AlignPCToBoundary(io.ifu.get_pc(1).pc, icBlockBytes)
    val uop_maybe_pc = block_pc | brupdate.b2.uop.pc_lob
    val npc = uop_maybe_pc + Mux(brupdate.b2.uop.is_rvc || brupdate.b2.uop.edge_inst, 2.U, 4.U)
    val jal_br_target = Wire(UInt(vaddrBitsExtended.W))
    jal_br_target := (uop_maybe_pc.asSInt + brupdate.b2.target_offset +
      (Fill(vaddrBitsExtended-1, brupdate.b2.uop.edge_inst) << 1).asSInt).asUInt
    val bj_addr = Mux(brupdate.b2.cfi_type === CFI_JALR, brupdate.b2.jalr_target, jal_br_target)
    val mispredict_target = Mux(brupdate.b2.pc_sel === PC_PLUS4, npc, bj_addr)    //重新计算此分支的pc,是否为pc+4
    io.ifu.redirect_val     := true.B
    io.ifu.redirect_pc      := mispredict_target              //重定向pc赋值
    io.ifu.redirect_flush   := true.B
    io.ifu.redirect_ftq_idx := brupdate.b2.uop.ftq_idx
    val use_same_ghist = (brupdate.b2.cfi_type === CFI_BR &&
                          !brupdate.b2.taken &&
                          bankAlign(block_pc) === bankAlign(npc))
    val ftq_entry = io.ifu.get_pc(1).entry                    //ftq条目
    val cfi_idx = (brupdate.b2.uop.pc_lob ^
      Mux(ftq_entry.start_bank === 1.U, 1.U << log2Ceil(bankBytes), 0.U))(log2Ceil(fetchWidth), 1)
                   //cfi索引  //pc_lobP：我们自己PC的低位。 与ftq [ftq_idx]结合使用可获得PC。
    val ftq_ghist = io.ifu.get_pc(1).ghist
    val next_ghist = ftq_ghist.update(
      ftq_entry.br_mask.asUInt,
      brupdate.b2.taken,
      brupdate.b2.cfi_type === CFI_BR,
      cfi_idx,
      true.B,
      io.ifu.get_pc(1).pc,
      ftq_entry.cfi_is_call && ftq_entry.cfi_idx.bits === cfi_idx,
      ftq_entry.cfi_is_ret  && ftq_entry.cfi_idx.bits === cfi_idx)

                       //选择重定向全局历史表
    io.ifu.redirect_ghist   := Mux(
      use_same_ghist,
      ftq_ghist,
      next_ghist)
    io.ifu.redirect_ghist.current_saw_branch_not_taken := use_same_ghist
  } .elsewhen (rob.io.flush_frontend || brupdate.b1.mispredict_mask =/= 0.U) {    //3、ROB刷新前端信号或者b1误预测掩码不为0   b1与b2的区别是啥？
    io.ifu.redirect_flush   := true.B                 //刷新并挂起前端
  }

  //commit了提取数据包中的最后一条指令后，将释放FTQ条目并将其返回给分支预测器。
  // Tell the FTQ it can deallocate entries by passing youngest ftq_idx.    //告诉FTQ它可以通过传递最小的ftq_idx来释放条目
  val youngest_com_idx = (coreWidth-1).U - PriorityEncoder(rob.io.commit.valids.reverse)
  io.ifu.commit.valid := rob.io.commit.valids.reduce(_|_) || rob.io.com_xcpt.valid
                         //是否将异常传达给CSRFile
  io.ifu.commit.bits  := Mux(rob.io.com_xcpt.valid,
                             rob.io.com_xcpt.bits.ftq_idx,
                             rob.io.commit.uops(youngest_com_idx).ftq_idx)

  assert(!(rob.io.commit.valids.reduce(_|_) && rob.io.com_xcpt.valid),
    "ROB can't commit and except in same cycle!")
                           //ROB不能在同一个周期内commit与异常

  for (i <- 0 until memWidth) {
    when (RegNext(io.lsu.exe(i).req.bits.sfence.valid)) {                   // sfence:only for mcalc
      io.ifu.sfence := RegNext(io.lsu.exe(i).req.bits.sfence)
    }
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Branch Prediction ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Decode Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // track mask of finished instructions in the bundle
  // use this to mask out insts coming from FetchBuffer that have been finished
  // for example, back pressure may cause us to only issue some instructions from FetchBuffer
  // but on the next cycle, we only want to retry a subset
  //bundle中已完成指令的跟踪掩码
  //使用它来mask来自FetchBuffer的已完成的指令
  //例如，后端压力可能导致我们仅从FetchBuffer发出一些指令，但在下一个周期，我们只想重试一个子集
  val dec_finished_mask = RegInit(0.U(coreWidth.W))

  //-------------------------------------------------------------
  // Pull out instructions and send to the Decoders    //拉取指令并发送到解码器

  io.ifu.fetchpacket.ready := dec_ready                //解码准备
  val dec_fbundle = io.ifu.fetchpacket.bits            //fetchpacket:给后端一个指令包, 由从Fetch Bundle输入到Fetch Buffer的转换的MicroOps组成的Bundle。这将移至“解码”阶段。

  //-------------------------------------------------------------
  // Decoders

  for (w <- 0 until coreWidth) {
    dec_valids(w)                      := io.ifu.fetchpacket.valid && dec_fbundle.uops(w).valid &&
                                          !dec_finished_mask(w)
    decode_units(w).io.enq.uop         := dec_fbundle.uops(w).bits    //解码单元入队
    decode_units(w).io.status          := csr.io.status
    decode_units(w).io.csr_decode      <> csr.io.decode(w)
    decode_units(w).io.interrupt       := csr.io.interrupt
    decode_units(w).io.interrupt_cause := csr.io.interrupt_cause

    dec_uops(w) := decode_units(w).io.deq.uop                         //解码单元出队
  }

  //-------------------------------------------------------------
  // FTQ GetPC Port Arbitration                                       //FTQ GetPC端口仲裁

  val jmp_pc_req  = Wire(Decoupled(UInt(log2Ceil(ftqSz).W)))
  val xcpt_pc_req = Wire(Decoupled(UInt(log2Ceil(ftqSz).W)))
  val flush_pc_req = Wire(Decoupled(UInt(log2Ceil(ftqSz).W)))

  val ftq_arb = Module(new Arbiter(UInt(log2Ceil(ftqSz).W), 3))

  // Order by the oldest. Flushes come from the oldest instructions in pipe
  // Decoding exceptions come from youngest   //按最旧的顺序排列。 flush来自管道中最古老的指令, 解码异常来自最小的
  ftq_arb.io.in(0) <> flush_pc_req
  ftq_arb.io.in(1) <> jmp_pc_req
  ftq_arb.io.in(2) <> xcpt_pc_req

  // Hookup FTQ                               //连接FTQ
  io.ifu.get_pc(0).ftq_idx := ftq_arb.io.out.bits
  ftq_arb.io.out.ready  := true.B

  // Branch Unit Requests (for JALs) (Should delay issue of JALs if this not ready)
  //分支单元请求（针对JAL）（如果尚未准备就绪，则应延迟发布JAL）        //1、jmp_pc_req赋值
  jmp_pc_req.valid := RegNext(iss_valids(jmp_unit_idx) && iss_uops(jmp_unit_idx).fu_code === FU_JMP)
  jmp_pc_req.bits  := RegNext(iss_uops(jmp_unit_idx).ftq_idx)

  jmp_unit.io.get_ftq_pc := DontCare
  jmp_unit.io.get_ftq_pc.pc               := io.ifu.get_pc(0).pc
  jmp_unit.io.get_ftq_pc.entry            := io.ifu.get_pc(0).entry
  jmp_unit.io.get_ftq_pc.next_val         := io.ifu.get_pc(0).next_val
  jmp_unit.io.get_ftq_pc.next_pc          := io.ifu.get_pc(0).next_pc


  // Frontend Exception Requests                              //2、xcpt_pc_req赋值、前端异常请求
  val xcpt_idx = PriorityEncoder(dec_xcpts)                   //异常索引
  xcpt_pc_req.valid    := dec_xcpts.reduce(_||_)
  xcpt_pc_req.bits     := dec_uops(xcpt_idx).ftq_idx
  //rob.io.xcpt_fetch_pc := RegEnable(io.ifu.get_pc.fetch_pc, dis_ready)
  rob.io.xcpt_fetch_pc := io.ifu.get_pc(0).pc

  flush_pc_req.valid   := rob.io.flush.valid                  //3、flush_pc_req赋值
  flush_pc_req.bits    := rob.io.flush.bits.ftq_idx

  // Mispredict requests (to get the correct target)          //错误预测请求（以获取正确的目标）,最老的误预测分支
  io.ifu.get_pc(1).ftq_idx := oldest_mispredict_ftq_idx


  //-------------------------------------------------------------
  // Decode/Rename1 pipeline logic

  dec_xcpts := dec_uops zip dec_valids map {case (u,v) => u.exception && v}
  val dec_xcpt_stall = dec_xcpts.reduce(_||_) && !xcpt_pc_req.ready
  // stall fetch/decode because we ran out of branch tags    //停顿fetch/解码，因为我们用完了分支标签
  val branch_mask_full = Wire(Vec(coreWidth, Bool()))

  //解码危害
  val dec_hazards = (0 until coreWidth).map(w =>
                      dec_valids(w) &&
                      (  !dis_ready
                      || rob.io.commit.rollback
                      || dec_xcpt_stall
                      || branch_mask_full(w)
                      || brupdate.b1.mispredict_mask =/= 0.U
                      || brupdate.b2.mispredict
                      || io.ifu.redirect_flush))             //重定向刷新，刷新并挂住前端？

  val dec_stalls = dec_hazards.scanLeft(false.B) ((s,h) => s || h).takeRight(coreWidth) //解码暂停
  dec_fire := (0 until coreWidth).map(w => dec_valids(w) && !dec_stalls(w))    //解码有效且未暂停，发射

  // all decoders are empty and ready for new instructions   //所有解码器都是空的，可以接受新指令
  dec_ready := dec_fire.last

  when (dec_ready || io.ifu.redirect_flush) {
    dec_finished_mask := 0.U
  } .otherwise {
    dec_finished_mask := dec_fire.asUInt | dec_finished_mask
  }

  //-------------------------------------------------------------
  // Branch Mask Logic
  //跟踪当前的“分支掩码”，并在“解码”中将分支掩码分发给每个微操作
  //(机器中的每个微型操作都有一个分支掩码，该分支掩码指出要在哪个分支下进行推测）。

  dec_brmask_logic.io.brupdate := brupdate
  dec_brmask_logic.io.flush_pipeline := RegNext(rob.io.flush.valid)

  for (w <- 0 until coreWidth) {
    dec_brmask_logic.io.is_branch(w) := !dec_finished_mask(w) && dec_uops(w).allocate_brtag   //allocate_brtag:我们是否为此分配了分支标记？
    dec_brmask_logic.io.will_fire(w) :=  dec_fire(w) &&
                                         dec_uops(w).allocate_brtag // rename, dispatch can back pressure us
    dec_uops(w).br_tag  := dec_brmask_logic.io.br_tag(w)
    dec_uops(w).br_mask := dec_brmask_logic.io.br_mask(w)
  }

  branch_mask_full := dec_brmask_logic.io.is_full            //用完了分支标签

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Register Rename Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Inputs
  for (rename <- rename_stages) {
    rename.io.kill := io.ifu.redirect_flush         //重定向flush触发 重命名的kill信号
    rename.io.brupdate := brupdate

    rename.io.debug_rob_empty := rob.io.empty

    rename.io.dec_fire := dec_fire                 //decode的信号
    rename.io.dec_uops := dec_uops

    rename.io.dis_fire := dis_fire                 //dispatch的信号
    rename.io.dis_ready := dis_ready

    rename.io.com_valids := rob.io.commit.valids   //commit的信号
    rename.io.com_uops := rob.io.commit.uops
    rename.io.rbk_valids := rob.io.commit.rbk_valids  //执行重命名状态的回滚（与commit.ups一起使用）。
    rename.io.rollback := rob.io.commit.rollback
  }


  // Outputs
  dis_uops := rename_stage.io.ren2_uops           //dispatch微操作 //ren2_uops:物理说明符可用，busy/ready状态可用。
  dis_valids := rename_stage.io.ren2_mask         //dispatch有效性
  ren_stalls := rename_stage.io.ren_stalls        //rename暂停



  /**
   * TODO This is a bit nasty, but it's currently necessary to
   * split the INT/FP rename pipelines into separate instantiations.
   * Won't have to do this anymore with a properly decoupled FP pipeline.
   */
  //TODO这有点令人讨厌，但是当前有必要将 INT / FP 重命名管道拆分为单独的实例。
  //不再需要使用正确解耦的FP管道来执行此操作。
  for (w <- 0 until coreWidth) {
    val i_uop   = rename_stage.io.ren2_uops(w)
    val f_uop   = if (usingFPU) fp_rename_stage.io.ren2_uops(w) else NullMicroOp
    val p_uop   = if (enableSFBOpt) pred_rename_stage.io.ren2_uops(w) else NullMicroOp
    val f_stall = if (usingFPU) fp_rename_stage.io.ren_stalls(w) else false.B
    val p_stall = if (enableSFBOpt) pred_rename_stage.io.ren_stalls(w) else false.B

    // lrs1 can "pass through" to prs1. Used solely to index the csr file.  //lrs1可以“通过”到prs1。 仅用于索引csr文件。
    dis_uops(w).prs1 := Mux(dis_uops(w).lrs1_rtype === RT_FLT, f_uop.prs1,
                        Mux(dis_uops(w).lrs1_rtype === RT_FIX, i_uop.prs1, dis_uops(w).lrs1))
    dis_uops(w).prs2 := Mux(dis_uops(w).lrs2_rtype === RT_FLT, f_uop.prs2, i_uop.prs2)
    dis_uops(w).prs3 := f_uop.prs3
    dis_uops(w).ppred := p_uop.ppred
    dis_uops(w).pdst := Mux(dis_uops(w).dst_rtype  === RT_FLT, f_uop.pdst,
                        Mux(dis_uops(w).dst_rtype  === RT_FIX, i_uop.pdst,
                                                               p_uop.pdst))
    dis_uops(w).stale_pdst := Mux(dis_uops(w).dst_rtype === RT_FLT, f_uop.stale_pdst, i_uop.stale_pdst)

    dis_uops(w).prs1_busy := i_uop.prs1_busy && (dis_uops(w).lrs1_rtype === RT_FIX) ||
                             f_uop.prs1_busy && (dis_uops(w).lrs1_rtype === RT_FLT)
    dis_uops(w).prs2_busy := i_uop.prs2_busy && (dis_uops(w).lrs2_rtype === RT_FIX) ||
                             f_uop.prs2_busy && (dis_uops(w).lrs2_rtype === RT_FLT)
    dis_uops(w).prs3_busy := f_uop.prs3_busy && dis_uops(w).frs3_en
    dis_uops(w).ppred_busy := p_uop.ppred_busy && dis_uops(w).is_sfb_shadow

    ren_stalls(w) := rename_stage.io.ren_stalls(w) || f_stall || p_stall



  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Dispatch Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  //-------------------------------------------------------------
  // Rename2/Dispatch pipeline logic

  val dis_prior_slot_valid = dis_valids.scanLeft(false.B) ((s,v) => s || v)  //dispatch先前的slot有效
  val dis_prior_slot_unique = (dis_uops zip dis_valids).scanLeft(false.B) {case (s,(u,v)) => s || v && u.is_unique}  //dispatch先前的slot unique
  val wait_for_empty_pipeline = (0 until coreWidth).map(w => (dis_uops(w).is_unique || custom_csrs.disableOOO) &&
                                  (!rob.io.empty || !io.lsu.fencei_rdy || dis_prior_slot_valid(w)))    //等待空的流水线
  val rocc_shim_busy = if (usingRoCC) !exe_units.rocc_unit.io.rocc.rxq_empty else false.B
  val wait_for_rocc = (0 until coreWidth).map(w =>
                        (dis_uops(w).is_fence || dis_uops(w).is_fencei) && (io.rocc.busy || rocc_shim_busy))
  val rxq_full = if (usingRoCC) exe_units.rocc_unit.io.rocc.rxq_full else false.B
  val block_rocc = (dis_uops zip dis_valids).map{case (u,v) => v && u.uopc === uopROCC}.scanLeft(rxq_full)(_||_)
  val dis_rocc_alloc_stall = (dis_uops.map(_.uopc === uopROCC) zip block_rocc) map {case (p,r) =>
                               if (usingRoCC) p && r else false.B}

  val idle_cycles = freechips.rocketchip.util.WideCounter(32)
  when (
    csr.io.csr_stall ||
      io.rocc.busy ||
      reset.asBool) {
    idle_cycles := 0.U
  }

  val risk_table = RegInit(VecInit(Seq.fill(numIntPhysRegs){false.B}))
  val fp_risk_table = RegInit(VecInit(Seq.fill(numFpPhysRegs){false.B}))
  rob.io.risk_table <> risk_table
  rob.io.fp_risk_table <> fp_risk_table
  fp_pipeline.io.risk_table <> risk_table
  fp_pipeline.io.fp_risk_table <> fp_risk_table
  io.lsu.risk_table <> risk_table
  io.lsu.fp_risk_table <> fp_risk_table
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //如果有已经发出的load且load-mask不为0，则暂停此处load(之后再考虑数据依赖)///////////////////////////////////////////////////////////////////////////////////////////////////
  for (w <- 0 until coreWidth) {
    when(dis_uops(w).debug_inst === 0x02f57463L.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-c6-dispatch \n")
    }
    when(dis_uops(w).debug_inst === 0x00054703L.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-d4-dispatch \n")
    }
    when(dis_uops(w).debug_inst(15,0) === 0x97baL.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-e0-dispatch \n")
    }
    when(dis_uops(w).debug_inst === 0x0007c783L.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-e2-dispatch \n")
    }/*
    when(dis_uops(w).debug_inst === 0x00679713L.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-88-dispatch \n")
    }
    when(dis_uops(w).debug_inst === 0x9e478793L.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-8c-dispatch \n")
    }
    when(dis_uops(w).debug_inst(15,0) === 0x97baL.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-90-dispatch \n")
    }
    when(dis_uops(w).debug_inst === 0x0007c783L.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" prs1=${dis_uops(w).prs1} ")
      printf(p" prs2=${dis_uops(w).prs2} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-92-dispatch \n")
    }
    when(dis_uops(w).debug_inst === 0x9af70b23L.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-aa-dispatch \n")
    }
    when(dis_uops(w).debug_inst(15,0) === 0x8082L.U && dis_uops(w).debug_pc === 0x800010eeL.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-ae-dispatch \n")
    }
    when(dis_uops(w).debug_pc === 0x8000110aL.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-8082-dispatch \n")
    }
    when(dis_uops(w).debug_inst(15,0) === 0x631cL.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-631c-dispatch \n")
    }
    when(dis_uops(w).debug_inst(15,0) === 0x07a2L.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-07a2-dispatch \n")
    }
    when(dis_uops(w).debug_pc === 0x800010a0L.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
     // printf(p" risk_table=${risk_table} ")
      printf(p" found-0x800010a0L-dispatch \n")
    }
    when(dis_uops(w).debug_pc === 0x800010a4L.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
      //printf(p" risk_table=${risk_table} ")
      printf(p" found-0x800010a4L-dispatch \n")
    }
    when(dis_uops(w).debug_pc === 0x800010bcL.U && dis_valids(w)===1.U) {
      printf(p" mask=${dis_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${dis_uops(w).pdst} ")
      //printf(p" risk_table=${risk_table} ")
      printf(p" found-0x800010bcL-dispatch \n")
    }*/
  }

               //dispatch的故障
  val dis_hazards = (0 until coreWidth).map(w =>
                      dis_valids(w) &&
                      (  !rob.io.ready
                      || ren_stalls(w)
                      || io.lsu.ldq_full(w) && dis_uops(w).uses_ldq
                      || io.lsu.stq_full(w) && dis_uops(w).uses_stq
                      || !dispatcher.io.ren_uops(w).ready
                      || wait_for_empty_pipeline(w)
                      || wait_for_rocc(w)
                      || dis_prior_slot_unique(w)
                      || dis_rocc_alloc_stall(w)
                      || brupdate.b1.mispredict_mask =/= 0.U
                      || brupdate.b2.mispredict
                      || io.ifu.redirect_flush           // 重定向刷新
                      ))


  io.lsu.fence_dmem := (dis_valids zip wait_for_empty_pipeline).map {case (v,w) => v && w} .reduce(_||_)  //fence_dmem:告诉DCache清除预取/推测未命中,具体作用是啥？

  val dis_stalls = dis_hazards.scanLeft(false.B) ((s,h) => s || h).takeRight(coreWidth)   //dispatch暂停
  dis_fire := dis_valids zip dis_stalls map {case (v,s) => v && !s}                       //dispatch发出
  dis_ready := !dis_stalls.last                                                           //dispatch准备 //.last是什么意思

  //-------------------------------------------------------------
  // LDQ/STQ Allocation Logic

  for (w <- 0 until coreWidth) {
    // Dispatching instructions request load/store queue entries when they can proceed.  //调度指令会在继续前进时 请求加载/存储队列条目。
    dis_uops(w).ldq_idx := io.lsu.dis_ldq_idx(w)
    dis_uops(w).stq_idx := io.lsu.dis_stq_idx(w)
  }



  for (i <- 0 until coreWidth) {

    when(dis_uops(i).debug_inst(15,0) === 0x97baL.U)
    {
      printf(p"pdst-1.0=${dis_uops(i).pdst} ")
      printf(p"mask-1.0=${dis_uops(i).br_mask =/= 0.U} ")
      printf(p"uses_ldq-1.0=${dis_uops(i).uses_ldq} ")
      printf(p" risk_table(dis_uops(i).prs1)=${risk_table(dis_uops(i).prs1)} ")
      printf(p" risk_table(dis_uops(i).prs2)=${risk_table(dis_uops(i).prs2)} ")
      printf(p" risk_table(dis_uops(i).prs3)=${risk_table(dis_uops(i).prs3)} ")
      printf(p" fp=${dis_uops(i).fp_val} ")
      printf("inst= 0x%x \n", dis_uops(i).debug_inst)
    }

    when(dis_uops(i).br_mask =/= 0.U && dis_uops(i).uses_ldq && ((!dis_uops(i).fp_val && dis_uops(i).pdst =/= 0.U && risk_table(dis_uops(i).prs1) === 0.B && risk_table(dis_uops(i).prs2) === 0.B && risk_table(dis_uops(i).prs3) === 0.B) || (dis_uops(i).fp_val && fp_risk_table(dis_uops(i).prs1) === 0.B && fp_risk_table(dis_uops(i).prs2) === 0.B && fp_risk_table(dis_uops(i).prs3) === 0.B)) ){
      when(dis_fire(i) ) {  //&& dis_ready(i)
        when(dis_uops(i).fp_val){
          fp_risk_table(dis_uops(i).pdst) := true.B
        } .otherwise {
          risk_table(dis_uops(i).pdst) := true.B
        }
        //dis_uops(i).risk := true.B
      }
      //printf("debug-inst= 0x%x ", int_iss_wakeups(i).bits.uop.debug_inst)
      //printf("debug-pdst-1= %d \n", int_iss_wakeups(i).bits.uop.pdst)
      when(dis_uops(i).pdst === 16.U){
        printf(p"pdst-1.1=${dis_uops(i).pdst} ")
        printf(p" fp=${dis_uops(i).fp_val} ")
        printf("inst= 0x%x \n", dis_uops(i).debug_inst)
      }
    }
      .elsewhen(dis_uops(i).br_mask =/= 0.U && ((dis_uops(i).fp_val && (fp_risk_table(dis_uops(i).prs1)===true.B || fp_risk_table(dis_uops(i).prs2)===true.B || fp_risk_table(dis_uops(i).prs3)===true.B)) || (!dis_uops(i).fp_val && dis_uops(i).pdst =/= 0.U && (risk_table(dis_uops(i).prs1)===true.B || risk_table(dis_uops(i).prs2)===true.B || risk_table(dis_uops(i).prs3)===true.B))) ){ //&& !dis_uops(i).uses_ldq
        when(dis_fire(i) ) {  //&& dis_ready(i)
          when(dis_uops(i).fp_val){
            fp_risk_table(dis_uops(i).pdst) := true.B
          } .otherwise {
            risk_table(dis_uops(i).pdst) := true.B
          }
          //dis_uops(i).risk := true.B
        }

        when(dis_uops(i).pdst === 37.U){
          printf(p"pdst-1.2=${dis_uops(i).pdst} ")
          printf(p" fp=${dis_uops(i).fp_val} ")
          printf("inst= 0x%x \n", dis_uops(i).debug_inst)
        }
        //printf("rename-not-load 0x%x \n", io.ren_uops(i).debug_inst)
        when(dis_uops(i).debug_inst === 0x0007c783L.U){
          printf(p"0x07a2L:io.ren_uops(i).pdst=${dis_uops(i).pdst} ")
          printf(p"mask=${dis_uops(i).br_mask} ")
          printf(p" prs1=${dis_uops(i).prs1} ")
          printf(p" risk=${risk_table(dis_uops(i).prs1)}")
          printf(p" prs2=${dis_uops(i).prs2} ")
          printf(p" risk=${risk_table(dis_uops(i).prs2)}")
          printf(p" prs3=${dis_uops(i).prs3} ")
          printf(p" risk=${risk_table(dis_uops(i).prs3)}")
          printf(p" fp=${dis_uops(i).fp_val} ")
          printf(p" risk_table=${risk_table} \n")
        }
        /*when(dis_uops(i).pdst === 38.U){
          printf(p"0x07a2L:pdst-1.2=${dis_uops(i).pdst} ")
          printf(p" fp=${dis_uops(i).fp_val} ")
          printf("inst= 0x%x \n", dis_uops(i).debug_inst)
        }*/
      }
      .otherwise{
        //dis_uops(i).risk := false.B
      }
  }


   for(i <- 0 until 10) {
      when(rob.io.clear_risk_table(i) =/= 0.U) {
        when(rob.io.clear_risk_table(i) < numIntPhysRegs.U) {
          risk_table(rob.io.clear_risk_table(i)) := false.B
        } .otherwise{
          fp_risk_table(rob.io.clear_risk_table(i) - numIntPhysRegs.U) := false.B
        }
      }
   }

  when(rob.io.flush.valid){
    for(i <- 0 until numIntPhysRegs){
      risk_table(i) := false.B
    }
    for(i <- 0 until numFpPhysRegs){
      fp_risk_table(i) := false.B
    }
  }


  //-------------------------------------------------------------
  // Rob Allocation Logic

  rob.io.enq_valids := dis_fire               //rob入队有效性
  rob.io.enq_uops   := dis_uops               //rob入队微操作
  rob.io.enq_partial_stall := dis_stalls.last // TODO come up with better ROB compacting scheme.  //TODO:提出更好的ROB压缩方案。
  rob.io.debug_tsc := debug_tsc_reg
  rob.io.csr_stall := csr.io.csr_stall




  // Minor hack: ecall and breaks need to increment the FTQ deq ptr earlier than commit, since
  // they write their PC into the CSR the cycle before they commit.
  // Since these are also unique, increment the FTQ ptr when they are dispatched
  //轻微入侵：ecall和breaks必须在commit之前增加FTQ deq ptr，因为他们在commit之前将PC写入CSR。
  //由于它们也是唯一的，因此在分派它们时增加FTQ ptr
   //is_sys_pc2epc: Is a ECall or Breakpoint -- both set EPC to PC.
  when (RegNext(dis_fire.reduce(_||_) && dis_uops(PriorityEncoder(dis_fire)).is_sys_pc2epc)) {
    io.ifu.commit.valid := true.B
    io.ifu.commit.bits  := RegNext(dis_uops(PriorityEncoder(dis_valids)).ftq_idx)                //ftq_idx:索引FTQ以找出我们的fetch PC。
  }

  for (w <- 0 until coreWidth) {
    // note: this assumes uops haven't been shifted - there's a 1:1 match between PC's LSBs and "w" here
    // (thus the LSB of the rob_idx gives part of the PC)
    //注意：这是假设uops尚未移动-PC的LSB与“w”之间存在1：1匹配
    //（因此rob_idx的LSB为PC的一部分）
    if (coreWidth == 1) {
      dis_uops(w).rob_idx := rob.io.rob_tail_idx
    } else {
      dis_uops(w).rob_idx := Cat(rob.io.rob_tail_idx >> log2Ceil(coreWidth).U,
                               w.U(log2Ceil(coreWidth).W))
    }
  }

  //-------------------------------------------------------------
  // RoCC allocation logic
  if (usingRoCC) {
    for (w <- 0 until coreWidth) {
      // We guarantee only decoding 1 RoCC instruction per cycle    //我们保证每个周期仅解码1条RoCC指令
      dis_uops(w).rxq_idx := exe_units.rocc_unit.io.rocc.rxq_idx(w)
    }
  }

  //-------------------------------------------------------------
  // Dispatch to issue queues                                      //dispatch到issue队列

  // Get uops from rename2                                         //从rename2阶段得到微操作
  for (w <- 0 until coreWidth) {
    dispatcher.io.ren_uops(w).valid := dis_fire(w)
    dispatcher.io.ren_uops(w).bits  := dis_uops(w)
  }

  var iu_idx = 0
  // Send dispatched uops to correct issue queues                 //发送dispatch的微指令到正确的issue队列，
  // Backpressure through dispatcher if necessary                 //如有必要，通过分配器进行backPressure(back Pressure是啥)
  for (i <- 0 until issueParams.size) {
    if (issueParams(i).iqType == IQT_FP.litValue) {
       fp_pipeline.io.dis_uops <> dispatcher.io.dis_uops(i)
    } else {
       issue_units(iu_idx).io.dis_uops <> dispatcher.io.dis_uops(i)
       iu_idx += 1
      
    }
  }





  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Issue Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  require (issue_units.map(_.issueWidth).sum == exe_units.length)   //要求issue单元宽度之和等于执行单元的长度

  var iss_wu_idx = 1
  var ren_wu_idx = 1
  // The 0th wakeup port goes to the ll_wbarb                       //第0个唤醒端口进入ll_wbarb（ll_wb）, load在此处

  val wakeup_back_yes   = Reg(Vec(rob.numWakeupPorts+1, Bool()))         //不该是rob.numWakeupPorts+1
  for(i <- 0 until rob.numWakeupPorts+1) {
    wakeup_back_yes(i) := false.B
  }
  for(i <- 0 until rob.numWakeupPorts+1) {
    rob.io.wakeup_yes(i) := wakeup_back_yes(i)
  }


   //rob.io.wakeup_valid(rob.io.wakeup_i(0)) &&
    //当从rob中到来的load唤醒与 正常的load唤醒一起到来时，先发出正常的load唤醒；  之后等正常load唤醒完毕，再唤醒rob load
  when(ll_wbarb.io.out.fire() && ll_wbarb.io.out.bits.uop.dst_rtype === RT_FIX){
    int_iss_wakeups(0).valid := ll_wbarb.io.out.fire() && ll_wbarb.io.out.bits.uop.dst_rtype === RT_FIX
    int_iss_wakeups(0).bits := ll_wbarb.io.out.bits
    int_iss_wakeups(0).bits.uop.comefrom_rob := false.B
    printf("core-wakeup1 ")
    printf(" inst=0x%x ",ll_wbarb.io.out.bits.uop.debug_inst)
    printf(p" pdst=${ll_wbarb.io.out.bits.uop.pdst} ")
    printf(p" cycles=${idle_cycles.value} \n")

  } .elsewhen(rob.io.wakeup_valid(rob.io.wakeup_i(0)) && !wakeup_back_yes(rob.io.wakeup_i(0)) ){
    int_iss_wakeups(0).valid := rob.io.wakeup_valid(rob.io.wakeup_i(0))
    int_iss_wakeups(0).bits := DontCare
    int_iss_wakeups(0).bits.uop.pdst := rob.io.wakeup_pdst(rob.io.wakeup_i(0))
    int_iss_wakeups(0).bits.uop.comefrom_rob := true.B
    //发出rob load唤醒成功信号给rob
    wakeup_back_yes(rob.io.wakeup_i(0)) := true.B

    printf("core-wakeup2 ")
    printf(p" rob.io.wakeup_i(0)=${rob.io.wakeup_i(0)} ")
    printf(p" pdst=${rob.io.wakeup_pdst(rob.io.wakeup_i(0))} ")
    printf(p" cycles=${idle_cycles.value} \n")
  }
    .elsewhen( (rob.io.wakeup_valid(rob.io.wakeup_i(1)) && !wakeup_back_yes(rob.io.wakeup_i(1)))){
      int_iss_wakeups(0).valid := rob.io.wakeup_valid(rob.io.wakeup_i(1))
      int_iss_wakeups(0).bits := DontCare
      int_iss_wakeups(0).bits.uop.pdst := rob.io.wakeup_pdst(rob.io.wakeup_i(1))
      int_iss_wakeups(0).bits.uop.comefrom_rob := true.B
      //发出rob load唤醒成功信号给rob
      wakeup_back_yes(rob.io.wakeup_i(1)) := true.B

      printf("core-wakeup3 ")
      printf(p" rob.io.wakeup_i(0)=${rob.io.wakeup_i(1)} ")
      printf(p" pdst=${rob.io.wakeup_pdst(rob.io.wakeup_i(1))} ")
      printf(p" cycles=${idle_cycles.value} \n")
    }
    .elsewhen(rob.io.wakeup_valid(rob.io.wakeup_i(2)) && !wakeup_back_yes(rob.io.wakeup_i(2))){
      int_iss_wakeups(0).valid := rob.io.wakeup_valid(rob.io.wakeup_i(2))
      int_iss_wakeups(0).bits := DontCare
      int_iss_wakeups(0).bits.uop.pdst := rob.io.wakeup_pdst(rob.io.wakeup_i(2))
      int_iss_wakeups(0).bits.uop.comefrom_rob := true.B
      //发出rob load唤醒成功信号给rob
      wakeup_back_yes(rob.io.wakeup_i(2)) := true.B

      printf("core-wakeup4 ")
      printf(p" rob.io.wakeup_i(0)=${rob.io.wakeup_i(2)} ")
      printf(p" pdst=${rob.io.wakeup_pdst(rob.io.wakeup_i(2))} ")
      printf(p" cycles=${idle_cycles.value} \n")
    }
    .elsewhen(rob.io.wakeup_valid(rob.io.wakeup_i(3)) && !wakeup_back_yes(rob.io.wakeup_i(3))){
      int_iss_wakeups(0).valid := rob.io.wakeup_valid(rob.io.wakeup_i(3))
      int_iss_wakeups(0).bits := DontCare
      int_iss_wakeups(0).bits.uop.pdst := rob.io.wakeup_pdst(rob.io.wakeup_i(3))
      int_iss_wakeups(0).bits.uop.comefrom_rob := true.B
      //发出rob load唤醒成功信号给rob
      wakeup_back_yes(rob.io.wakeup_i(3)) := true.B

      printf("core-wakeup5 ")
      printf(p" rob.io.wakeup_i(0)=${rob.io.wakeup_i(3)} ")
      printf(p" pdst=${rob.io.wakeup_pdst(rob.io.wakeup_i(3))} ")
      printf(p" cycles=${idle_cycles.value} \n")
    }
    .otherwise{
    int_iss_wakeups(0).valid := ll_wbarb.io.out.fire() && ll_wbarb.io.out.bits.uop.dst_rtype === RT_FIX
    int_iss_wakeups(0).bits := ll_wbarb.io.out.bits
    int_iss_wakeups(0).bits.uop.comefrom_rob := false.B
  }



  //还原rob load唤醒成功信号
  for(i <- 0 until 4) {
    when(wakeup_back_yes(i) === true.B) {
      wakeup_back_yes(i) := RegNext(false.B)
    }
  }
  //挤占式rob唤醒会让正常唤醒被挤掉
  


  //int_iss_wakeups(0).valid := ll_wbarb.io.out.fire() && ll_wbarb.io.out.bits.uop.dst_rtype === RT_FIX


  int_ren_wakeups(0).valid := ll_wbarb.io.out.fire() && ll_wbarb.io.out.bits.uop.dst_rtype === RT_FIX
  int_ren_wakeups(0).bits  := ll_wbarb.io.out.bits

  for (i <- 1 until memWidth) {
    int_iss_wakeups(i).valid := mem_resps(i).valid && mem_resps(i).bits.uop.dst_rtype === RT_FIX
    int_iss_wakeups(i).bits  := mem_resps(i).bits

    int_ren_wakeups(i).valid := mem_resps(i).valid && mem_resps(i).bits.uop.dst_rtype === RT_FIX
    int_ren_wakeups(i).bits  := mem_resps(i).bits
    iss_wu_idx += 1
    ren_wu_idx += 1
  }

  // loop through each issue-port (exe_units are statically connected to an issue-port)  //遍历每个issue端口（exe_units静态连接到发布端口）
  for (i <- 0 until exe_units.length) {
    if (exe_units(i).writesIrf) {
      val fast_wakeup = Wire(Valid(new ExeUnitResp(xLen)))
      val slow_wakeup = Wire(Valid(new ExeUnitResp(xLen)))
      fast_wakeup := DontCare
      slow_wakeup := DontCare

      val resp = exe_units(i).io.iresp
      assert(!(resp.valid && resp.bits.uop.rf_wen && resp.bits.uop.dst_rtype =/= RT_FIX))
  //rf_wen:该寄存器是否回写,RT_FIX具体含义？

      // Fast Wakeup (uses just-issued uops that have known latencies)                     //快速唤醒（使用具有已知延迟的刚刚发出的uops）
            //ldst_val:有目的地吗？ 对于store无效，rd == x0，依此类推。
      fast_wakeup.bits.uop := iss_uops(i)
      fast_wakeup.valid    := iss_valids(i) &&
                              iss_uops(i).bypassable &&
                              iss_uops(i).dst_rtype === RT_FIX &&
                              iss_uops(i).ldst_val &&
                              !(io.lsu.ld_miss && (iss_uops(i).iw_p1_poisoned || iss_uops(i).iw_p2_poisoned))
  //是否通过负载推测性唤醒了操作数1或2？
   //只有整数操作数被推测唤醒，因此我们可以忽略p3。
     //ld_miss:告诉IQs，我们上个周期推测的负载被错误推测

      // Slow Wakeup (uses write-port to register file)                                   //慢唤醒（使用写端口寄存器文件）
      slow_wakeup.bits.uop := resp.bits.uop
      slow_wakeup.valid    := resp.valid &&
                                resp.bits.uop.rf_wen &&
                                !resp.bits.uop.bypassable &&
                                resp.bits.uop.dst_rtype === RT_FIX


      if (exe_units(i).bypassable) {
        int_iss_wakeups(iss_wu_idx) := fast_wakeup
        iss_wu_idx += 1
      }
      if (!exe_units(i).alwaysBypassable) {
        int_iss_wakeups(iss_wu_idx) := slow_wakeup
        iss_wu_idx += 1
      }

      if (exe_units(i).bypassable) {
        int_ren_wakeups(ren_wu_idx) := fast_wakeup
        ren_wu_idx += 1
      }
      if (!exe_units(i).alwaysBypassable) {
        int_ren_wakeups(ren_wu_idx) := slow_wakeup
        ren_wu_idx += 1
      }
    }
  }


  
  require (iss_wu_idx == numIntIssueWakeupPorts)                            //iss_wakeup_idx(整数issue唤醒端口数量)
  require (ren_wu_idx == numIntRenameWakeupPorts)                           //ren_wakeup_idx(整数rename唤醒端口数量)
  require (iss_wu_idx == ren_wu_idx)

  // jmp unit performs fast wakeup of the predicate bits                    //jmp单元执行预测位的快速唤醒
  require (jmp_unit.bypassable)
                 //is_sfb_br:是br.是sfb.启用sfb.    sfb是啥？
  pred_wakeup.valid := (iss_valids(jmp_unit_idx) &&
                        iss_uops(jmp_unit_idx).is_sfb_br &&
                        !(io.lsu.ld_miss && (iss_uops(jmp_unit_idx).iw_p1_poisoned || iss_uops(jmp_unit_idx).iw_p2_poisoned))
  )
  pred_wakeup.bits.uop := iss_uops(jmp_unit_idx)
  pred_wakeup.bits.fflags := DontCare
  pred_wakeup.bits.data := DontCare
  pred_wakeup.bits.predicated := DontCare

  // Perform load-hit speculative wakeup through a special port (performs a poison wake-up).    //通过特殊端口执行负载命中的推测性唤醒（执行有毒唤醒）。
  issue_units map { iu =>
     iu.io.spec_ld_wakeup := io.lsu.spec_ld_wakeup
  }


  // Connect the predicate wakeup port         //连接predicate唤醒端口
  issue_units map { iu =>
    iu.io.pred_wakeup_port.valid := false.B
    iu.io.pred_wakeup_port.bits := DontCare
  }
  if (enableSFBOpt) {
    int_iss_unit.io.pred_wakeup_port.valid := pred_wakeup.valid
    int_iss_unit.io.pred_wakeup_port.bits := pred_wakeup.bits.uop.pdst
  }


  // ----------------------------------------------------------------
  // Connect the wakeup ports to the busy tables in the rename stages    //将唤醒端口连接到 重命名阶段的繁忙表

  for ((renport, intport) <- rename_stage.io.wakeups zip int_ren_wakeups) {
    renport <> intport
  }
  if (usingFPU) {
    for ((renport, fpport) <- fp_rename_stage.io.wakeups zip fp_pipeline.io.wakeups) {
       renport <> fpport
    }
  }
               //启用sfb（sfb与predicate的关系）
  if (enableSFBOpt) {
    pred_rename_stage.io.wakeups(0) := pred_wakeup
  } else {
    pred_rename_stage.io.wakeups := DontCare
  }

  // If we issue loads back-to-back endlessly (probably because we are executing some tight loop)
  // the store buffer will never drain, breaking the memory-model forward-progress guarantee
  // If we see a large number of loads saturate the LSU, pause for a cycle to let a store drain
  //如果我们不断地连续发出负载（可能是因为我们正在执行一些紧密循环）
  //存储缓冲区将永远不会排空，从而破坏了内存模型的前向保证
  //如果我们看到大量负载使LSU饱和，请暂停一个周期以使存储排空
  val loads_saturating = (mem_iss_unit.io.iss_valids(0) && mem_iss_unit.io.iss_uops(0).uses_ldq)   //负载饱和
  val saturating_loads_counter = RegInit(0.U(5.W))             //饱和load计数器
  when (loads_saturating) { saturating_loads_counter := saturating_loads_counter + 1.U }
  .otherwise { saturating_loads_counter := 0.U }
  val pause_mem = RegNext(loads_saturating) && saturating_loads_counter === ~(0.U(5.W))

  var iss_idx = 0
  var int_iss_cnt = 0
  var mem_iss_cnt = 0
  for (w <- 0 until exe_units.length) {
    var fu_types = exe_units(w).io.fu_types
    val exe_unit = exe_units(w)
    if (exe_unit.readsIrf) {
      if (exe_unit.supportedFuncUnits.muld) {
        // Supress just-issued divides from issuing back-to-back, since it's an iterative divider.
        // But it takes a cycle to get to the Exe stage, so it can't tell us it is busy yet.
        //因为它是一个迭代式除法器，所以禁止从刚发行到发行的除法。 //但是进入Exe阶段需要一个周期，所以它不能告诉我们它现在很忙。
        val idiv_issued = iss_valids(iss_idx) && iss_uops(iss_idx).fu_code_is(FU_DIV)
        fu_types = fu_types & RegNext(~Mux(idiv_issued, FU_DIV, 0.U))
      }

                  //执行单元有mem单元，则对mem赋值
      if (exe_unit.hasMem) {
        iss_valids(iss_idx) := mem_iss_unit.io.iss_valids(mem_iss_cnt)
        iss_uops(iss_idx)   := mem_iss_unit.io.iss_uops(mem_iss_cnt)
        mem_iss_unit.io.fu_types(mem_iss_cnt) := Mux(pause_mem, 0.U, fu_types)
        mem_iss_cnt += 1
      } else {
        iss_valids(iss_idx) := int_iss_unit.io.iss_valids(int_iss_cnt)
        iss_uops(iss_idx)   := int_iss_unit.io.iss_uops(int_iss_cnt)
        int_iss_unit.io.fu_types(int_iss_cnt) := fu_types
        int_iss_cnt += 1
      }
      iss_idx += 1
    }
  }
  require(iss_idx == exe_units.numIrfReaders)       //iss_idx等于执行单元读端口数量

  issue_units.map(_.io.tsc_reg := debug_tsc_reg)
  issue_units.map(_.io.brupdate := brupdate)        //issue单元更新 分支更新信息
  issue_units.map(_.io.flush_pipeline := RegNext(rob.io.flush.valid))   //issue单元更新 流水线刷新信息

  // Load-hit Misspeculations     //负载命中错误推测
  require (mem_iss_unit.issueWidth <= 2)       //issueWidth:一次可以issue的操作数量
  issue_units.map(_.io.ld_miss := io.lsu.ld_miss)       //issue单元更新 告诉IQs，我们推测的上一个周期的load是错误的

  mem_units.map(u => u.io.com_exception := RegNext(rob.io.flush.valid))  //mem单元更新 用rob flush更新com_exception




  for(iu <- issue_units) {
    iu.io.idle_cycles := idle_cycles.value
    iu.io.return_issue := rob.io.return_issue
  }


  // Wakeup (Issue & Writeback)        //唤醒（issue和写回）                 //////////////////////////////////////////执行完的指令唤醒其他依赖的指令//////////////////////////////
  for {
    iu <- issue_units
    (issport, wakeup) <- iu.io.wakeup_ports zip int_iss_wakeups
  }{
    issport.valid := wakeup.valid
    //issport.valid := wakeup.valid
    issport.bits.pdst := wakeup.bits.uop.pdst
    issport.bits.poisoned := wakeup.bits.uop.iw_p1_poisoned || wakeup.bits.uop.iw_p2_poisoned
    issport.bits.debug_inst := wakeup.bits.uop.debug_inst
    issport.bits.br_mask := (wakeup.bits.uop.br_mask =/= 0.U)
    issport.bits.risk_rob_idx := wakeup.bits.uop.rob_idx
    //issport.bits.risk := wakeup.bits.uop.risk
    when(wakeup.bits.uop.comefrom_rob){
      issport.bits.risk := false.B
    } .otherwise{
      issport.bits.risk := Mux(wakeup.bits.uop.fp_val,fp_risk_table(wakeup.bits.uop.pdst),risk_table(wakeup.bits.uop.pdst))
    }

      //printf(" 0x%x ", Sext(wakeup.bits.uop.debug_pc(vaddrBits-1,0), xLen))
      //printf(p" pdst =${wakeup.bits.uop.pdst} ")
      //printf(p" prs1 =${wakeup.bits.uop.prs1} ")
      //printf(p" prs2 =${wakeup.bits.uop.prs2}\n")



    //(wakeup.bits.uop.uses_ldq && wakeup.bits.uop.br_mask.asUInt =/= 0.U)
    //issport.valid := wakeup.valid
    //issport.bits.pdst := wakeup.bits.uop.pdst
    //issport.bits.poisoned := wakeup.bits.uop.iw_p1_poisoned || wakeup.bits.uop.iw_p2_poisoned

    /*when((mem_resps(i).bits.uop.uses_ldq && mem_resps(i).bits.uop.br_mask.asUInt =/= 0.U)) {
       issport.valid := RegEnable(wakeup.valid,)
    } .elsewhen((mem_resps(i).bits.uop.uses_ldq && mem_resps(i).bits.uop.br_mask.asUInt === 0.U)) {
       issport.valid := wakeup.valid
    }*/

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    require (iu.io.wakeup_ports.length == int_iss_wakeups.length) //要求issue_units的唤醒端口的长度等于 int_iss_wakeups的长度   3
  }



  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Register Read Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Register Read <- Issue (rrd <- iss)             //（issue到寄存器读）， 输入信号给iregister_read
  iregister_read.io.rf_read_ports <> iregfile.io.read_ports
  iregister_read.io.prf_read_ports := DontCare
  if (enableSFBOpt) {
    iregister_read.io.prf_read_ports <> pregfile.io.read_ports
  }

           //iregister_read 的issue有效性赋值
  for (w <- 0 until exe_units.numIrfReaders) {
    iregister_read.io.iss_valids(w) :=
      iss_valids(w) && !(io.lsu.ld_miss && (iss_uops(w).iw_p1_poisoned || iss_uops(w).iw_p2_poisoned))
  }
  iregister_read.io.iss_uops := iss_uops
  iregister_read.io.iss_uops map { u => u.iw_p1_poisoned := false.B; u.iw_p2_poisoned := false.B }

  iregister_read.io.brupdate := brupdate
  iregister_read.io.kill   := RegNext(rob.io.flush.valid)     //rob刷新则触发kill信号

  iregister_read.io.bypass := bypasses                        //可否绕过
  iregister_read.io.pred_bypass := pred_bypasses              //jmp绕过

  for (w <- 0 until exe_units.numIrfReaders) {
    when(iss_uops(w).debug_inst === 0x02f57463L.U && iss_valids(w)===1.U) {
      printf(p" mask=${iss_uops(w).br_mask}")   //76分派之后，发出之前，分支解析成功
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" found-c6-issued \n")
    }
    when(iss_uops(w).debug_inst === 0x00054703L.U && iss_valids(w)===1.U) {
      printf(p" mask=${iss_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" found-d4-issued \n")
    }
    when(iss_uops(w).debug_inst(15,0) === 0x97baL.U && iss_valids(w)===1.U) {
      printf(p" mask=${iss_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" found-e0-issued \n")
    }
    when(iss_uops(w).debug_inst === 0x0007c783L.U && iss_valids(w)===1.U) {
      printf(p" mask=${iss_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${iss_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-e2-issued \n")
    }
    /*when(iss_uops(w).debug_inst === 0x9e478793L.U && iss_valids(w)===1.U) {
      printf(p" mask=${iss_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" found-8c-issued \n")
    }
    when(iss_uops(w).debug_inst(15,0) === 0x97baL.U && iss_valids(w)===1.U) {
      printf(p" mask=${iss_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" pdst=${iss_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-90-issued \n")
    }
    when(iss_uops(w).debug_inst === 0x0007c783L.U && iss_valids(w)===1.U) {
      printf(p" mask=${iss_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" prs1=${iss_uops(w).prs1} ")
      printf(p" prs2=${iss_uops(w).prs2} ")
      printf(p" risk-prs1=${risk_table(iss_uops(w).prs1)} ")
      printf(p" risk-prs2=${risk_table(iss_uops(w).prs2)} ")
      printf(p" pdst=${iss_uops(w).pdst} ")
      printf(p" risk_table=${risk_table} ")
      printf(p" found-92-issued \n")
    }
     when(iss_uops(w).debug_inst === 0x9af70b23L.U && iss_valids(w)===1.U) {
       printf(p" mask=${iss_uops(w).br_mask}")
       printf(p" cycles=${idle_cycles.value} ")
      printf(p" found-aa-issued \n")
    }
     when(iss_uops(w).debug_inst(15,0) === 0x8082L.U && iss_uops(w).debug_pc === 0x800010eeL.U && iss_valids(w)===1.U) {
       printf(p" mask=${iss_uops(w).br_mask}")
       printf(p" cycles=${idle_cycles.value} ")
       printf(p" found-ae-issued \n")
    }
    when(iss_uops(w).debug_inst(15,0) === 0x631cL.U && iss_valids(w)===1.U) {
      printf(p" mask=${iss_uops(w).br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" found-631c-issued \n")
    }*/
  }

  //-------------------------------------------------------------
  // Privileged Co-processor 0 Register File
  // Note: Normally this would be bad in that I'm writing state before
  // committing, so to get this to work I stall the entire pipeline for
  // CSR instructions so I never speculate these instructions.
  //特权协处理器0寄存器文件
  //注意：通常这很不好，因为我在提交之前就在写状态.因此为了使它起作用，我暂停了整个CSR指令流水线，因此我从不推测这些指令。

  val csr_exe_unit = exe_units.csr_unit             //csr执行单元

  // for critical path reasons, we aren't zero'ing this out if resp is not valid  //由于关键路径原因，如果resp无效，我们不会将其清零
  val csr_rw_cmd = csr_exe_unit.io.iresp.bits.uop.ctrl.csr_cmd
  val wb_wdata = csr_exe_unit.io.iresp.bits.data   //csr执行单元的回复。 将MicroOp与数据捆绑在一起。wdata是啥？

  csr.io.rw.addr        := csr_exe_unit.io.iresp.bits.uop.csr_addr
  csr.io.rw.cmd         := freechips.rocketchip.rocket.CSR.maskCmd(csr_exe_unit.io.iresp.valid, csr_rw_cmd)
  csr.io.rw.wdata       := wb_wdata

  // Extra I/O
  // Delay retire/exception 1 cycle
  //额外的I / O
  //延迟 退休/异常 1个周期
  csr.io.retire    := RegNext(PopCount(rob.io.commit.arch_valids.asUInt))
  csr.io.exception := RegNext(rob.io.com_xcpt.valid)
  // csr.io.pc used for setting EPC during exception or CSR.io.trace.  //csr.io.pc用于在异常或CSR.io.trace期间设置EPC。

  csr.io.pc        := (boom.util.AlignPCToBoundary(io.ifu.get_pc(0).com_pc, icBlockBytes)
                     + RegNext(rob.io.com_xcpt.bits.pc_lob)
                     - Mux(RegNext(rob.io.com_xcpt.bits.edge_inst), 2.U, 0.U))
  // Cause not valid for for CALL or BREAKPOINTs (CSRFile will override it).    //cause对于CALL或BREAKPOINT无效（CSRFile将覆盖它）。
  csr.io.cause     := RegNext(rob.io.com_xcpt.bits.cause)
  csr.io.ungated_clock := clock

  val tval_valid = csr.io.exception &&
    csr.io.cause.isOneOf(
      //Causes.illegal_instruction.U, we currently only write 0x0 for illegal instructions  //Causes.illegal_instruction.U，我们目前仅将0x0写入非法指令
      Causes.breakpoint.U,
      Causes.misaligned_load.U,
      Causes.misaligned_store.U,
      Causes.load_access.U,
      Causes.store_access.U,
      Causes.fetch_access.U,
      Causes.load_page_fault.U,
      Causes.store_page_fault.U,
      Causes.fetch_page_fault.U)

  csr.io.tval := Mux(tval_valid,
    RegNext(encodeVirtualAddress(rob.io.com_xcpt.bits.badvaddr, rob.io.com_xcpt.bits.badvaddr)), 0.U)
   //com_xcpt 将异常传达给CSRFile

  // TODO move this function to some central location (since this is used elsewhere). //将此功能移到某个中心位置（因为此功能已在其他地方使用）。
  def encodeVirtualAddress(a0: UInt, ea: UInt) =
    if (vaddrBitsExtended == vaddrBits) {
      ea
    } else {
      // Efficient means to compress 64-bit VA into vaddrBits+1 bits.
      // (VA is bad if VA(vaddrBits) != VA(vaddrBits-1)).
      //将64位VA压缩为vaddrBits + 1位的有效方法。                  //一种压缩虚拟地址的办法
      //（如果VA（vaddrBits）！= VA（vaddrBits-1），则VA不好）。
      val a = a0.asSInt >> vaddrBits
      val msb = Mux(a === 0.S || a === -1.S, ea(vaddrBits), !ea(vaddrBits-1))
      Cat(msb, ea(vaddrBits-1,0))
    }

  // reading requires serializing the entire pipeline   //阅读需要序列化整个管道，csr令人迷惑
  csr.io.fcsr_flags.valid := rob.io.commit.fflags.valid
  csr.io.fcsr_flags.bits  := rob.io.commit.fflags.bits
  csr.io.set_fs_dirty.get := rob.io.commit.fflags.valid

  exe_units.withFilter(_.hasFcsr).map(_.io.fcsr_rm := csr.io.fcsr_rm)
  io.fcsr_rm := csr.io.fcsr_rm

  if (usingFPU) {
    fp_pipeline.io.fcsr_rm := csr.io.fcsr_rm
  }

  csr.io.hartid := io.hartid
  csr.io.interrupts := io.interrupts

// TODO can we add this back in, but handle reset properly and save us
//      the mux above on csr.io.rw.cmd?
//   assert (!(csr_rw_cmd =/= rocket.CSR.N && !exe_units(0).io.resp(0).valid),
//   "CSRFile is being written to spuriously.")

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Execute Stage ****    //执行阶段
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  iss_idx = 0
  var bypass_idx = 0
  for (w <- 0 until exe_units.length) {
    val exe_unit = exe_units(w)
   //readsIrf： 此执行单元是否需要整数寄存器端口
    if (exe_unit.readsIrf) {
      exe_unit.io.req <> iregister_read.io.exe_reqs(iss_idx)   //iregister_read.io.exe_reqs：将微操作发送到执行管道

            //如果执行单元可绕过
      if (exe_unit.bypassable) {
        for (i <- 0 until exe_unit.numBypassStages) {
          bypasses(bypass_idx) := exe_unit.io.bypass(i)
          bypass_idx += 1
        }          //将执行单元的bypass传递给bypasses
      }
      iss_idx += 1
    }
  }
  require (bypass_idx == exe_units.numTotalBypassPorts)     //要求bypass数量 等于 执行单元总bypass端口。 为啥？
  for (i <- 0 until jmp_unit.numBypassStages) {
    pred_bypasses(i) := jmp_unit.io.bypass(i)
  }



  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Load/Store Unit ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // enqueue basic load/store info in Decode           //使基本的 加载/存储信息入队
  for (w <- 0 until coreWidth) {
    io.lsu.dis_uops(w).valid := dis_fire(w)
    io.lsu.dis_uops(w).bits  := dis_uops(w)
  }
  io.lsu.idle_cycles := idle_cycles.value

  // tell LSU about committing loads and stores to clear entries  //告知LSU有关 committing load和store 以清除条目
  io.lsu.commit                  := rob.io.commit

  // tell LSU that it should fire a load that waits for the rob to clear  //告诉LSU应该发出负载that等待rob清除
  io.lsu.commit_load_at_rob_head := rob.io.com_load_is_at_rob_head
// rob.io.com_load_is_at_rob_head: 告诉LSU，ROB的头是load

  //com_xcpt.valid comes too early, will fight against a branch that resolves same cycle as an exception
  //com_xcpt valid出现得太早，会与在相同的周期解析为异常的分支发生冲突
  io.lsu.exception := RegNext(rob.io.flush.valid)

  // Handle Branch Mispeculations      //处理分支错误预测，用rob信息更新isu信息
  io.lsu.brupdate := brupdate
  io.lsu.rob_head_idx := rob.io.rob_head_idx
  io.lsu.rob_pnr_idx  := rob.io.rob_pnr_idx

  io.lsu.tsc_reg := debug_tsc_reg


  if (usingFPU) {
    io.lsu.fp_stdata <> fp_pipeline.io.to_sdq
  }


  rob.io.risk_pdst := mem_iss_unit.io.risk_pdst
  rob.io.risk_debug_inst := mem_iss_unit.io.risk_debug_inst
  rob.io.risk_rob_idx := mem_iss_unit.io.risk_rob_idx
  rob.io.idle_cycles := idle_cycles.value


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Writeback Stage ****
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  var w_cnt = 1
  iregfile.io.write_ports(0) := WritePort(ll_wbarb.io.out, ipregSz, xLen, RT_FIX)  //ipregSz: 整数物理寄存器的大小
  ll_wbarb.io.in(0) <> mem_resps(0)      //ll_wb似乎与第0个写端口有关系
  assert (ll_wbarb.io.in(0).ready)
   // never backpressure the memory unit.
   //从不backpressure存储单元。
  for (i <- 1 until memWidth) {
    iregfile.io.write_ports(w_cnt) := WritePort(mem_resps(i), ipregSz, xLen, RT_FIX)     //WritePort: 将ExeUnitResps转换为与regfile的WritePort I/Os匹配的实用函数。
    w_cnt += 1
  }

  for (i <- 0 until exe_units.length) {
    if (exe_units(i).writesIrf) {
      val wbresp = exe_units(i).io.iresp  //执行单元返回 赋值给wbresp
      val wbpdst = wbresp.bits.uop.pdst   //写回 目标物理地址
      val wbdata = wbresp.bits.data       //写回数据

      def wbIsValid(rtype: UInt) =
        wbresp.valid && wbresp.bits.uop.rf_wen && wbresp.bits.uop.dst_rtype === rtype     //rf_wen: 这个寄存器能回写吗
      val wbReadsCSR = wbresp.bits.uop.ctrl.csr_cmd =/= freechips.rocketchip.rocket.CSR.N //写回 readCSR

      iregfile.io.write_ports(w_cnt).valid     := wbIsValid(RT_FIX)     //整数寄存器文件写回端口 有效性
      iregfile.io.write_ports(w_cnt).bits.addr := wbpdst                //整数寄存器文件写回端口 目的地址
      wbresp.ready := true.B
          //如果有CSR,则看 是否写回 readCSR
      if (exe_units(i).hasCSR) {
        iregfile.io.write_ports(w_cnt).bits.data := Mux(wbReadsCSR, csr.io.rw.rdata, wbdata)
      } else {
        iregfile.io.write_ports(w_cnt).bits.data := wbdata   //写回数据赋值给寄存器写端口
      }

      assert (!wbIsValid(RT_FLT), "[fppipeline] An FP writeback is being attempted to the Int Regfile.")
        //正在尝试对Int Regfile进行FP回写

      assert (!(wbresp.valid &&
        !wbresp.bits.uop.rf_wen &&
        wbresp.bits.uop.dst_rtype === RT_FIX),
        "[fppipeline] An Int writeback is being attempted with rf_wen disabled.")
   //试图在禁用rf_wen的情况下进行Int回写。 rf_wen:这个寄存器能回写吗

      assert (!(wbresp.valid &&
        wbresp.bits.uop.rf_wen &&
        wbresp.bits.uop.dst_rtype =/= RT_FIX),
        "[fppipeline] writeback being attempted to Int RF with dst != Int type exe_units("+i+").iresp")
             //尝试用dst != Int type来对整数寄存器文件写回
      w_cnt += 1
    }
  }
  require(w_cnt == iregfile.io.write_ports.length)          //寄存器写端口的长度 等于 w_cnt

   //如果启用sfb，则对pregfile赋值
  if (enableSFBOpt) {
    pregfile.io.write_ports(0).valid     := jmp_unit.io.iresp.valid && jmp_unit.io.iresp.bits.uop.is_sfb_br
    pregfile.io.write_ports(0).bits.addr := jmp_unit.io.iresp.bits.uop.pdst
    pregfile.io.write_ports(0).bits.data := jmp_unit.io.iresp.bits.data
  }

       //如果启用fpu，则连接fp_pipeline
  if (usingFPU) {
    // Connect IFPU
    fp_pipeline.io.from_int  <> exe_units.ifpu_unit.io.ll_fresp
    // Connect FPIU
    ll_wbarb.io.in(1)        <> fp_pipeline.io.to_int  //to_int: to 整数寄存器
    // Connect FLDs
    fp_pipeline.io.ll_wports <> exe_units.memory_units.map(_.io.ll_fresp)   //ll_wports： from memory unit
  }
      //启用Rocc，需要同时启用fpu
  if (usingRoCC) {
    require(usingFPU)
    ll_wbarb.io.in(2)       <> exe_units.rocc_unit.io.ll_iresp
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Commit Stage ****      //提交阶段
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  for (w <- 0 until 1) {
    when(ll_wbarb.io.out.bits.uop.debug_inst === 0x02f57663L.U) {
      printf(p" mask=${ll_wbarb.io.out.bits.uop.br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" found-76-exed1 \n")
    }
    when(ll_wbarb.io.out.bits.uop.debug_inst === 0x00054783L.U) {
      printf(p" mask=${ll_wbarb.io.out.bits.uop.br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" found-84-exed1 \n")
    }
    when(ll_wbarb.io.out.bits.uop.debug_inst === 0x93678793L.U) {
      printf(p" mask=${ll_wbarb.io.out.bits.uop.br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" found-7e-exed1 \n")
    }
    when(ll_wbarb.io.out.bits.uop.debug_inst(15,0) === 0x953eL.U) {
      printf(p" mask=${ll_wbarb.io.out.bits.uop.br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" found-82-exed1 \n")
    }
    when(ll_wbarb.io.out.bits.uop.debug_inst === 0x9e478793L.U) {
      printf(p" mask=${ll_wbarb.io.out.bits.uop.br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" found-8c-exed1 \n")
    }
    when(ll_wbarb.io.out.bits.uop.debug_inst(15,0) === 0x97baL.U) {
      printf(p" mask=${ll_wbarb.io.out.bits.uop.br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" found-90-exed1 \n")
    }
    when(ll_wbarb.io.out.bits.uop.debug_inst === 0x0007c783L.U) {
      printf(p" mask=${ll_wbarb.io.out.bits.uop.br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" found-92-exed1 \n")
    }
    when(ll_wbarb.io.out.bits.uop.debug_inst === 0x9af70b23L.U) {
      printf(p" mask=${ll_wbarb.io.out.bits.uop.br_mask}")
      printf(p" found-aa-exed1 \n")
    }
    when(ll_wbarb.io.out.bits.uop.debug_inst(15,0) === 0x631cL.U) {
      printf(p" mask=${ll_wbarb.io.out.bits.uop.br_mask}")
      printf(p" cycles=${idle_cycles.value} ")
      printf(p" found-631c-exed1 \n")
    }
  }
  for (i <- 1 until memWidth) {
    when(mem_resps(i).bits.uop.debug_inst === 0x02f57663L.U) {
      printf(p" found-76-exed2 \n")
    }
    when(mem_resps(i).bits.uop.debug_inst === 0x00054783L.U) {
      printf(p" found-84-exed2 \n")
    }
    when(mem_resps(i).bits.uop.debug_inst === 0x93678793L.U) {
      printf(p" found-7e-exed2 \n")
    }
    when(mem_resps(i).bits.uop.debug_inst(15,0) === 0x953eL.U) {
      printf(p" found-82-exed2 \n")
    }
    when(mem_resps(i).bits.uop.debug_inst === 0x9e478793L.U) {
      printf(p" found-8c-exed2 \n")
    }
    when(mem_resps(i).bits.uop.debug_inst(15,0) === 0x97baL.U) {
      printf(p" found-90-exed2 \n")
    }
    when(mem_resps(i).bits.uop.debug_inst === 0x0007c783L.U) {
      printf(p" found-92-exed2 \n")
    }
    when(mem_resps(i).bits.uop.debug_inst === 0x9af70b23L.U) {
      printf(p" found-aa-exed2 \n")
    }
  }
  for (eu <- exe_units) {
    if (eu.writesIrf) {
      val resp = eu.io.iresp
      when(resp.bits.uop.debug_inst === 0x02f57663L.U) {
        printf(p" mask=${resp.bits.uop.br_mask}")
        printf(p" cycles=${idle_cycles.value} ")
        printf(p" found-76-exed3 \n")
      }
      when(resp.bits.uop.debug_inst === 0x00054783L.U) {
        printf(p" mask=${resp.bits.uop.br_mask}")
        printf(p" cycles=${idle_cycles.value} ")
        printf(p" found-84-exed3 \n")
      }
      when(resp.bits.uop.debug_inst === 0x93678793L.U) {
        printf(p" mask=${resp.bits.uop.br_mask}")
        printf(p" cycles=${idle_cycles.value} ")
        printf(p" found-7e-exed3 \n")
      }
      when(resp.bits.uop.debug_inst(15,0) === 0x953eL.U) {
        printf(p" mask=${resp.bits.uop.br_mask}")
        printf(p" cycles=${idle_cycles.value} ")
        printf(p" found-82-exed3 \n")
      }
      when(resp.bits.uop.debug_inst === 0x9e478793L.U) {
        printf(p" mask=${resp.bits.uop.br_mask}")
        printf(p" cycles=${idle_cycles.value} ")
        printf(p" found-8c-exed3 \n")
      }
      when(resp.bits.uop.debug_inst(15,0) === 0x97baL.U) {
        printf(p" mask=${resp.bits.uop.br_mask}")
        printf(p" cycles=${idle_cycles.value} ")
        printf(p" found-90-exed3 \n")
      }
      when(resp.bits.uop.debug_inst === 0x0007c783L.U) {
        printf(p" mask=${resp.bits.uop.br_mask}")
        printf(p" cycles=${idle_cycles.value} ")
        printf(p" found-92-exed3 \n")
      }
      when(resp.bits.uop.debug_inst === 0x9af70b23L.U) {
        printf(p" mask=${resp.bits.uop.br_mask}")
        printf(p" cycles=${idle_cycles.value} ")
        printf(p" found-aa-exed3 \n")
      }
      when(resp.bits.uop.debug_pc === 0x8000110aL.U) {
        printf(p" mask=${resp.bits.uop.br_mask}")
        printf(p" cycles=${idle_cycles.value} ")
        printf(p" found-main-ret-exed3 \n")
      }
    }
  }

  // Writeback
  // ---------
  // First connect the ll_wport    //首先连接ll_wport
  val ll_uop = ll_wbarb.io.out.bits.uop
  rob.io.wb_resps(0).valid  := ll_wbarb.io.out.valid && !(ll_uop.uses_stq && !ll_uop.is_amo)
  rob.io.wb_resps(0).bits   <> ll_wbarb.io.out.bits
  rob.io.debug_wb_valids(0) := ll_wbarb.io.out.valid && ll_uop.dst_rtype =/= RT_X   //RT_X: not-a-register
  rob.io.debug_wb_wdata(0)  := ll_wbarb.io.out.bits.data
  var cnt = 1
    //然后连接mem_resps
  for (i <- 1 until memWidth) {
    val mem_uop = mem_resps(i).bits.uop
    rob.io.wb_resps(cnt).valid := mem_resps(i).valid && !(mem_uop.uses_stq && !mem_uop.is_amo)
    rob.io.wb_resps(cnt).bits  := mem_resps(i).bits
    rob.io.debug_wb_valids(cnt) := mem_resps(i).valid && mem_uop.dst_rtype =/= RT_X
    rob.io.debug_wb_wdata(cnt)  := mem_resps(i).bits.data
    cnt += 1
  }
  var f_cnt = 0 // rob fflags port index    //rob fflags端口索引
          //最后是执行单元 iresp
  for (eu <- exe_units) {
    if (eu.writesIrf)
    {
      val resp   = eu.io.iresp
      val wb_uop = resp.bits.uop
      val data   = resp.bits.data

      rob.io.wb_resps(cnt).valid := resp.valid && !(wb_uop.uses_stq && !wb_uop.is_amo)
      rob.io.wb_resps(cnt).bits  <> resp.bits
      rob.io.debug_wb_valids(cnt) := resp.valid && wb_uop.rf_wen && wb_uop.dst_rtype === RT_FIX
         //如果执行单元有fflag标识
      if (eu.hasFFlags) {
        rob.io.fflags(f_cnt) <> resp.bits.fflags
        f_cnt += 1
      }
      //如果执行单元有CSR标识 //csr读数据
      if (eu.hasCSR) {
        rob.io.debug_wb_wdata(cnt) := Mux(wb_uop.ctrl.csr_cmd =/= freechips.rocketchip.rocket.CSR.N,
          csr.io.rw.rdata,
          data)
      } else {
        rob.io.debug_wb_wdata(cnt) := data
      }
      cnt += 1
    }
  }

  require(cnt == numIrfWritePorts)    //numIrfWritePorts = 执行单元写端口的数量 + memWidth
            //如果启用fpu，rob io继续更新
  if (usingFPU) {
    for ((wdata, wakeup) <- fp_pipeline.io.debug_wb_wdata zip fp_pipeline.io.wakeups) {
      rob.io.wb_resps(cnt) <> wakeup
      rob.io.fflags(f_cnt) <> wakeup.bits.fflags
      rob.io.debug_wb_valids(cnt) := wakeup.valid
      rob.io.debug_wb_wdata(cnt) := wdata
      cnt += 1
      f_cnt += 1

      assert (!(wakeup.valid && wakeup.bits.uop.dst_rtype =/= RT_FLT),
        "[core] FP wakeup does not write back to a FP register.")
    //FP唤醒不写回FP寄存器

      assert (!(wakeup.valid && !wakeup.bits.uop.fp_val),
        "[core] FP wakeup does not involve an FP instruction.")
      //FP唤醒不涉及FP指令
      //fp_val:是浮点指令（F或D扩展名）？如果不是ld / st，它将把异常位写回到fcsr。
    }
  }

  require (cnt == rob.numWakeupPorts)   //numWakeupPorts: rob唤醒端口数量
  require (f_cnt == rob.numFpuPorts)    //numFpuPorts: rob fpu端口

  // branch resolution     //分支解析
  rob.io.brupdate <> brupdate  //rob分支信息连接

  exe_units.map(u => u.io.status := csr.io.status)   //执行单元status等于csr的状态
  if (usingFPU)
    fp_pipeline.io.status := csr.io.status

  // Connect breakpoint info to memaddrcalcunit    //将断点信息连接到内存地址计算单元
  for (i <- 0 until memWidth) {

    mem_units(i).io.status   := csr.io.status
    mem_units(i).io.bp       := csr.io.bp
    mem_units(i).io.mcontext := csr.io.mcontext
    mem_units(i).io.scontext := csr.io.scontext

  }

  // LSU <> ROB      //LSU连接ROB
  rob.io.lsu_clr_bsy    := io.lsu.clr_bsy                 //lsu_clr_bsy: store的unbusy端口。 +1表示fpstdata .
                                                          //clr_bsy: 收到stdata时store清除繁忙位//memWidth为int，1为fp（以避免fpstdata）
  rob.io.lsu_clr_unsafe := io.lsu.clr_unsafe              //lsu_clr_unsafe:用于将负载/存储 取消标记为投机危险的端口。
                                                          //clr_unsafe:  推测性安全负载（阻拦内存排序失败）
  rob.io.lxcpt          <> io.lsu.lxcpt                   //lsu exception

  assert (!(csr.io.singleStep), "[core] single-step is unsupported.")
  //单步不支持


  //-------------------------------------------------------------
  // **** Flush Pipeline ****
  //-------------------------------------------------------------
  // flush on exceptions, miniexeptions, and after some special instructions   //刷新 异常，miniexception以及遵循一些特殊指令

   //fpu 流水线刷新
  if (usingFPU) {
    fp_pipeline.io.flush_pipeline := RegNext(rob.io.flush.valid)
  }

  for (w <- 0 until exe_units.length) {
    exe_units(w).io.req.bits.kill := RegNext(rob.io.flush.valid)   //将rob的flush信号赋给功能单元kill信号
  }

  assert (!(rob.io.com_xcpt.valid && !rob.io.flush.valid),
    "[core] exception occurred, but pipeline flush signal not set!")
   //发生异常，但未设置管道刷新信号

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Outputs to the External World ****        //输出到外部世界(挂起流水线)
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // detect pipeline freezes and throw error        //检测流水线冻结并引发错误
  val idle_cycles1 = freechips.rocketchip.util.WideCounter(32)             //orR: 或归约运算符

  when (rob.io.commit.valids.asUInt.orR ||
        csr.io.csr_stall ||
        io.rocc.busy ||
        reset.asBool) {
    idle_cycles1 := 0.U
  }
  assert (!(idle_cycles1.value(13)), "Pipeline has hung.")

     //管道已挂起

  if (usingFPU) {
    fp_pipeline.io.debug_tsc_reg := debug_tsc_reg
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // **** Handle Cycle-by-Cycle Printouts ****              //处理逐周期打印输出
  //-------------------------------------------------------------
  //-------------------------------------------------------------
  
  
  if (COMMIT_LOG_PRINTF) {             //debug打印参数：转储commit状态，用于与ISA sim进行比较
    var new_commit_cnt = 0.U

    for (w <- 0 until coreWidth) {
      val priv = RegNext(csr.io.status.prv) // erets change the privilege. Get the old one
      // To allow for diffs against spike :/
      //允许针对spike的差异
           //打印指令                         //微操作是否是RVC指令
      def printf_inst(uop: MicroOp) = {
        when (uop.is_rvc) {
          printf(p" cycles=${idle_cycles.value} ")
          printf("(0x%x)", uop.debug_inst(15,0))
        } .otherwise {
          printf(p" cycles=${idle_cycles.value} ")
          printf("(0x%x)", uop.debug_inst)
        }
      }

       //rob的commit的架构有效
      when (rob.io.commit.arch_valids(w)) {
        printf("%d 0x%x ",
          priv,
          Sext(rob.io.commit.uops(w).debug_pc(vaddrBits-1,0), xLen))
        printf_inst(rob.io.commit.uops(w))

        //分为commit的微操作是整数还是浮点类型，打印出他们的逻辑目的寄存器（ldst），以及debug_wdata（写回的写数据）
        when (rob.io.commit.uops(w).dst_rtype === RT_FIX && rob.io.commit.uops(w).ldst =/= 0.U) {
          printf(" x%d 0x%x\n",
            rob.io.commit.uops(w).pdst,
            rob.io.commit.debug_wdata(w))

            
        } .elsewhen (rob.io.commit.uops(w).dst_rtype === RT_FLT) {
          printf(" f%d 0x%x\n",
            rob.io.commit.uops(w).pdst,
            rob.io.commit.debug_wdata(w))



          ///////////////////////////////////////debug-hezhuo////////////////////////////
          /*val ss=this.toString  //新增
          when(rob.io.commit.uops(w).ldst === 1.U) {
            printf( p"$ss" )  //新增
            printf(p" jmp_unit_idx2=$jmp_unit_idx\n ") //新增
            printf(p" coreWidth2=$coreWidth\n ") //新增
          }*/
          ////////////////////////////////////////////////////////////////////////////////



        } .otherwise {
          printf("\n")
        }
      }
    }
  } else if (BRANCH_PRINTF) {
    val debug_ghist = RegInit(0.U(globalHistoryLength.W))
    //转储分支预测结果 //全局历史表

    //rob刷新并且type === xcpt.U || type === eret.U
    when (rob.io.flush.valid && FlushTypes.useCsrEvec(rob.io.flush.bits.flush_typ)) {
      debug_ghist := 0.U
    }

    var new_ghist = debug_ghist

    for (w <- 0 until coreWidth) {
      when (rob.io.commit.arch_valids(w) &&
        (rob.io.commit.uops(w).is_br || rob.io.commit.uops(w).is_jal || rob.io.commit.uops(w).is_jalr)) {
        // for (i <- 0 until globalHistoryLength) {
        //   printf("%x", new_ghist(globalHistoryLength-i-1))
        // }
        // printf("\n")
        // 打印：哪种预测结构可根据此操作提供预测、分支是否预测发生、是否是br\jar\jalr，debug_pc
        printf("%x %x %x %x %x %x\n",
          rob.io.commit.uops(w).debug_fsrc, rob.io.commit.uops(w).taken,
          rob.io.commit.uops(w).is_br, rob.io.commit.uops(w).is_jal,
          rob.io.commit.uops(w).is_jalr, Sext(rob.io.commit.uops(w).debug_pc(vaddrBits-1,0), xLen))

      }
      //根据is_br、taken来更新new_ghist
      new_ghist = Mux(rob.io.commit.arch_valids(w) && rob.io.commit.uops(w).is_br,
        Mux(rob.io.commit.uops(w).taken, new_ghist << 1 | 1.U(1.W), new_ghist << 1),
        new_ghist)
    }
    debug_ghist := new_ghist
  }

  // TODO: Does anyone want this debugging functionality?
  val coreMonitorBundle = Wire(new CoreMonitorBundle(xLen, fLen))

  coreMonitorBundle := DontCare
  coreMonitorBundle.clock  := clock
  coreMonitorBundle.reset  := reset


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Page Table Walker            //将csr的ptw信息赋给io.ptw

  io.ptw.ptbr       := csr.io.ptbr
  io.ptw.status     := csr.io.status
  io.ptw.pmp        := csr.io.pmp
  io.ptw.sfence     := io.ifu.sfence

  //-------------------------------------------------------------
  //-------------------------------------------------------------

  io.rocc := DontCare
  io.rocc.exception := csr.io.exception && csr.io.status.xs.orR
     //设置rocc执行单元
  if (usingRoCC) {
    exe_units.rocc_unit.io.rocc.rocc         <> io.rocc
    exe_units.rocc_unit.io.rocc.dis_uops     := dis_uops
    exe_units.rocc_unit.io.rocc.rob_head_idx := rob.io.rob_head_idx
    exe_units.rocc_unit.io.rocc.rob_pnr_idx  := rob.io.rob_pnr_idx
    exe_units.rocc_unit.io.com_exception     := rob.io.flush.valid
    exe_units.rocc_unit.io.status            := csr.io.status

    for (w <- 0 until coreWidth) {
      exe_units.rocc_unit.io.rocc.dis_rocc_vals(w) := (
        dis_fire(w) &&
        dis_uops(w).uopc === uopROCC &&
        !dis_uops(w).exception
      )
    }
  }

  if (usingTrace) {
    for (w <- 0 until coreWidth) {
      // Delay the trace so we have a cycle to pull PCs out of the FTQ   //延迟跟踪，以便我们有一个周期将PC从FTQ中拉出
      io.trace(w).valid      := RegNext(rob.io.commit.arch_valids(w))

      // Recalculate the PC    //重新计算PC
      io.ifu.debug_ftq_idx(w) := rob.io.commit.uops(w).ftq_idx
      val iaddr = (AlignPCToBoundary(io.ifu.debug_fetch_pc(w), icBlockBytes)
                   + RegNext(rob.io.commit.uops(w).pc_lob)
                   - Mux(RegNext(rob.io.commit.uops(w).edge_inst), 2.U, 0.U))(vaddrBits-1,0)
      io.trace(w).iaddr      := Sext(iaddr, xLen)

      def getInst(uop: MicroOp, inst: UInt): UInt = {
        Mux(uop.is_rvc, Cat(0.U(16.W), inst(15,0)), inst)
      }

      def getWdata(uop: MicroOp, wdata: UInt): UInt = {
        Mux((uop.dst_rtype === RT_FIX && uop.ldst =/= 0.U) || (uop.dst_rtype === RT_FLT), wdata, 0.U(xLen.W))
      }

      // use debug_insts instead of uop.debug_inst to use the rob's debug_inst_mem
      // note: rob.debug_insts comes 1 cycle later
      //使用debug_insts而不是uop.debug_inst来使用rob的debug_inst_mem
      //注意：rob.debug_insts在1个周期之后
      io.trace(w).insn       := getInst(RegNext(rob.io.commit.uops(w)), rob.io.commit.debug_insts(w))
      io.trace(w).wdata.map { _ := RegNext(getWdata(rob.io.commit.uops(w), rob.io.commit.debug_wdata(w))) }

      // Comment out this assert because it blows up FPGA synth-asserts
      // This tests correctedness of the debug_inst mem
      // when (RegNext(rob.io.commit.valids(w))) {
      //   assert(rob.io.commit.debug_insts(w) === RegNext(rob.io.commit.uops(w).debug_inst))
      // }
      // This tests correctedness of recovering pcs through ftq debug ports
      // when (RegNext(rob.io.commit.valids(w))) {
      //   assert(Sext(io.trace(w).iaddr, xLen) ===
      //     RegNext(Sext(rob.io.commit.uops(w).debug_pc(vaddrBits-1,0), xLen)))
      // }

      // These csr signals do not exactly match up with the ROB commit signals.   //这些csr信号与ROB commit信号不完全匹配。
      io.trace(w).priv       := RegNext(csr.io.status.prv)
      // Can determine if it is an interrupt or not based on the MSB of the cause //可以根据cause的MSB确定是否为中断
      io.trace(w).exception  := RegNext(rob.io.com_xcpt.valid && !rob.io.com_xcpt.bits.cause(xLen - 1))
      io.trace(w).interrupt  := RegNext(rob.io.com_xcpt.valid && rob.io.com_xcpt.bits.cause(xLen - 1))
      io.trace(w).cause      := RegNext(rob.io.com_xcpt.bits.cause)
      io.trace(w).tval       := RegNext(csr.io.tval)
    }
    dontTouch(io.trace)
  } else {
    io.trace := DontCare
    io.trace map (t => t.valid := false.B)
    io.ifu.debug_ftq_idx := DontCare
  }
}
